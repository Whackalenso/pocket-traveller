// power_test.ino — Pico + ADXL345 (I2C 0x1D) + SH1106 SPI OLED
//
// Motion classifier + sprite playback, plus power behavior:
//   * Charging: USB 5V (TP4056 IN+) divided down and read on CHARGE_PIN.
//     Plugged in -> "charging" screen, which blanks after CHARGE_SCREEN_MS and
//     comes back on a nudge, until unplugged.
//   * Downtime: after DEEPSLEEP_AFTER_MS idle -> blank the OLED, then stop the
//     chip in RP2040 DORMANT until the ADXL345 activity interrupt wakes it.
//     (The shallower idle-sleep animation that used to precede this is
//     compiled out while there is no sleep art -- see IDLE_SLEEP_ENABLED.)
//
// WIRING
//   ADXL345:  VCC->3V3  GND->GND  SDA->GP0  SCL->GP1
//             CS ->3V3  (REQUIRED: CS low puts it in SPI mode)
//             SDO->3V3  (gives address 0x1D; SDO->GND would be 0x53)
//             INT1->GP8 (only needed if WAKE_ON_INT1 is 1)
//   OLED:     CLK->GP2  MOSI->GP3  RES->GP4  DC->GP5  CS->GP6
//   CHARGE:   TP4056 IN+ --[680R]--+--[1k]-- GND, tap junction -> GP7
//             (gives ~2.98 V at 5 V in). Shared GND with Pico REQUIRED.
//
// ADXL345 gotchas baked in below:
//   * DEVID (0x00) must read 0xE5.
//   * Boots in STANDBY; POWER_CTL (0x2D) bit 3 must be set or data is frozen.
//   * Full-res is 3.9 mg/LSB, RIGHT-justified -> no >>4 shift.
//   * Counts are scaled to milli-g so 1 g = 1000, matching old tuning.

#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>
#include "sprites.h"

#define ADDR 0x1D

// ---- Wake source selector ---------------------------------------------------
// 1 = wake from downtime using the ADXL345 hardware activity interrupt (INT1).
// 0 = wake by polling the accelerometer in software (no INT1 wire needed).
// Start at 0 to prove the sleep flow works, then try 1 to test the interrupt.
#define WAKE_ON_INT1 1

// ---- Deep sleep selector ----------------------------------------------------
// 1 = downtime really stops the chip (RP2040 DORMANT: every clock off, wake
//     only on a GPIO level). Microamps.
// 0 = downtime spins in __wfi() waiting for the pin. Correct behavior, but the
//     core stays clocked -- milliamps. This is the fallback if dormant misbehaves
//     on real hardware; nothing else in the sketch changes.
#define TRUE_DEEP_SLEEP 1

#if TRUE_DEEP_SLEEP
#if !WAKE_ON_INT1
#error "TRUE_DEEP_SLEEP requires WAKE_ON_INT1=1: a dormant chip cannot poll the ADXL345."
#endif
#if !defined(PICO_RP2040)
#error "The dormant path is RP2040-only (xosc_dormant + clk_rtc do not exist on RP2350)."
#endif
#include "pico/runtime_init.h"          // clocks_init()
#include "hardware/clocks.h"            // clock_configure, clock_stop, set_sys_clock_khz
#include "hardware/gpio.h"              // gpio_set_dormant_irq_enabled
#include "hardware/pll.h"               // pll_deinit
#include "hardware/xosc.h"              // xosc_dormant
#include "hardware/regs/rosc.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/rosc.h"
#endif

// ---------------- power pins -------------------------------------------------
static const uint8_t CHARGE_PIN = 11;   // HIGH = USB plugged into TP4056
static const uint8_t INT1_PIN   = 15;   // ADXL345 INT1 (active HIGH on activity)

// Idle time before the Pico drops into the sleep wait.
// Set short while testing; raise for real use.
static const unsigned long DEEPSLEEP_AFTER_MS = 10UL * 60UL * 1000UL;   // 10 minutes

// How long the charging screen stays lit before it blanks. A charge is hours
// long and this panel is a mono OLED, so a permanently lit "charging" would
// eventually ghost into every frame drawn after it.
static const unsigned long CHARGE_SCREEN_MS = 10UL * 1000UL;            // 10 seconds

// ======================= TUNE THESE =========================================
float ENERGY_GATE = 1000.0f;  // variance in (milli-g)^2. 1 g = 1000 mg.
float RHYTHM_TH   = 0.2f;     // autocorr peak 0..1 in the step band.

// Short-window energy that means "a hand is on this thing right now".
// Same units as ENERGY_GATE but measured over ~200 ms instead of 2.5 s, so it
// reacts before the device reaches your face. ~4000 is ~63 mg RMS of wobble:
// well above resting sensor noise, well below a deliberate pickup.
float HANDLE_GATE = 4000.0f;

// ---- Idle sleep -------------------------------------------------------------
// Idle sleep is the shallow one: the pet dozes off with its own sprite still on
// screen after IDLE_SLEEP_MS, well before DEEPSLEEP_AFTER_MS blanks the panel
// entirely. It is off right now because no rank has sleep art -- there is
// nothing to draw. To bring it back: put sleep_0.png (and any further frames)
// in every art/<rank>/ folder, re-run
//     python gen_sprites.py art sprites.h
// and the generator flips SPRITES_HAVE_SLEEP to 1, which re-enables everything
// guarded below. Nothing else has to change.
#define IDLE_SLEEP_ENABLED SPRITES_HAVE_SLEEP

#if IDLE_SLEEP_ENABLED
// Idle time before rest -> sleep animation (must be < DEEPSLEEP_AFTER_MS).
static const unsigned long IDLE_SLEEP_MS = 30UL * 1000UL;   // 30 s
#endif

// Software motion-wake threshold, in milli-g away from 1 g rest.
static const float WAKE_MOTION_MG = 1000.0f;

// ADXL345 activity threshold, 62.5 mg/LSB. 8 -> ~500 mg. Lower = touchier.
static const uint8_t ACT_THRESH = 8;

static const uint8_t SPRITE_SCALE = 3;
// ============================================================================

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);


// ---------------- persistent state -------------------------------------------
// XP and rank live in the last 4 KB flash sector, mirrored in RAM by the core's
// EEPROM emulation. commit() erases that whole sector, so it is called only at
// the charging / downtime transitions, never from loop().

static const uint8_t  RANK_HOURS = 72;        // rank window = 3 days

// Level curve. Reaching level L costs LEVEL_A * (L-1)^LEVEL_P seconds of
// motion. The exponent is what sets the shape: it is fixed by wanting the
// year target to be only ~52x the week target while the level number goes up
// 10x, i.e. p = ln(365/7) / ln(99/9) = 1.65. LEVEL_A then falls out of the
// week target and lands on a round 1000 s.
//
// Against a typical campus day of ~90 min of walking that gives:
//   lvl 2 = 17 min of motion     lvl 10 = 10.4 h   (1 week)
//   lvl 20 = 36 h  (3.4 weeks)   lvl 100 = 545 h   (1 year)
// Per-level cost grows from 17 min at the bottom to ~9 h at level 100.
// If real wearers move more or less than that, scale LEVEL_A and leave the
// exponent alone: it is what holds the week:year ratio, at any activity level.
static const float LEVEL_A = 1000.0f;         // seconds of motion for level 2
static const float LEVEL_P = 1.65f;

// The rank order here is also the sprite order: RANK_SPRITES[] in the generated
// sprites.h is indexed by these values, and RANKS in gen_sprites.py is the same
// list. The static_assert below catches the two drifting apart.
enum Rank : uint8_t { RANK_CHUD = 0, RANK_PLEB, RANK_NOMAD, RANK_COUNT };
static const char* const RANK_NAME[] = {"chud", "pleb", "nomad"};
static_assert(RANK_SPRITE_COUNT == RANK_COUNT,
              "sprites.h has a different number of ranks than enum Rank");

// Rank boundaries, in active seconds inside the 72 h window. The DOWN values
// sit below the UP values on purpose: without that gap the rank flickers every
// time you hover on a threshold.
static const uint32_t PLEB_UP  = 3UL * 3600, PLEB_DOWN  = 2UL * 3600;   // ~1 h/day
static const uint32_t NOMAD_UP = 9UL * 3600, NOMAD_DOWN = 7UL * 3600;   // ~3 h/day

// A rank change is announced with a card, but almost never at the moment it
// happens: the window rolls on the hour, usually while the device is in a
// pocket or on a desk. So updateRank() only records that one is owed, and the
// card is shown at the next time we know someone is looking -- a pickup or a
// wake. Declared up here because updateRank() runs long before the card code.
int8_t  pendingRankDir = 0;      // 0 none, +1 promoted, -1 demoted
uint8_t pendingRankTo  = 0;      // rank to name in the card

struct SaveData {
  uint32_t magic;                 // virgin flash reads 0xFF..., so this catches it
  uint16_t version;
  uint8_t  rank;
  uint8_t  checksum;
  uint32_t xp;                    // lifetime active seconds -- only ever grows
  uint32_t clockSec;              // seconds this device has ever been powered
  uint32_t lastHour;              // clockSec/3600 when the ring last rolled
  uint16_t bucket[RANK_HOURS];    // active seconds per hour of the window
};                                // 164 bytes, no padding

static const uint32_t SAVE_MAGIC   = 0x32474D54UL;   // "TMG2"
static const uint16_t SAVE_VERSION = 2;
static const int      SAVE_ADDR    = 0;

// The "today" readout is the newest 24 hours of the same ring the rank uses.
// There is no real-time clock on this board, so it is a rolling 24 h rather
// than since-midnight: the hour ring is the only calendar the device has.
static const uint8_t TODAY_HOURS = 24;

SaveData save;
bool     saveDirty    = false;
uint16_t cachedLevel  = 1;        // levelFromXP costs a few powf; recompute on change
uint8_t  cachedProgress = 0;      // 0..255 of the way to the next level
uint32_t cachedTodaySec = 0;      // summing 24 buckets per redraw is pointless
uint32_t clockLastMs  = 0;
uint16_t clockCarryMs = 0;

static uint8_t saveChecksum(SaveData s) {   // by value: zero the field, sum the rest
  s.checksum = 0;
  const uint8_t* p = (const uint8_t*)&s;
  uint8_t c = 0;
  for (size_t i = 0; i < sizeof(SaveData); i++) c += p[i];
  return c;
}

// Total motion needed to stand at the start of a level. Level 1 is free.
static uint32_t xpForLevel(uint16_t lvl) {
  if (lvl <= 1) return 0;
  return (uint32_t)(LEVEL_A * powf((float)(lvl - 1), LEVEL_P));
}

// The curve inverts in closed form, so this is one powf rather than a walk up
// the ladder. powf is only good to ~7 digits, so a level sitting exactly on a
// boundary can come back one off; the two loops below run zero or one step to
// put it back. Also yields how far into the level we are, for the HUD bar.
uint16_t levelFromXP(uint32_t xp, uint8_t* progress = nullptr) {
  uint32_t lvl = 1 + (uint32_t)powf((float)xp / LEVEL_A, 1.0f / LEVEL_P);
  if (lvl > 9999) lvl = 9999;
  while (lvl > 1    && xp <  xpForLevel(lvl))     lvl--;
  while (lvl < 9999 && xp >= xpForLevel(lvl + 1)) lvl++;

  if (progress) {
    const uint32_t lo = xpForLevel(lvl), hi = xpForLevel(lvl + 1);
    *progress = (hi > lo) ? (uint8_t)(((xp - lo) * 255UL) / (hi - lo)) : 255;
  }
  return (uint16_t)lvl;
}

void loadSave() {
  EEPROM.begin(512);                          // RAM mirror; commit erases 4 KB
  EEPROM.get(SAVE_ADDR, save);
  if (save.magic != SAVE_MAGIC || save.version != SAVE_VERSION ||
      save.checksum != saveChecksum(save)) {
    memset(&save, 0, sizeof(save));           // first boot, corrupt, or old layout
    save.magic   = SAVE_MAGIC;
    save.version = SAVE_VERSION;
    save.rank    = RANK_CHUD;
    saveDirty    = true;
    Serial.println("save: fresh");
  }
  if (save.rank > RANK_NOMAD) save.rank = RANK_CHUD;  // never index RANK_NAME OOB
  cachedLevel = levelFromXP(save.xp, &cachedProgress);
  clockLastMs = millis();

  Serial.print("save: level="); Serial.print(cachedLevel);
  Serial.print(" xp=");         Serial.print(save.xp);
  Serial.print(" rank=");       Serial.println(RANK_NAME[save.rank]);
}

// Erases a flash sector: ~30 ms with interrupts off. Transitions only.
void commitSave() {
  if (!saveDirty) return;
  save.checksum = saveChecksum(save);
  EEPROM.put(SAVE_ADDR, save);
  if (EEPROM.commit()) saveDirty = false;
}

uint32_t recentActiveSec() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < RANK_HOURS; i++) total += save.bucket[i];
  return total;
}

// Walks backwards from the hour currently being filled, so the newest bucket
// is always included and the oldest 48 h of the rank window are not.
uint32_t todayActiveSec() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < TODAY_HOURS; i++)
    total += save.bucket[(save.lastHour + RANK_HOURS - i) % RANK_HOURS];
  return total;
}

void updateRank() {
  const uint32_t r = recentActiveSec();
  const Rank cur = (Rank)save.rank;
  Rank target = (r >= NOMAD_UP) ? RANK_NOMAD : (r >= PLEB_UP) ? RANK_PLEB : RANK_CHUD;
  if (target < cur) {                          // falling: require the lower band
    if (cur == RANK_NOMAD && r >= NOMAD_DOWN) target = RANK_NOMAD;
    if (cur == RANK_PLEB  && r >= PLEB_DOWN)  target = RANK_PLEB;
  }
  if (target != cur) {
    save.rank = target;
    saveDirty = true;
    // Queue the announcement. If one is already owed and undelivered, the newer
    // change replaces it: the card should describe where the pet ended up, not
    // a transition the wearer never saw.
    pendingRankDir = (target > cur) ? +1 : -1;
    pendingRankTo  = (uint8_t)target;
    Serial.print("rank -> "); Serial.println(RANK_NAME[target]);
  }
}

// Advance the persistent clock and roll the hour ring. Called at the top of
// loop(), so the delta also covers the charging wait, which polls and therefore
// keeps millis() running.
//
// Downtime sleep is different: with TRUE_DEEP_SLEEP the timer is stopped along
// with everything else, so a night in dormant adds roughly zero here. clockSec
// is therefore awake-seconds, not wall-clock seconds, and the 72 h rank window
// and 24 h "today" readout roll on awake time too -- activity ages out while
// the device is being carried, not while it sits in a drawer. There is no RTC
// on this board and dormant stops every oscillator, so there is nothing left
// running to measure the gap with. The one thing this does NOT do is warp:
// millis() comes back where it left off, so the delta below stays small and
// the ring never takes a bogus multi-hour jump on wake.
void advanceClock() {
  const uint32_t now = millis();
  uint32_t d = (now - clockLastMs) + clockCarryMs;   // unsigned: wrap-safe
  clockLastMs  = now;
  clockCarryMs = d % 1000;
  const uint32_t sec = d / 1000;
  if (!sec) return;

  save.clockSec += sec;
  const uint32_t h = save.clockSec / 3600;
  if (h != save.lastHour) {
    uint32_t steps = h - save.lastHour;
    if (steps > RANK_HOURS) steps = RANK_HOURS;      // long gap: wipe the ring
    for (uint32_t i = 1; i <= steps; i++)
      save.bucket[(save.lastHour + i) % RANK_HOURS] = 0;
    save.lastHour = h;
    saveDirty = true;
  }
  updateRank();
}

// One second of movement: feeds both the lifetime counter and the rank window.
void addActiveSecond() {
  save.xp++;
  cachedLevel = levelFromXP(save.xp, &cachedProgress);
  uint16_t& b = save.bucket[save.lastHour % RANK_HOURS];
  if (b < 65535) b++;
  saveDirty = true;
}

// ---------------- sensor -----------------------------------------------------

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission();          // full stop, not repeated start
  Wire.requestFrom(ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

// ADXL345 full-resolution scale: 3.9 mg per LSB -> magnitude in milli-g.
static const float LSB_TO_MG = 3.9f;

void readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x32);                // DATAX0
  Wire.endTransmission();
  Wire.requestFrom(ADDR, (uint8_t)6);
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.available() ? Wire.read() : 0;
  x = (int16_t)(b[0] | (b[1] << 8));
  y = (int16_t)(b[2] | (b[3] << 8));
  z = (int16_t)(b[4] | (b[5] << 8));
}

// Magnitude of the current sample, in milli-g (~1000 at rest).
float readMagMg() {
  int16_t x, y, z;
  readXYZ(x, y, z);
  return sqrtf((float)x * x + (float)y * y + (float)z * z) * LSB_TO_MG;
}

// Normal data-reading configuration used by the classifier.
void enableNormalMode() {
  writeReg(0x2D, 0x00);   // standby while reconfiguring
  writeReg(0x31, 0x08);   // DATA_FORMAT: full res, +/-2g, INT active HIGH
  writeReg(0x2C, 0x0A);   // BW_RATE: 100 Hz output data rate
  writeReg(0x2E, 0x00);   // INT_ENABLE: no interrupts during normal running
  writeReg(0x2D, 0x08);   // POWER_CTL: measurement mode (REQUIRED)
}

// Arm the ADXL345 activity interrupt on INT1.
// ACT_INACT_CTL bit 7 selects ac-coupled activity, which subtracts a running
// reference so gravity does NOT sit permanently over the threshold. That was
// the bug in the LIS3DH attempt; here it is one bit.
void enableMotionWake() {
  writeReg(0x2D, 0x00);        // standby while configuring
  writeReg(0x24, ACT_THRESH);  // THRESH_ACT, 62.5 mg/LSB
  writeReg(0x27, 0xF0);        // ACT_INACT_CTL: ac-coupled, X/Y/Z activity on
  writeReg(0x2F, 0x00);        // INT_MAP: all interrupts -> INT1 pin
  writeReg(0x2E, 0x10);        // INT_ENABLE: activity only
  writeReg(0x2D, 0x08);        // measurement mode
  (void)readReg(0x30);         // INT_SOURCE read clears pending interrupt
}

// Software motion check: how far magnitude strays from 1 g.
bool motionDetected() {
  return fabsf(readMagMg() - 1000.0f) > WAKE_MOTION_MG;
}

// ---------------- classifier -------------------------------------------------

const int   ODR_HZ   = 50;
const int   WIN      = 128;
const int   HOP      = 25;
const int   LAG_LO   = 12;
const int   LAG_HI   = 36;
const int   SMOOTH_N = 2;

float bufv[WIN];
int   head = 0, filled = 0, sinceHop = 0;

const char* state        = "IDLE";
const char* pendingState = "IDLE";
int         pendingCount = 0;

float lastVar = 0, lastRhythm = 0;

unsigned long idleSince = 0;

void pushSample(float mag) {
  bufv[head] = mag;
  head = (head + 1) % WIN;
  if (filled < WIN) filled++;
}

void windowFeatures(float &variance, float &rhythm) {
  static float d[WIN];
  float mean = 0;
  for (int i = 0; i < WIN; i++) d[i] = bufv[(head + i) % WIN];
  for (int i = 0; i < WIN; i++) mean += d[i];
  mean /= WIN;
  float energy = 0;
  for (int i = 0; i < WIN; i++) { d[i] -= mean; energy += d[i] * d[i]; }
  variance = energy / WIN;
  rhythm = 0;
  if (energy <= 0) return;
  for (int lag = LAG_LO; lag <= LAG_HI; lag++) {
    float s = 0;
    for (int i = 0; i < WIN - lag; i++) s += d[i] * d[i + lag];
    float c = s / energy;
    if (c > rhythm) rhythm = c;
  }
}

const char* classify(float variance, float rhythm) {
  if (variance < ENERGY_GATE) return "IDLE";
  return (rhythm > RHYTHM_TH) ? "MOVING" : "IDLE";
}

// ---------------- handled ("someone picked this up") -------------------------
// classify() folds high-energy-but-arrhythmic motion into IDLE, so a pickup and
// a device face-down on a desk look identical to it. They are not: a pickup is
// the one moment we know the screen is about to be looked at. Detect it off a
// much shorter window than the classifier uses, because a 2.5 s window plus the
// SMOOTH_N debounce would resolve about a second after the device already
// reached your face.

static const int      SHORT_WIN         = 10;    // ~200 ms at 50 Hz
static const uint32_t HANDLE_RELEASE_MS = 800;   // quiet time before "put down"
static const uint32_t STILL_BEFORE_MS   = 1500;  // stillness that makes it a pickup

bool     handled        = false;
uint32_t handledLastMs  = 0;   // last sample whose short window was over the gate
uint32_t unhandledSince = 0;   // when the device last went quiet

// Variance of the newest SHORT_WIN samples. Cheap enough to run every sample.
float shortVar() {
  if (filled < SHORT_WIN) return 0;
  float mean = 0;
  for (int i = 1; i <= SHORT_WIN; i++) mean += bufv[(head - i + WIN) % WIN];
  mean /= SHORT_WIN;
  float energy = 0;
  for (int i = 1; i <= SHORT_WIN; i++) {
    const float d = bufv[(head - i + WIN) % WIN] - mean;
    energy += d * d;
  }
  return energy / SHORT_WIN;
}

// ---------------- sprite drawing ---------------------------------------------

static void drawXBMPScaled(u8g2_uint_t x, u8g2_uint_t y,
                           uint8_t w, uint8_t h, const uint8_t* bits) {
  if (SPRITE_SCALE <= 1) {
    u8g2.drawXBMP(x, y, w, h, bits);
    return;
  }
  const uint8_t bytesPerRow = (w + 7) / 8;
  for (uint8_t row = 0; row < h; row++) {
    for (uint8_t col = 0; col < w; col++) {
      const uint8_t b = bits[row * bytesPerRow + (col >> 3)];
      if (b & (1 << (col & 7))) {
        u8g2.drawBox(x + col * SPRITE_SCALE, y + row * SPRITE_SCALE,
                     SPRITE_SCALE, SPRITE_SCALE);
      }
    }
  }
}

// Sprites are not all one size: chud is 18x18, pleb 16x16, nomad 16x32 (and
// nomad's walk is one pixel taller than its rest). So a frame is placed by its
// feet rather than its box -- horizontally centered, bottom row always landing
// SPRITE_BOTTOM_PAD above the end of the panel. A taller pet grows upward into
// the empty space under the header instead of sinking through the floor, and
// nothing shifts vertically when the rank changes or the walk cycle bobs.
//
// 8 px is where the old fixed 16x16 sprite ended up when it was centered in the
// lower half of the 128 px panel, so the pet stands exactly where it used to.
static const u8g2_uint_t SPRITE_BOTTOM_PAD = 8;

static void drawAnimFrame(const Animation* a, uint8_t frame) {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();
  const u8g2_uint_t sw = (u8g2_uint_t)a->w * SPRITE_SCALE;
  const u8g2_uint_t sh = (u8g2_uint_t)a->h * SPRITE_SCALE;
  const u8g2_uint_t base = dh - SPRITE_BOTTOM_PAD;   // y of the row under the feet

  // Unsigned math: a sprite wider or taller than the panel pins to the edge
  // instead of wrapping around to a huge coordinate.
  const u8g2_uint_t x = (dw > sw)   ? (u8g2_uint_t)((dw - sw) / 2) : 0;
  const u8g2_uint_t y = (base > sh) ? (u8g2_uint_t)(base - sh)     : 0;
  drawXBMPScaled(x, y, a->w, a->h, a->frames[frame]);
}

struct AnimPlayer {
  const Animation* anim = nullptr;
  uint8_t  frame = 0;
  uint32_t lastTick = 0;

  void play(const Animation* a, uint32_t now) {
    if (a == anim) return;
    anim = a;
    frame = 0;
    lastTick = now;
  }

  void update(uint32_t now) {
    if (!anim || now - lastTick < anim->frameMs) return;
    lastTick = now;
    if (++frame >= anim->frameCount)
      frame = anim->loop ? 0 : anim->frameCount - 1;
  }

  void draw() const {
    if (anim) drawAnimFrame(anim, frame);
  }
};

AnimPlayer player;

// The sprite set for the rank we are currently at. Bounds-checked because
// save.rank comes out of flash.
const RankSprites& rankSprites() {
  return RANK_SPRITES[(save.rank < RANK_SPRITE_COUNT) ? save.rank : RANK_CHUD];
}

Activity activityFromMotion(unsigned long now) {
  if (strcmp(state, "MOVING") == 0) return WALK;
#if IDLE_SLEEP_ENABLED
  if (idleSince != 0 && (now - idleSince) >= IDLE_SLEEP_MS) return SLEEP;
#else
  (void)now;
#endif
  return REST;
}

const Animation* animForActivity(Activity a) {
  const RankSprites& s = rankSprites();
  switch (a) {
    case WALK:  return s.walk;
#if IDLE_SLEEP_ENABLED
    case SLEEP: return s.sleep ? s.sleep : s.rest;
#endif
    case REST:
    default:    return s.rest;
  }
}

// "level 3 chud" across the top, with a progress bar to the next level under
// it. 4x6 keeps the longest realistic string inside the 64 px width of the
// rotated panel; anything wider is left-aligned instead of centered so it
// clips off the right rather than both ends.
// Padding keeps the text and bar off the panel edges instead of bleeding into
// the bezel. PAD_X eats into the usable text width, so it stays small: 4 px a
// side still leaves 14 characters of 4x6 across the 64 px panel.
static const u8g2_uint_t PAD_TOP = 8;
static const u8g2_uint_t PAD_X   = 4;

static const u8g2_uint_t TEXT_BASE = PAD_TOP + 6;   // 4x6 baseline
static const u8g2_uint_t BAR_Y = PAD_TOP + 8;   // sprite starts at y=72, so this is clear
static const u8g2_uint_t BAR_H = 5;

static void drawHeader() {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t bw = dw - 2 * PAD_X;   // padded width shared by text and bar

  char line[24];
  snprintf(line, sizeof(line), "lvl %u %s",
           (unsigned)cachedLevel, RANK_NAME[save.rank]);
  u8g2.setFont(u8g2_font_4x6_tf);
  const u8g2_uint_t w = u8g2.getStrWidth(line);
  u8g2.drawStr((w < bw) ? PAD_X + (bw - w) / 2 : 0, TEXT_BASE, line);

  // Outline the padded width, fill the inside proportionally. The fill is
  // forced to at least 1 px once any progress exists, so the bar never reads as
  // empty right after a level-up.
  u8g2.drawFrame(PAD_X, BAR_Y, bw, BAR_H);
  const u8g2_uint_t inner = bw - 2;
  u8g2_uint_t fill = (u8g2_uint_t)(((uint32_t)inner * cachedProgress) / 255);
  if (fill == 0 && cachedProgress > 0) fill = 1;
  if (fill) u8g2.drawBox(PAD_X + 1, BAR_Y + 1, fill, BAR_H - 2);
}

// ---------------- flavor text cards ------------------------------------------
// A card takes over the sprite area for a few seconds to narrate whatever the
// pet is supposedly up to. Fired on pickup and on waking from downtime, i.e.
// only at moments we have reason to believe someone is actually looking.
//
// The pool itself lives in card_lines.h -- that file is pure data (string
// literals and commas), pasted in here by the preprocessor, so adding a quip
// never means touching this sketch.
static const char* const CARD_LINES[] = {
  #include "card_lines.h"
};
static const uint8_t CARD_COUNT = sizeof(CARD_LINES) / sizeof(CARD_LINES[0]);

// The minimum on-screen time scales with the length of the line, so a short
// quip does not linger and a long one does not get yanked mid-sentence.
// BASE covers noticing the card at all; PER_CHAR is the reading itself.
// 55 ms/char is ~200 wpm at an average 5-letter word, slowed for a 16-column
// panel you are reading at arm's length.
static const uint32_t CARD_BASE_MS     = 2000;   // floor, before any text
static const uint32_t CARD_MS_PER_CHAR = 55;
static const uint32_t CARD_HOLD_MS     = 1500;   // lingers after you set it down
static const uint32_t CARD_MAX_MS      = 6000;   // hard cap if you keep holding it
static const uint32_t CARD_COOLDOWN_MS = 5UL * 60UL * 1000UL;

// Cards take the whole panel -- no level, no bar, no sprite -- but stay in the
// normal portrait orientation, same 4x6 font as the rest of the UI. That is 16
// columns across the 64 px width. u8g2 has no font scaler, so going bigger
// means a different font: 6x12 is double height at 10 columns, 8x13 is a true
// 2x at 8. Both cost enough columns to hard-break ordinary words.
#define CARD_FONT  u8g2_font_4x6_tf
static const u8g2_uint_t CARD_LINE_GAP = 2;   // px added to the glyph height

// 8 lines of 8 px is 64, comfortably inside the 128 px height now that the
// header is not drawn. The longest line in the pool lands in 3.
static const uint8_t CARD_MAX_LINES = 8;
static const uint8_t CARD_MAX_CHARS = 20;

// Text is wrapped to slightly less than the panel width, so a full line does
// not run edge to edge. Purely cosmetic; the block stays centered on the panel.
static const u8g2_uint_t CARD_SIDE_INSET = 14;

const char* cardText   = nullptr;   // null = no card showing
uint32_t    cardStart  = 0;
uint32_t    cardMinMs  = 0;         // reading time for the line now showing
uint32_t    cardNextOk = 0;         // millis() before which no card may fire
uint8_t     cardLast   = 0xFF;      // last index used, to avoid immediate repeats
bool        cardIsRank = false;     // showing the rank announcement, not a quip

// Cards are composed at runtime -- rank announcements from a format string,
// quips by picking among the alternatives in the line -- so they need storage
// that outlives the call: cardText is only a pointer. Only one card is ever on
// screen, so both kinds share this buffer. The longest rank string is 53
// characters; a quip is capped by whatever fits here.
char cardBuf[96];

inline bool cardActive() { return cardText != nullptr; }

// How long this particular line needs to be readable.
uint32_t cardReadMs(const char* text) {
  return CARD_BASE_MS + (uint32_t)strlen(text) * CARD_MS_PER_CHAR;
}

// Lines may offer alternatives: a word written as "a/b/c" becomes one of a, b
// or c, drawn fresh every time the card is shown, so one line can read
// differently on each pickup.
//
//     "chad is getting into painting/fishing/photography"
//       -> "chad is getting into fishing"
//
// The run of alternatives is one space-delimited word, and trailing
// punctuation rides along with whatever gets picked, so
//     "chad is stretching/flexing, allegedly"
// keeps its comma either way. Every slash is read as an alternation, so a line
// cannot contain a literal one -- write "24-7", not "24/7".
static void expandCard(const char* src, char* dst, size_t cap) {
  static const char PUNCT[] = ",.!?;:";
  size_t o = 0;

  while (*src && o + 1 < cap) {
    if (*src == ' ') { dst[o++] = *src++; continue; }

    const char* tok = src;                       // one whitespace-delimited word
    while (*src && *src != ' ') src++;
    size_t tlen = (size_t)(src - tok);

    // Punctuation hangs off the end of the word, not off the last alternative.
    size_t core = tlen;
    while (core > 0 && strchr(PUNCT, tok[core - 1])) core--;

    // Alternatives are the slash-separated spans inside the core.
    uint8_t alts = 1;
    for (size_t k = 0; k < core; k++) if (tok[k] == '/') alts++;

    size_t begin = 0, end = core;
    if (alts > 1) {
      uint8_t want = (uint8_t)random(alts);
      uint8_t seen = 0;
      size_t k = 0;
      begin = 0;
      while (k < core && seen < want) { if (tok[k] == '/') { seen++; begin = k + 1; } k++; }
      end = begin;
      while (end < core && tok[end] != '/') end++;
    }

    // The chosen alternative, then the punctuation we set aside.
    for (size_t k = begin; k < end && o + 1 < cap; k++)  dst[o++] = tok[k];
    for (size_t k = core;  k < tlen && o + 1 < cap; k++) dst[o++] = tok[k];
  }

  dst[o] = '\0';
}

void showCard(const char* text, uint32_t now) {
  cardText   = text;
  cardStart  = now;
  cardMinMs  = cardReadMs(text);
  cardIsRank = false;
}

// Pick a line at random, never the same one twice in a row.
void showRandomCard(uint32_t now) {
  uint8_t i = (uint8_t)random(CARD_COUNT);
  if (CARD_COUNT > 1 && i == cardLast) i = (uint8_t)((i + 1) % CARD_COUNT);
  cardLast = i;
  // CARD_LINES[i] is the template; cardBuf is this particular reading of it.
  expandCard(CARD_LINES[i], cardBuf, sizeof(cardBuf));
  showCard(cardBuf, now);
}

// The queued rank change, spelled out. The queue is NOT cleared here -- that
// happens when the card finishes being displayed. If it gets pulled early
// (a walk starting), the announcement survives to be made properly later.
void showRankCard(uint32_t now) {
  snprintf(cardBuf, sizeof(cardBuf),
           "chad has been moving %s recently and is now a %s",
           (pendingRankDir > 0) ? "more" : "less",
           RANK_NAME[(pendingRankTo <= RANK_NOMAD) ? pendingRankTo : RANK_CHUD]);
  showCard(cardBuf, now);
  cardIsRank = true;
}

// What to show at a moment we know is being watched. A rank change outranks a
// random quip -- it is rare, it is the only real news the device has, and it
// has been waiting for exactly this moment.
void showAttentionCard(uint32_t now) {
  if (pendingRankDir != 0) showRankCard(now);
  else                     showRandomCard(now);
}

// Greedy word wrap. Breaks at the last space that fits; a single word longer
// than the line is hard-broken rather than dropped.
//
// Takes a flat buffer plus a stride rather than a 2D array: the .ino
// preprocessor hoists generated prototypes above the constants up there, so a
// CARD_MAX_CHARS in the signature would not be in scope yet.
static uint8_t wrapText(const char* s, char* out, uint8_t stride,
                        uint8_t maxLines, uint8_t maxChars) {
  if (maxChars > stride - 1) maxChars = stride - 1;
  uint8_t n = 0;
  while (*s && n < maxLines) {
    while (*s == ' ') s++;                 // eat the break we just made
    if (!*s) break;
    uint8_t len = 0;
    while (s[len] && len < maxChars) len++;
    if (s[len]) {                          // more text follows: back up to a space
      uint8_t brk = len;
      while (brk > 0 && s[brk] != ' ') brk--;
      if (brk > 0) len = brk;              // brk == 0 -> unbreakable, hard break
    }
    char* dst = out + (uint16_t)n * stride;
    memcpy(dst, s, len);
    dst[len] = '\0';
    s += len;
    n++;
  }
  return n;
}

// Centered on the whole panel, since the header is not drawn behind a card.
static void drawCard(const char* text) {
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();

  u8g2.setFont(CARD_FONT);
  const u8g2_uint_t cw = u8g2.getMaxCharWidth();    // 4x6 is fixed width
  const u8g2_uint_t lh = u8g2.getMaxCharHeight() + CARD_LINE_GAP;

  const u8g2_uint_t tw = (dw > CARD_SIDE_INSET)
                         ? (u8g2_uint_t)(dw - CARD_SIDE_INSET) : dw;

  char lines[CARD_MAX_LINES][CARD_MAX_CHARS + 1];
  const uint8_t n = wrapText(text, &lines[0][0], CARD_MAX_CHARS + 1,
                             CARD_MAX_LINES,
                             (uint8_t)(cw ? tw / cw : CARD_MAX_CHARS));

  // Unsigned math: a block taller than the panel pins to the top rather than
  // wrapping around to a huge y.
  const u8g2_uint_t block = (u8g2_uint_t)(n * lh);
  u8g2_uint_t y = ((dh > block) ? (u8g2_uint_t)((dh - block) / 2) : 0)
                  + u8g2.getAscent();

  for (uint8_t i = 0; i < n; i++) {
    const u8g2_uint_t w = u8g2.getStrWidth(lines[i]);
    u8g2.drawStr((w < dw) ? (u8g2_uint_t)((dw - w) / 2) : 0, y, lines[i]);
    y += lh;
  }
}

void drawFrame() {
  // A card takes over the panel completely: level and bar left off so the text
  // owns the whole screen.
  if (cardActive()) {
    u8g2.clearBuffer();
    drawCard(cardText);
    u8g2.sendBuffer();
    return;
  }

  u8g2.clearBuffer();
  player.draw();
  drawHeader();
  u8g2.sendBuffer();
}

// ============================================================================
// SLEEP BACKEND
// ============================================================================
// Two waits, two backends, because they have opposite power budgets:
//
//   CHARGING -> polling. The CPU spins on delay(), which costs milliamps. That
//     is free on wall power, and it keeps the USB serial port alive, which is
//     exactly when you want it.
//
//   DOWNTIME -> RP2040 DORMANT. Every clock in the chip stops, crystal
//     included; only a level on a wake-enabled GPIO restarts it. Microamps.
//
// pico-extras (pico/sleep.h) is not bundled with the arduino-pico core, so the
// dormant sequence below is written against the raw SDK clock/oscillator API.
// It is the same sequence pico-extras uses: park every clock on the crystal,
// kill the PLLs and the ring oscillator so nothing else is left running, stop
// the crystal, and on wake rebuild the whole clock tree with clocks_init().
//
// Three things dormant costs, all handled here or noted at the call site:
//   1. It cannot poll, so it needs a real interrupt line -> WAKE_ON_INT1.
//   2. millis() is frozen for the whole sleep -- see advanceClock().
//   3. Waking is a full clock re-init. clocks_init() puts every clock back
//      exactly where main() left it, so the SPI/I2C dividers programmed at
//      boot are correct again without touching u8g2 or Wire.
// ============================================================================

// The charging wait. Always polling: see above. Ends when the cable comes out,
// when INT1 reports a nudge (only if `watchMotion`), or after `timeoutMs` --
// pass 0 to wait indefinitely. Returns whether it is still plugged in, which is
// the only distinction either caller cares about.
//
// delay(), not __wfi(), because __wfi() only returns once something interrupts;
// delay() is guaranteed to come back and services USB on the way, which is the
// whole reason charging stays on the polling backend.
static bool chargingWait(bool watchMotion, unsigned long timeoutMs) {
  const unsigned long start = millis();
  for (;;) {
    if (digitalRead(CHARGE_PIN) == LOW) return false;
    if (watchMotion && digitalRead(INT1_PIN) == HIGH) return true;
    if (timeoutMs && (millis() - start) >= timeoutMs) return true;
    delay(20);
  }
}

#if TRUE_DEEP_SLEEP

// The core's SDK copy ships no rosc.h helper, so this is the register poke
// pico-extras would have made. ENABLE is a 12-bit magic-value field, not a bit,
// which is why it is masked in rather than set.
static void roscSetEnabled(bool on) {
  uint32_t ctrl = rosc_hw->ctrl & ~ROSC_CTRL_ENABLE_BITS;
  ctrl |= (on ? ROSC_CTRL_ENABLE_VALUE_ENABLE : ROSC_CTRL_ENABLE_VALUE_DISABLE)
          << ROSC_CTRL_ENABLE_LSB;
  rosc_hw->ctrl = ctrl;
  if (on) while (!(rosc_hw->status & ROSC_STATUS_STABLE_BITS)) tight_loop_contents();
  else    while ( (rosc_hw->status & ROSC_STATUS_STABLE_BITS)) tight_loop_contents();
}

// Park everything on the crystal, so that stopping the crystal stops the chip.
// Order matters: clk_sys has to be off the PLL before the PLL is torn down, and
// the ring oscillator goes last because it is the only other thing that could
// keep a clock alive through DORMANT.
static void dormantPrepareClocks() {
  clock_configure(clk_ref,  CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0,
                  XOSC_HZ, XOSC_HZ);
  clock_configure(clk_sys,  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, 0,
                  XOSC_HZ, XOSC_HZ);
  clock_stop(clk_usb);
  clock_stop(clk_adc);
  clock_stop(clk_rtc);
  clock_configure(clk_peri, 0, CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                  XOSC_HZ, XOSC_HZ);
  pll_deinit(pll_sys);
  pll_deinit(pll_usb);
  roscSetEnabled(false);
}

// Rebuild the clock tree exactly as main() found it at boot. Nothing here has
// to touch the voltage regulator: DORMANT is not a reset, so the core voltage
// main() picked for this F_CPU is still standing when we come back.
static void dormantRestoreClocks() {
  roscSetEnabled(true);                 // kickstart, as pico-extras does
  clocks_hw->sleep_en0 = ~0u;           // undo any gating before clocks_init
  clocks_hw->sleep_en1 = ~0u;
  clocks_init();                        // xosc, both PLLs, every clock, ticks
#if (F_CPU != 125000000)
  set_sys_clock_khz(F_CPU / 1000, true);   // the same call main() makes
#endif
}

// Stop the chip until `pin` reads high.
//
// LEVEL, not EDGE, on purpose. The ADXL345 holds INT1 high until INT_SOURCE is
// read, so a level trigger wakes immediately if the interrupt landed in the
// window between arming and the crystal actually stopping. An edge trigger
// would miss that one and sleep forever.
static void dormantUntilPinHigh(uint8_t pin) {
  gpio_set_dormant_irq_enabled(pin, GPIO_IRQ_LEVEL_HIGH, true);
  xosc_dormant();                       // <-- the chip stops on this line
  gpio_acknowledge_irq(pin, GPIO_IRQ_LEVEL_HIGH);
  gpio_set_dormant_irq_enabled(pin, GPIO_IRQ_LEVEL_HIGH, false);
}

void deepSleepUntilPinHigh(uint8_t pin) {
  // Already asserted? Then there is nothing to wait for, and arming a wake on a
  // level that is already true is the one way this hangs.
  if (digitalRead(pin) == HIGH) return;

  // The dormant wake path is at the oscillator, not the NVIC, so masking
  // interrupts cannot stop the wake -- it only keeps a stray handler from
  // running against a half-built clock tree.
  noInterrupts();
  dormantPrepareClocks();
  dormantUntilPinHigh(pin);
  dormantRestoreClocks();
  interrupts();
}

#else   // TRUE_DEEP_SLEEP == 0

static void pollWaitForPin(uint8_t pin, int level) {
  while (digitalRead(pin) != level) {
    __wfi();
  }
}

void deepSleepUntilPinHigh(uint8_t pin) { pollWaitForPin(pin, HIGH); }

#endif  // TRUE_DEEP_SLEEP

// Wait for movement using whichever wake source is selected above.
void waitForMotion() {
#if WAKE_ON_INT1
  enableMotionWake();
  (void)readReg(0x30);              // clear stale interrupt before sleeping
  delay(20);                        // let the pin settle
  deepSleepUntilPinHigh(INT1_PIN);
  (void)readReg(0x30);              // clear it again on wake
#else
  while (!motionDetected()) {
    delay(100);
  }
#endif
}

// ============================================================================

#if IDLE_SLEEP_ENABLED
void drawStaticAnim(const Animation* a) {
  u8g2.clearBuffer();
  drawAnimFrame(a, 0);
  drawHeader();
  u8g2.sendBuffer();
}
#endif

// Reset the classifier so stale pre-sleep samples do not leak into the
// first post-wake decision.
void resetClassifier(unsigned long now) {
  head = 0; filled = 0; sinceHop = 0;
  state = "IDLE"; pendingState = "IDLE"; pendingCount = 0;
  idleSince = now;

  // The motion that woke us would otherwise read as a fresh pickup the moment
  // the buffer refills. Start "already quiet" so the wake card is the only one.
  handled = false;
  handledLastMs  = now;
  unhandledSince = now;
  cardText   = nullptr;
  cardIsRank = false;
}

void drawChargingScreen() {
  u8g2.setPowerSave(0);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawStr(4, u8g2.getDisplayHeight() / 2, "charging");
  u8g2.sendBuffer();
}

// Charging: show the charging screen, blank it, wait out the charge.
//
// The screen does not stay lit. A charge is hours and the panel is a mono OLED,
// so it says its piece for CHARGE_SCREEN_MS and then goes dark; a nudge brings
// it back for another CHARGE_SCREEN_MS. Power is free on the cable -- this is
// about the panel, not the battery. Without INT1 there is nothing to nudge
// with, so it simply blanks once and stays that way until unplugged.
void handleCharging() {
  commitSave();                    // last chance before we stop running
  Serial.println("-> charging");

#if WAKE_ON_INT1
  enableMotionWake();              // so a nudge can bring the screen back
  const bool peek = true;
#else
  const bool peek = false;
#endif

  bool plugged = true;
  while (plugged) {
    drawChargingScreen();
    // Lit. Only the cable matters now: a nudge during these seconds is asking
    // for a screen that is already on.
    plugged = chargingWait(false, CHARGE_SCREEN_MS);
    if (!plugged) break;

    u8g2.setPowerSave(1);          // dark, and staying that way until nudged
    if (peek) {
      delay(20);                   // let INT1 settle
      (void)readReg(0x30);         // drop whatever latched while it was lit
    }
    plugged = chargingWait(peek, 0);
  }

  Serial.println("<- unplugged");
  enableNormalMode();
  resetClassifier(millis());
  player.play(rankSprites().rest, millis());
  drawFrame();
}

// Downtime: blank the panel, wait for motion. With sleep art present this is
// preceded by a held sleep frame so the pet is visibly nodding off rather than
// the screen just dying; without it the panel goes straight to black.
//
// Note for anyone watching the serial monitor: you will never see this fire.
// Reaching it requires CHARGE_PIN low, i.e. USB unplugged, i.e. no serial port.
// Confirm dormant on a battery with a meter in series, not on the console.
void handleDowntimeSleep() {
  commitSave();                    // last chance before we stop running
  Serial.println("-> downtime sleep");
  Serial.flush();                  // dormant stops clk_usb mid-transfer otherwise
  cardText   = nullptr;            // never sleep on a half-shown card
  cardIsRank = false;              // an unfinished rank card stays queued
#if IDLE_SLEEP_ENABLED
  drawStaticAnim(animForActivity(SLEEP));
  delay(1500);
#endif
  u8g2.setPowerSave(1);            // blank the OLED to save power

  waitForMotion();

  Serial.println("<- woke on motion");
  u8g2.setPowerSave(0);
  enableNormalMode();

  const unsigned long now = millis();
  resetClassifier(now);            // clears any card, so fire ours after it
  player.play(rankSprites().rest, now);

  // The best card slot there is: the panel was blank, it just lit up, and
  // whatever moved the device is a hand. Bypasses the cooldown on purpose.
  showAttentionCard(now);
  cardNextOk = now + CARD_COOLDOWN_MS;
  drawFrame();
}

// ---------------- setup / loop -----------------------------------------------

void setup() {
  SPI.setSCK(2);
  SPI.setTX(3);
  u8g2.begin();
  u8g2.setDrawColor(1);

  Serial.begin(115200);
  delay(2000);                      // give the host time to attach the port
  // no while(!Serial): must not hang on battery power

  loadSave();

  // Unseeded random() repeats the same sequence every boot, which would make
  // the first card after every power-on identical. clockSec survives reboots.
  randomSeed(save.clockSec ^ micros());

  Wire.setSDA(12); Wire.setSCL(13);
  Wire.begin();
  Wire.setClock(100000);

  uint8_t devid = readReg(0x00);
  Serial.print("DEVID (want 0xE5) = 0x");
  Serial.println(devid, HEX);
  if (devid != 0xE5) Serial.println("  !! not a real ADXL345, or bad wiring");

  enableNormalMode();

  pinMode(CHARGE_PIN, INPUT);       // divider defines both states
  pinMode(INT1_PIN, INPUT);         // ADXL345 drives this actively

  Serial.print("wake source: ");
  Serial.println(WAKE_ON_INT1 ? "INT1 hardware interrupt" : "software polling");
  Serial.print("downtime sleep: ");
  Serial.println(TRUE_DEEP_SLEEP ? "DORMANT (clocks stopped)" : "polling __wfi");

  resetClassifier(millis());
  player.play(rankSprites().rest, millis());
  drawFrame();
}

void loop() {
  static unsigned long nextT = 0;
  static unsigned long nextDbg = 0;
  unsigned long now = millis();
  if (nextT == 0) nextT = now;

  advanceClock();

  // Periodic status line — throttled so it does not flood the monitor.
  if ((long)(now - nextDbg) >= 0) {
    nextDbg = now + 500;
    Serial.print("chg="); Serial.print(digitalRead(CHARGE_PIN));
    Serial.print(" int1="); Serial.print(digitalRead(INT1_PIN));
    Serial.print(" mag="); Serial.print(readMagMg(), 0);
    Serial.print(" var="); Serial.print(lastVar, 0);
    Serial.print(" rhy="); Serial.print(lastRhythm, 2);
    Serial.print(" state="); Serial.print(state);
    Serial.print(" svar="); Serial.print(shortVar(), 0);
    Serial.print(" held="); Serial.print(handled);
    Serial.print(" lvl="); Serial.print(cachedLevel);
    Serial.print(" xp="); Serial.print(save.xp);
    Serial.print(" recent="); Serial.print(recentActiveSec());
    Serial.print(" rank="); Serial.println(RANK_NAME[save.rank]);
  }

  // --- Charging takes priority ---
  if (digitalRead(CHARGE_PIN) == HIGH) {
    handleCharging();
    nextT = 0;
    return;
  }

  // --- Downtime sleep ---
  if (idleSince != 0 && (now - idleSince) >= DEEPSLEEP_AFTER_MS) {
    handleDowntimeSleep();
    nextT = 0;
    return;
  }

  // Accel sample + classify on the ODR clock
  if ((long)(now - nextT) >= 0) {
    nextT += 1000 / ODR_HZ;

    pushSample(readMagMg());
    sinceHop++;

    // --- pickup detection, off the short window ---
    // Deliberately outside the SMOOTH_N debounce: that exists to stop
    // WALK/IDLE flapping, and a second of lag here would miss the look.
    if (shortVar() > HANDLE_GATE) {
      handledLastMs = now;
      if (!handled) {
        const bool wasStill =
            unhandledSince != 0 && (now - unhandledSince) >= STILL_BEFORE_MS;
        handled = true;
        idleSince = 0;              // do not blank the panel while it is in a hand
        // A pending rank change ignores the cooldown: it is news, it fires
        // once, and it has already waited for someone to look.
        if (wasStill && !cardActive() && strcmp(state, "MOVING") != 0 &&
            (pendingRankDir != 0 || (long)(now - cardNextOk) >= 0)) {
          showAttentionCard(now);
        }
      }
    } else if (handled && (now - handledLastMs) >= HANDLE_RELEASE_MS) {
      handled = false;
      unhandledSince = now;
      if (strcmp(state, "IDLE") == 0) idleSince = now;   // sleep clock resumes
    }

    // XP is time spent moving, accumulated a second at a time.
    static uint16_t activeMs = 0;
    if (strcmp(state, "MOVING") == 0) {
      activeMs += 1000 / ODR_HZ;
      if (activeMs >= 1000) { activeMs -= 1000; addActiveSecond(); }
    }

    if (filled == WIN && sinceHop >= HOP) {
      sinceHop = 0;
      windowFeatures(lastVar, lastRhythm);
      const char* raw = classify(lastVar, lastRhythm);

      if (strcmp(raw, state) == 0) {
        pendingCount = 0;
      } else if (strcmp(raw, pendingState) == 0) {
        if (++pendingCount >= SMOOTH_N) {
          state = raw;
          pendingCount = 0;
          idleSince = (strcmp(state, "IDLE") == 0 && !handled) ? now : 0;
        }
      } else {
        pendingState = raw;
        pendingCount = 1;
      }
    }
  }

  // --- card lifetime ---
  if (cardActive()) {
    const uint32_t age = now - cardStart;
    if (strcmp(state, "MOVING") == 0) {
      // The start of a walk also spikes the short window, so a card can fire a
      // beat before the classifier catches up. Pull it, and refund the
      // cooldown: nobody saw that one, and it should not cost the real pickup
      // a card later. The short guard stops the same motion re-triggering.
      // pendingRankDir is left set, so an interrupted announcement comes back.
      cardText   = nullptr;
      cardIsRank = false;
      cardNextOk = now + 5000;
    } else {
      // The cap never cuts a line short of its own reading time; a long enough
      // line just overrides it rather than getting truncated mid-sentence.
      const uint32_t cap = (CARD_MAX_MS > cardMinMs) ? CARD_MAX_MS : cardMinMs;
      if (age >= cap ||
          (age >= cardMinMs && !handled &&
           (now - handledLastMs) >= CARD_HOLD_MS)) {
        // It was shown in full, so the announcement is now delivered.
        if (cardIsRank) pendingRankDir = 0;
        cardText   = nullptr;
        cardIsRank = false;
        cardNextOk = now + CARD_COOLDOWN_MS;
      }
    }
  }

  // Animation ticks independently of the classifier hop
  player.play(animForActivity(activityFromMotion(now)), now);

  static uint8_t lastFrame = 255;
  static const Animation* lastAnim = nullptr;
  static uint16_t lastLevel = 0;
  static uint8_t  lastRank  = 255;
  static uint8_t  lastProgress = 255;
  static const char* lastCard = nullptr;
  player.update(now);
  // A card is static, so while one is up the only thing worth redrawing for is
  // the card itself appearing or clearing. The sprite keeps ticking underneath
  // and the stale trackers are refreshed on the way out.
  if (cardText != lastCard ||
      (!cardActive() &&
       (player.anim != lastAnim || player.frame != lastFrame ||
        cachedLevel != lastLevel || save.rank != lastRank ||
        cachedProgress != lastProgress))) {
    lastAnim     = player.anim;
    lastFrame    = player.frame;
    lastLevel    = cachedLevel;
    lastRank     = save.rank;
    lastProgress = cachedProgress;
    lastCard     = cardText;
    drawFrame();
  }
}
