// tamagotchi.ino — Pico + LIS3DH clone (I2C 0x1D) + SH1106 SPI OLED
//
// Combines motion_state_display (accel classifier) with tamagotchi_anim
// (sprite playback). Screen shows:
//   REST   when IDLE
//   WALK   when MOVING
//   SLEEP  when IDLE for IDLE_SLEEP_MS
//
// Decision per 2.56 s window (updated every 0.5 s):
//   variance <  ENERGY_GATE                        -> IDLE
//   variance >= ENERGY_GATE and rhythm <= RHYTHM_TH -> IDLE
//   variance >= ENERGY_GATE and rhythm >  RHYTHM_TH -> MOVING
//
// POWER BEHAVIOR (added):
//   * Charging: USB 5V (from TP4056 IN+) is divided down and read on
//     CHARGE_PIN. When plugged in -> draw a static "charging..." screen,
//     then deep-sleep the Pico. Wake when USB is removed (pin goes low).
//   * Downtime: after the guy has been in SLEEP for DEEPSLEEP_AFTER_MS,
//     draw one static sleeping frame, blank the OLED, and deep-sleep.
//     Wake on motion via the LIS3DH INT1 pin (INT1_PIN).
//
// WIRING (added):
//   CHARGE_PIN (GP7) <- resistor divider from TP4056 IN+ (USB 5V).
//       Size the divider so the pin sees ~3.0V (NOT 3.3V) at 5V in, for
//       headroom against a slightly-high USB source. e.g. 10k top / 15k
//       bottom -> 5*15/25 = 3.0V. Add nothing else; divider's bottom leg
//       also holds the pin low when unplugged.
//   INT1_PIN   (GP8) <- LIS3DH INT1 output (active-high, see enableMotionWake).
//   Shared ground between TP4056 and Pico is required.

#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "sprites.h"

#define ADDR 0x1D

// ---------------- power pins -------------------------------------------------
static const uint8_t CHARGE_PIN = 7;   // divided USB-present signal (HIGH = plugged in)
static const uint8_t INT1_PIN   = 8;   // LIS3DH INT1 (HIGH = motion) -> wake source

// After the guy has been asleep this long, drop the Pico into deep sleep
// (wake on motion). Keep this comfortably longer than IDLE_SLEEP_MS so the
// sleep *animation* shows for a while before the chip actually powers down.
static const unsigned long DEEPSLEEP_AFTER_MS = 1UL * 10UL * 1000UL;  // 15 min

// ======================= TUNE THESE =========================================
float ENERGY_GATE = 1000.0f;  // variance (raw-LSB^2, 12-bit >>4 units).
float RHYTHM_TH   = 0.2f;     // autocorr peak 0..1 in the step band.

// How long to stay IDLE before switching from rest -> sleep animation.
static const unsigned long IDLE_SLEEP_MS = 1UL * 5UL * 1000UL;  // 10 minutes

// Integer scale for on-screen sprite size (1 = native, 2 = double, etc.).
static const uint8_t SPRITE_SCALE = 3;
// ============================================================================

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET
// U8G2_R3 = 90° CCW → 64x128 portrait
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 6, 5, 4);

// ---------------- sensor -----------------------------------------------------

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}
uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}
int16_t axis(uint8_t lo, uint8_t hi) {
  return (int16_t)(readReg(lo) | (readReg(hi) << 8));
}

// Configure the LIS3DH to raise INT1 (active-high) on movement, so it can
// wake the Pico from deep sleep. MOTION_THS is the sensitivity dial:
// lower = wakes on a gentle nudge, higher = needs a firmer shake.
static const uint8_t ACT_THRESH = 16;   // 62.5 mg/LSB -> 16 = ~1g; lower = more sensitive
void enableMotionWake() {
  writeReg(0x2D, 0x00);   // standby while configuring
  writeReg(0x24, ACT_THRESH);  // THRESH_ACT
  writeReg(0x27, 0xF0);   // ACT_INACT_CTL: ac-coupled, X/Y/Z activity enabled
  writeReg(0x2F, 0x00);   // INT_MAP: activity -> INT1 pin
  writeReg(0x2E, 0x10);   // INT_ENABLE: activity interrupt only
  writeReg(0x2D, 0x08);   // back to measurement mode
  (void)readReg(0x30);    // read INT_SOURCE to clear
}

// Restore the "normal" data-reading config used by the classifier.
void enableNormalMode() {
  writeReg(0x20, 0x57);   // 100 Hz ODR, XYZ on
  writeReg(0x23, 0x08);   // high-res, +/-2g
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

// millis() when we last entered IDLE (0 = not idle / just became moving).
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

  void draw(u8g2_uint_t x, u8g2_uint_t y) const {
    if (anim)
      drawXBMPScaled(x, y, anim->w, anim->h, anim->frames[frame]);
  }
};

AnimPlayer player;

Activity activityFromMotion(unsigned long now) {
  if (strcmp(state, "MOVING") == 0) return WALK;
  if (idleSince != 0 && (now - idleSince) >= IDLE_SLEEP_MS) return SLEEP;
  return REST;
}

const Animation* animForActivity(Activity a) {
  switch (a) {
    case WALK:  return &ANIM_WALK;
    case SLEEP: return &ANIM_SLEEP;
    case REST:
    default:    return &ANIM_REST;
  }
}

void drawFrame() {
  u8g2.clearBuffer();
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();
  const u8g2_uint_t sw = ANIM_REST.w * SPRITE_SCALE;
  const u8g2_uint_t sh = ANIM_REST.h * SPRITE_SCALE;
  // Portrait (taller than wide): centre horizontally in the bottom half.
  // Landscape: centre on the whole panel.
  const u8g2_uint_t x = (dw - sw) / 2;
  const u8g2_uint_t y = (dh > dw)
      ? dh / 2 + (dh / 2 - sh) / 2
      : (dh - sh) / 2;
  player.draw(x, y);
  u8g2.sendBuffer();
}

// ============================================================================
// SLEEP BACKEND  —  THIS IS THE ONLY THING YOU SWAP TO GET TRUE DEEP SLEEP.
// ============================================================================
//
// Both power cases (charging + downtime) go through these two functions:
//     deepSleepUntilPinHigh(pin)   // used for motion wake  (INT1 goes HIGH)
//     deepSleepUntilPinLow(pin)    // used for unplug wake   (CHARGE goes LOW)
//
// CURRENT BACKEND: POLLING.
//   The CPU sleeps between checks with __wfi() and wakes on the SysTick
//   millis() interrupt every ~1 ms, re-checks the pin, and either returns or
//   goes back to sleep. This is version-proof and behaves correctly, but the
//   Pico core stays powered, so draw is only modestly reduced (~mA, not µA).
//   Fine for CHARGING (wall power). NOT enough to save the battery OVERNIGHT.
//
// LATER: replace ONLY the two function bodies below with the pico/sleep.h
//   dormant path (stop the oscillator, wake on a GPIO edge, restore clocks).
//   Nothing else in the sketch changes — the call sites stay identical.
// ============================================================================

#include "hardware/clocks.h"

// Poll `pin` until it reaches `level`, sleeping the CPU between checks.
static void pollWaitForPin(uint8_t pin, int level) {
  while (digitalRead(pin) != level) {
    __wfi();   // sleep until the next interrupt (millis tick), then re-check
  }
}

void deepSleepUntilPinHigh(uint8_t pin) { pollWaitForPin(pin, HIGH); }
void deepSleepUntilPinLow(uint8_t pin)  { pollWaitForPin(pin, LOW);  }

// ============================================================================

// Draw a single static frame (no animation) and push it to the panel once.
void drawStaticAnim(const Animation* a) {
  u8g2.clearBuffer();
  const u8g2_uint_t dw = u8g2.getDisplayWidth();
  const u8g2_uint_t dh = u8g2.getDisplayHeight();
  const u8g2_uint_t sw = a->w * SPRITE_SCALE;
  const u8g2_uint_t sh = a->h * SPRITE_SCALE;
  const u8g2_uint_t x = (dw - sw) / 2;
  const u8g2_uint_t y = (dh > dw) ? dh / 2 + (dh / 2 - sh) / 2 : (dh - sh) / 2;
  drawXBMPScaled(x, y, a->w, a->h, a->frames[0]);
  u8g2.sendBuffer();
}

// Charging: show a static "charging..." screen, sleep until USB is removed.
void handleCharging() {
  u8g2.setPowerSave(0);           // keep screen ON (plugged in; power is free)
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x15_tf);
  // Panel is portrait (R3): draw centred-ish. Coordinates are display-space.
  u8g2.drawStr(4, u8g2.getDisplayHeight() / 2, "charging");
  u8g2.drawStr(20, u8g2.getDisplayHeight() / 2 + 18, "...");
  u8g2.sendBuffer();

  deepSleepUntilPinLow(CHARGE_PIN);   // wake when unplugged (pin goes low)

  // Woke up: resume normal sensing/animation cleanly.
  enableNormalMode();
  idleSince = millis();
  player.play(&ANIM_REST, idleSince);
  drawFrame();
}

// Downtime: show one static sleeping frame, blank the OLED, sleep until motion.
void handleDowntimeSleep() {
  drawStaticAnim(&ANIM_SLEEP);        // one sleeping frame...
  delay(1500);                        // ...visible briefly...
  u8g2.setPowerSave(1);               // ...then blank the panel to save power.

  enableMotionWake();                 // arm INT1 on movement
  (void)readReg(0x31);        // clear any stale interrupt BEFORE sleeping
  delay(20);                  // let the pin settle low
  deepSleepUntilPinHigh(INT1_PIN);    // wake when picked up / moved

  // Woke up on motion: restore everything.
  u8g2.setPowerSave(0);
  enableNormalMode();
  (void)readReg(0x31);                // clear INT1 source
  idleSince = millis();
  state = "IDLE";
  player.play(&ANIM_REST, idleSince);
  drawFrame();
}

// ---------------- setup / loop -----------------------------------------------

void setup() {
  SPI.setSCK(2);   // CLK  -> GP2
  SPI.setTX(3);    // MOSI -> GP3
  u8g2.begin();
  u8g2.setDrawColor(1);

  Serial.begin(115200);
  // no while(!Serial): must not hang on battery power

  Wire.setSDA(0); Wire.setSCL(1);
  Wire.begin();
  Wire.setClock(100000);
  enableNormalMode();     // 100 Hz ODR, XYZ on, high-res +/-2g

  // Power sensing pins.
  // CHARGE_PIN: divider holds it low when unplugged, ~3V when plugged in.
  // No internal pull needed (divider defines both states), so plain INPUT.
  pinMode(CHARGE_PIN, INPUT);
  // INT1: LIS3DH drives it high on motion; plain INPUT (chip actively drives).
  pinMode(INT1_PIN, INPUT_PULLDOWN);

  idleSince = millis();
  player.play(&ANIM_REST, idleSince);
  drawFrame();
}

void loop() {
  Serial.print("INT1="); Serial.println(digitalRead(INT1_PIN));

  static unsigned long nextT = 0;
  unsigned long now = millis();
  if (nextT == 0) nextT = now;

  // --- Power: charging takes priority over everything ---
  // CHARGE_PIN reads HIGH when USB is plugged into the TP4056.
  if (digitalRead(CHARGE_PIN) == HIGH) {
    Serial.println("charging");
    handleCharging();           // static screen + sleep until unplugged
    nextT = 0;                  // reset the ODR clock after waking
    return;
  }

  // --- Power: downtime deep sleep (wake on motion) ---
  // Once the guy has been in SLEEP long enough, power down until moved.
  if (idleSince != 0 && (now - idleSince) >= DEEPSLEEP_AFTER_MS) {
    handleDowntimeSleep();      // sleep frame + blank + wake on motion
    nextT = 0;                  // reset the ODR clock after waking
    return;
  }

  // Accel sample + classify on the ODR clock
  if ((long)(now - nextT) >= 0) {
    nextT += 1000 / ODR_HZ;

    int16_t x, y, z;
    readXYZ(x, y, z);
    pushSample(sqrtf((float)x*x + (float)y*y + (float)z*z));
    sinceHop++;

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
          if (strcmp(state, "IDLE") == 0) {
            idleSince = now;
          } else {
            idleSince = 0;
          }
        }
      } else {
        pendingState = raw;
        pendingCount = 1;
      }
    }
  }

  // Animation ticks independently of the classifier hop
  player.play(animForActivity(activityFromMotion(now)), now);

  static uint8_t lastFrame = 255;
  static const Animation* lastAnim = nullptr;
  player.update(now);
  if (player.anim != lastAnim || player.frame != lastFrame) {
    lastAnim = player.anim;
    lastFrame = player.frame;
    drawFrame();
  }
}
