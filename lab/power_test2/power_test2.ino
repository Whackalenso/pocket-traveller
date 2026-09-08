// power_test.ino — Pico + ADXL345 (I2C 0x1D) + SH1106 SPI OLED
//
// Motion classifier + sprite playback, plus power behavior:
//   * Charging: USB 5V (TP4056 IN+) divided down and read on CHARGE_PIN.
//     Plugged in -> static "charging" screen -> sleep until unplugged.
//   * Downtime: after DEEPSLEEP_AFTER_MS idle -> one static sleep frame,
//     blank the OLED, sleep until motion.
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
#include "sprites.h"

#define ADDR 0x1D

// ---- Wake source selector ---------------------------------------------------
// 1 = wake from downtime using the ADXL345 hardware activity interrupt (INT1).
// 0 = wake by polling the accelerometer in software (no INT1 wire needed).
// Start at 0 to prove the sleep flow works, then try 1 to test the interrupt.
#define WAKE_ON_INT1 1

// ---------------- power pins -------------------------------------------------
static const uint8_t CHARGE_PIN = 11;   // HIGH = USB plugged into TP4056
static const uint8_t INT1_PIN   = 15;   // ADXL345 INT1 (active HIGH on activity)

// Idle time before the Pico drops into the sleep wait.
// Set short while testing; raise for real use.
static const unsigned long DEEPSLEEP_AFTER_MS = 6UL * 10UL * 1000UL;   // 60 s

// ======================= TUNE THESE =========================================
float ENERGY_GATE = 1000.0f;  // variance in (milli-g)^2. 1 g = 1000 mg.
float RHYTHM_TH   = 0.2f;     // autocorr peak 0..1 in the step band.

// Idle time before rest -> sleep animation (must be < DEEPSLEEP_AFTER_MS).
static const unsigned long IDLE_SLEEP_MS = 5UL * 1000UL;   // 5 s

// Software motion-wake threshold, in milli-g away from 1 g rest.
static const float WAKE_MOTION_MG = 1000.0f;

// ADXL345 activity threshold, 62.5 mg/LSB. 8 -> ~500 mg. Lower = touchier.
static const uint8_t ACT_THRESH = 8;

static const uint8_t SPRITE_SCALE = 3;
// ============================================================================

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R3, 8, 7, 6);

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
  const u8g2_uint_t x = (dw - sw) / 2;
  const u8g2_uint_t y = (dh > dw) ? dh / 2 + (dh / 2 - sh) / 2 : (dh - sh) / 2;
  player.draw(x, y);
  u8g2.sendBuffer();
}

// ============================================================================
// SLEEP BACKEND — THIS IS THE ONLY THING YOU SWAP FOR TRUE DEEP SLEEP.
// ============================================================================
// Currently POLLING: the CPU sleeps between checks with __wfi() and wakes on
// the millis() tick. Correct behavior, but the core stays powered (~mA, not
// uA). Fine for CHARGING (wall power); NOT enough to save battery overnight.
//
// Later: replace these bodies with the pico/sleep.h dormant path. Call sites
// do not change. Note dormant sleep cannot poll, so it requires WAKE_ON_INT1.
// ============================================================================

static void pollWaitForPin(uint8_t pin, int level) {
  while (digitalRead(pin) != level) {
    __wfi();
  }
}

void deepSleepUntilPinHigh(uint8_t pin) { pollWaitForPin(pin, HIGH); }
void deepSleepUntilPinLow(uint8_t pin)  { pollWaitForPin(pin, LOW);  }

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

// Reset the classifier so stale pre-sleep samples do not leak into the
// first post-wake decision.
void resetClassifier(unsigned long now) {
  head = 0; filled = 0; sinceHop = 0;
  state = "IDLE"; pendingState = "IDLE"; pendingCount = 0;
  idleSince = now;
}

// Charging: static screen, sleep until USB is removed.
void handleCharging() {
  Serial.println("-> charging");
  u8g2.setPowerSave(0);            // screen stays on; power is free when plugged in
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawStr(4, u8g2.getDisplayHeight() / 2, "charging");
  u8g2.sendBuffer();

  deepSleepUntilPinLow(CHARGE_PIN);   // wake when unplugged

  Serial.println("<- unplugged");
  enableNormalMode();
  resetClassifier(millis());
  player.play(&ANIM_REST, millis());
  drawFrame();
}

// Downtime: one static sleep frame, blank the panel, wait for motion.
void handleDowntimeSleep() {
  Serial.println("-> downtime sleep");
  drawStaticAnim(&ANIM_SLEEP);
  delay(1500);
  u8g2.setPowerSave(1);            // blank the OLED to save power

  waitForMotion();

  Serial.println("<- woke on motion");
  u8g2.setPowerSave(0);
  enableNormalMode();
  resetClassifier(millis());
  player.play(&ANIM_REST, millis());
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

  resetClassifier(millis());
  player.play(&ANIM_REST, millis());
  drawFrame();
}

void loop() {
  static unsigned long nextT = 0;
  static unsigned long nextDbg = 0;
  unsigned long now = millis();
  if (nextT == 0) nextT = now;

  // Periodic status line — throttled so it does not flood the monitor.
  if ((long)(now - nextDbg) >= 0) {
    nextDbg = now + 500;
    Serial.print("chg="); Serial.print(digitalRead(CHARGE_PIN));
    Serial.print(" int1="); Serial.print(digitalRead(INT1_PIN));
    Serial.print(" mag="); Serial.print(readMagMg(), 0);
    Serial.print(" var="); Serial.print(lastVar, 0);
    Serial.print(" rhy="); Serial.print(lastRhythm, 2);
    Serial.print(" state="); Serial.println(state);
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
          idleSince = (strcmp(state, "IDLE") == 0) ? now : 0;
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
