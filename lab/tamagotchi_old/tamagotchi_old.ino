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

#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "sprites.h"

#define ADDR 0x1D

// ======================= TUNE THESE =========================================
float ENERGY_GATE = 1000.0f;  // variance (raw-LSB^2, 12-bit >>4 units).
float RHYTHM_TH   = 0.2f;     // autocorr peak 0..1 in the step band.

// How long to stay IDLE before switching from rest -> sleep animation.
static const unsigned long IDLE_SLEEP_MS = 10UL * 60UL * 1000UL;  // 10 minutes

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
  writeReg(0x20, 0x57);   // 100 Hz ODR, XYZ on
  writeReg(0x23, 0x08);   // high-res, +/-2g

  idleSince = millis();
  player.play(&ANIM_REST, idleSince);
  drawFrame();
}

void loop() {
  static unsigned long nextT = 0;
  unsigned long now = millis();
  if (nextT == 0) nextT = now;

  // Accel sample + classify on the ODR clock
  if ((long)(now - nextT) >= 0) {
    nextT += 1000 / ODR_HZ;

    int16_t x = axis(0x28, 0x29) >> 4;
    int16_t y = axis(0x2A, 0x2B) >> 4;
    int16_t z = axis(0x2C, 0x2D) >> 4;
    pushSample(sqrtf((float)x * x + (float)y * y + (float)z * z));
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
