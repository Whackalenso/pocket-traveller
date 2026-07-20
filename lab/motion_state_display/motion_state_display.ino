// motion_state_display.ino — Pico + LIS3DH clone (I2C 0x1D) + SH1106 SPI OLED
//
// Two states on screen:
//   IDLE    sitting, standing, fidgeting, riding in a car/bus
//   MOVING  slow walk, walk, run — anything rhythmically on-your-feet
//
// Decision per 2.56 s window (updated every 0.5 s):
//   variance <  ENERGY_GATE                        -> IDLE   (too calm)
//   variance >= ENERGY_GATE and rhythm <= RHYTHM_TH -> IDLE  (energetic but
//                                                     aperiodic: bumps/fidget)
//   variance >= ENERGY_GATE and rhythm >  RHYTHM_TH -> MOVING
//
// Built on your confirmed-working sensor reads (single-register, repeated
// start) and your confirmed-working U8g2 SPI display setup.
//
// TUNING: thresholds are right below. With CAL_MODE = 1:
//   - screen shows live v:/r: numbers (untethered eyeball calibration)
//   - serial prints CSV lines "variance,rhythm" you can save into files
//     named cal_1.csv, cal_2.csv, ... for analyze_cal.py (one activity per file)

#include <Wire.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>

#define ADDR 0x1D

// ======================= TUNE THESE =========================================
float ENERGY_GATE = 1000.0f;  // variance (raw-LSB^2, 12-bit >>4 units).
                              // PLACEHOLDER — calibrate with analyze_cal.py.
float RHYTHM_TH   = 0.2f;    // autocorr peak 0..1 in the step band.
#define CAL_MODE  0           // 1 = show v/r on screen + CSV over serial. 0 = state only.
// ============================================================================

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 6, 5, 4);

// ---------------- sensor (your working access pattern) ----------------------

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

// ---------------- classifier settings ---------------------------------------

const int   ODR_HZ   = 50;    // our sampling rate (sensor ODR is 100 Hz)
const int   WIN      = 128;   // 2.56 s window
const int   HOP      = 25;    // re-classify every 0.5 s
const int   LAG_LO   = 12;    // 50/4  -> 4.0 Hz upper step frequency
const int   LAG_HI   = 36;    // 50/1.4-> 1.4 Hz lower step frequency
const int   SMOOTH_N = 2;     // consecutive windows needed to switch state

float bufv[WIN];
int   head = 0, filled = 0, sinceHop = 0;

const char* state        = "IDLE";
const char* pendingState = "IDLE";
int         pendingCount = 0;

float lastVar = 0, lastRhythm = 0;

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

// ---------------- display ----------------------------------------------------

void drawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_10x20_tf);
  int w = u8g2.getStrWidth(state);
  u8g2.drawStr((128 - w) / 2, 30, state);
#if CAL_MODE
  u8g2.setFont(u8g2_font_6x10_tf);
  char line[32];
  snprintf(line, sizeof(line), "v:%.0f r:%.2f", lastVar, lastRhythm);
  u8g2.drawStr(4, 60, line);
#endif
  u8g2.sendBuffer();
}

// ---------------- setup / loop ------------------------------------------------

void setup() {
  SPI.setSCK(2);   // CLK  -> GP2
  SPI.setTX(3);    // MOSI -> GP3
  u8g2.begin();

  Serial.begin(115200);
  // no while(!Serial): must not hang on battery power

  Wire.setSDA(0); Wire.setSCL(1);
  Wire.begin();
  Wire.setClock(100000);
  writeReg(0x20, 0x57);   // 100 Hz ODR, XYZ on   (your working config)
  writeReg(0x23, 0x08);   // high-res, +/-2g      (your working config)
  // If hard running seems capped, try +/-8g: writeReg(0x23, 0x28); recalibrate after.

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(10, 30, "warming up...");
  u8g2.sendBuffer();

#if CAL_MODE
  Serial.println("variance,rhythm");   // CSV header for capture
#endif
}

void loop() {
  static unsigned long nextT = 0;
  unsigned long now = millis();
  if (nextT == 0) nextT = now;
  if ((long)(now - nextT) < 0) return;
  nextT += 1000 / ODR_HZ;

  int16_t x = axis(0x28, 0x29) >> 4;   // 12-bit high-res counts
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
      if (++pendingCount >= SMOOTH_N) { state = raw; pendingCount = 0; }
    } else {
      pendingState = raw;
      pendingCount = 1;
    }

#if CAL_MODE
    // pure CSV so the serial monitor capture is directly usable as a file
    Serial.print(lastVar, 1); Serial.print(","); Serial.println(lastRhythm, 3);
#endif
    drawScreen();
  }
}
