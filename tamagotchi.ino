#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <string> // Required header
#include <U8g2lib.h>
#include <SPI.h>

#define ADDR 0x1D

Adafruit_LIS3DH lis = Adafruit_LIS3DH(&Wire);

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 6, 5, 4);

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

void setup() {
  // Screen
  SPI.setSCK(2);   // CLK  -> GP2
  SPI.setTX(3);    // MOSI -> GP3
  u8g2.begin();

  
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.setSDA(0); Wire.setSCL(1);
  Wire.begin();
  Wire.setClock(100000);
  writeReg(0x20, 0x57);   // 100Hz, XYZ on
  writeReg(0x23, 0x08);   // high-res, +/-2g
  Serial.println("started (single-register reads)");
}

void loop() {
  int16_t x = axis(0x28, 0x29);
  int16_t y = axis(0x2A, 0x2B);
  int16_t z = axis(0x2C, 0x2D);
  Serial.print("raw x="); Serial.print(x >> 4);
  Serial.print(" y="); Serial.print(y >> 4);
  Serial.print(" z="); Serial.println(z >> 4);

  std::string message = std::to_string(x >> 4) + ", " + std::to_string(y >> 4) + ", " + std::to_string(z >> 4);

  // Screeh
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(20, 30, message.c_str());
  u8g2.sendBuffer();
  delay(200);
}