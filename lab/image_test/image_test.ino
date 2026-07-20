#include <U8g2lib.h>
#include <SPI.h>

// SH1106 128x64, 4-wire hardware SPI. Args: rotation, CS, DC, RESET
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, 6, 5, 4);

static const uint8_t walk1[] PROGMEM = { 0x00, 0x3C, /* ... */ };
display.drawBitmap(x, y, walk1, 32, 32, SH110X_WHITE);
display.display();

void setup() {
  SPI.setSCK(2);   // CLK  -> GP2
  SPI.setTX(3);    // MOSI -> GP3
  u8g2.begin();
}

void loop() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(20, 30, "Hello, Pico!");
  u8g2.drawStr(10, 50, "SH1106 over SPI");
  u8g2.sendBuffer();
  delay(1000);
}