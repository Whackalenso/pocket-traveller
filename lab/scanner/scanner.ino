#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.begin();
  Serial.println("I2C scanner (Wire1) ready");
}

void loop() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire1.beginTransmission(addr);
    if (Wire1.endTransmission() == 0) {
      Serial.print("Found 0x");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("Nothing found - check CS tied high");
  Serial.println("---");
  delay(2000);
}