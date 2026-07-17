#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3DH lis = Adafruit_LIS3DH(&Wire);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();

  if (!lis.begin(0x1D)) {           // the address your scanner actually found
    Serial.println("begin(0x1D) failed");
    while (1) delay(10);
  }
  Serial.println("LIS3DH found at 0x1D!");
  lis.setRange(LIS3DH_RANGE_4_G);
  lis.setDataRate(LIS3DH_DATARATE_50_HZ);
}

void loop() {
  sensors_event_t e;
  lis.getEvent(&e);
  float mag = sqrt(e.acceleration.x * e.acceleration.x +
                   e.acceleration.y * e.acceleration.y +
                   e.acceleration.z * e.acceleration.z);
  Serial.print("mag: ");
  Serial.println(mag, 2);
  delay(100);
}