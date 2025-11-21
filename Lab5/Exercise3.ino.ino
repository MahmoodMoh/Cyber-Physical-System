#include <Wire.h>

#define SENSOR_ADDR 0x20 // Example address, check your sensor

void setup() {
  Wire.begin(); // Initialize I2C
  Serial.begin(9600);
}

void loop() {
  // Request distance from sensor
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x01); // Command to read distance (depends on sensor)
  Wire.endTransmission();

  delay(10); // Small delay for sensor to process

  Wire.requestFrom(SENSOR_ADDR, 2); // Request 2 bytes (distance)
  if (Wire.available() == 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int distance = (highByte << 8) | lowByte;
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(1000);
}