#include <Arduino.h>
#include <Wire.h>

// I2C read
uint16_t readRegister(uint8_t devAddr, uint8_t reg) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(devAddr, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// I2C write
void writeRegister(uint8_t devAddr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(devAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Wire.begin(); // SDA/SCL default on Nano ESP32
  Serial.begin(115200);
  delay(1000);

  // writeRegister(0x40,0x15,0x01);
  // delay(1000);

  // writeRegister(0x44,0x03,0xFD);
  // delay(1000);

  // writeRegister(0x44,0x03,0x08);
  // delay(1000);
  
  // writeRegister(0x44,0x03,0x00);
  // delay(1000);

}

void loop() {
  uint8_t variable = readRegister(0x44,0xFE);
  Serial.println(variable);
}
