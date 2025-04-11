#include "Dynamixel_ll.h"

DynamixelLL motor1(Serial1, 1);  // ID = 1
DynamixelLL motor2(Serial1, 2);  // ID = 2

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial1.setTX(0);
  Serial1.setRX(1);
  motor1.begin(57600);
  motor2.begin(57600);

  delay(1000);
  motor1.ledOff();
  motor2.ledOff();

  motor1.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte

  motor1.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  delay(1000);
}

void loop() {
  motor1.writeRegister(116, 2000, 4);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(116, 2000, 4);  // Address 65, Value 1, Size 1 byte
  delay(3000);
  motor1.writeRegister(116, 0, 4);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(116, 0, 4);  // Address 65, Value 1, Size 1 byte
  delay(3000);
}
