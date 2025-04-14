#include "Dynamixel_ll.h"

DynamixelLL dxl(Serial2, 0); // L'ID in questo caso è irrilevante per il sync write

const uint8_t motorIDs[] = {210, 211}; // ID dei due motori
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);
uint32_t positions[numMotors];

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial2.setTX(4);
  Serial2.setRX(5);
  dxl.begin(57600);

  // Abilita il torque su entrambi i motori
  DynamixelLL motor1(Serial2, 211);
  DynamixelLL motor2(Serial2, 210);
  motor1.setTorqueEnable(true);
  motor2.setTorqueEnable(true);

  delay(1000);
}

void loop() {
  // Movimento in posizione iniziale
  positions[0] = 0;
  positions[1] = 0;
  dxl.syncWriteGoalPositions(motorIDs, positions, numMotors);
  Serial.println("Posizione iniziale inviata ai motori");
  delay(2000);

  // Movimento in posizione opposta
  positions[0] = 4095;
  positions[1] = 4095;
  dxl.syncWriteGoalPositions(motorIDs, positions, numMotors);
  Serial.println("Posizione finale inviata ai motori");
  delay(2000);
}
