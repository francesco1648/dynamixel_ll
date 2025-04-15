#include "Dynamixel_ll.h"

// L'istanza dxl viene utilizzata per il comando Sync Write (ID irrilevante per il broadcast)
DynamixelLL dxl(Serial2, 0);

// Array degli ID dei motori e numero di motori
const uint8_t motorIDs[] = {210, 211};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// Array che conterrà le posizioni da inviare ad ogni motore
uint32_t positions[numMotors];

void setup() {
  Serial.begin(115200);
  // Attende l'apertura della seriale per il monitor seriale
  while (!Serial);

  // Imposta i pin della seriale hardware Serial1
  Serial1.setTX(4);
  Serial1.setRX(5);

  // Inizializza la comunicazione con i Dynamixel a 57600 baud
  dxl.begin(57600);

  // Abilita il torque su entrambi i motori.
  DynamixelLL motor1(Serial1, 211);
  DynamixelLL motor2(Serial1, 210);
  motor1.setTorqueEnable(true);
  motor2.setTorqueEnable(true);

  delay(1000);
}

void loop() {
  // Movimento in posizione iniziale
  positions[0] = 0;
  positions[1] = 0;
  // Invia il comando di Sync Write per impostare la posizione dei motori
  dxl.syncWriteGoalPositions(motorIDs, positions, numMotors);
  Serial.println("Posizione iniziale inviata ai motori");
  delay(2000);

  // Movimento in posizione finale (posizione opposta)
  positions[0] = 4095;
  positions[1] = 4095;
  // Invia il comando di Sync Write per impostare la nuova posizione dei motori
  dxl.syncWriteGoalPositions(motorIDs, positions, numMotors);
  Serial.println("Posizione finale inviata ai motori");
  delay(2000);
}
