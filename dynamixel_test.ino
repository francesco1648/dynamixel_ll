#include "Dynamixel_ll.h"

DynamixelLL motor1(Serial1, 210);  // ID = 1
DynamixelLL motor2(Serial1, 211);  // ID = 2
int pos1 = 2047; // Posizione iniziale del motore 1
int pos2 = 2048; // Posizione iniziale del motore 2
int step = 50;  // Incremento/decremento per ogni pressione del tasto

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
  motor1.writeRegister(116, pos1, 4);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(116, pos2, 4);  // Address 65, Value 1, Size 1 byte
  delay(1000);
}

void loop() {
  if (Serial.available()>0)
  {

    char c = Serial.read();

       switch (c)
       {
       case 'w': // W -> Motore 1 avanti
       case 'W':
pos1 += step; // Incrementa la posizione del motore 1
pos2 -= step; // Incrementa la posizione del motore 2
  motor1.writeRegister(116, pos1, 4);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(116, pos2, 4);  // Address 65, Value 1, Size 1 byte
         Serial.print("Motore 1 posizione: ");
         Serial.print(pos1);
         Serial.print("\t");
         Serial.print("Motore 2 posizione: ");
         Serial.println(pos2);
         break;

       case 's': // S -> Motore 1 indietro
       case 'S':
pos1 -= step; // Decrementa la posizione del motore 1
pos2 += step; // Decrementa la posizione del motore 2

  motor1.writeRegister(116, pos1, 4);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(116, pos2, 4);  // Address 65, Value 1, Size 1 byte
       Serial.print("Motore 1 posizione: ");
         Serial.print(pos1);
         Serial.print("\t");
         Serial.print("Motore 2 posizione: ");
         Serial.println(pos2);
         break;

       case 'd': // D -> Motore 2 avanti
       case 'D':
pos1 -= step; // Incrementa la posizione del motore 1
pos2 -= step; // Incrementa la posizione del motore 2

  motor1.writeRegister(116, pos1, 4);  // Address 65, Value 1, Size 1 byte
  motor2.writeRegister(116, pos2, 4);  // Address 65, Value 1, Size 1 byte
         Serial.print("Motore 1 posizione: ");
         Serial.print(pos1);
         Serial.print("\t");
         Serial.print("Motore 2 posizione: ");
         Serial.println(pos2);
         break;

       case 'a': // A -> Motore 2 indietro
       case 'A':
pos1 += step; // Decrementa la posizione del motore 1
pos2 += step; // Decrementa la posizione del motore 2

       motor1.writeRegister(116, pos1, 4);  // Address 65, Value 1, Size 1 byte
       motor2.writeRegister(116, pos2, 4);  // Address 65, Value 1, Size 1 byte
       Serial.print("Motore 1 posizione: ");
         Serial.print(pos1);
         Serial.print("\t");
         Serial.print("Motore 2 posizione: ");
         Serial.println(pos2);
         break;

       default:
         Serial.println("Tasto non riconosciuto.");
         break;
         delay(10);
       }

       while (Serial.available())
         Serial.read(); // Pulisce il buffer
         delay(10);
     }
     delay(10);







}
