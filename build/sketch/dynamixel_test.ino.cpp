#include <Arduino.h>
#line 1 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
#include "Dynamixel_ll.h"

DynamixelLL mot_Left_1(Serial1, 210);  // ID = 1
DynamixelLL mot_Right_1(Serial1, 211);  // ID = 2
DynamixelLL mot_2(Serial1, 112);  // ID = 3
DynamixelLL mot_3(Serial1, 113);  // ID = 4
DynamixelLL mot_4(Serial1, 214);  // ID = 5
DynamixelLL mot_5(Serial1, 215);  // ID = 6
int pos_mot_Left_1 = 2920; // Posizione iniziale del motore 1
int pos_mot_Right_1 = 1364; // Posizione iniziale del motore 2
int pos_mot_2 = 2017; // Posizione iniziale del motore 3
int pos_mot_3 = 1995; // Posizione iniziale del motore 4
int pos_mot_4 = 1066; // Posizione iniziale del motore 5
int pos_mot_5 = 2025; // Posizione iniziale del motore 6
int step = 50;  // Incremento/decremento per ogni pressione del tasto

#line 17 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
void setup();
#line 67 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
void loop();
#line 17 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
void setup() {

  Serial.begin(115200);
  while (!Serial);

  Serial1.setTX(0);
  Serial1.setRX(1);
  mot_Left_1.begin(57600);
  mot_Right_1.begin(57600);
  mot_2.begin(57600);
  mot_3.begin(57600);
  mot_4.begin(57600);
  mot_5.begin(57600);
mot_Left_1.setVelocity(0);  // Imposta la velocità del motore 1
  mot_Right_1.setVelocity(0);  // Imposta la velocità del motore 2
  mot_2.setVelocity(0);  // Imposta la velocità del motore 3
  mot_3.setVelocity(0);  // Imposta la velocità del motore 4
  mot_4.setVelocity(0);  // Imposta la velocità del motore 5
  mot_5.setVelocity(0);  // Imposta la velocità del motore 6
  delay(1000);
  mot_Left_1.ledOff();
  mot_Right_1.ledOff();
  mot_2.ledOff();
  mot_3.ledOff();
  mot_4.ledOff();
  mot_5.ledOff();

  mot_Left_1.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_2.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_3.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_4.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_5.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte

  mot_Left_1.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_2.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_3.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_4.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  mot_5.writeRegister(64, 1, 1);  // Address 65, Value 1, Size 1 byte
  delay(1000);
  mot_Left_1.writeRegister(116, pos_mot_Left_1, 4);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.writeRegister(116, pos_mot_Right_1, 4);  // Address 65, Value 1, Size 1 byte
  mot_2.writeRegister(116, pos_mot_2, 4);  // Address 65, Value 1, Size 1 byte
  mot_3.writeRegister(116, pos_mot_3, 4);  // Address 65, Value 1, Size 1 byte
  mot_4.writeRegister(116, pos_mot_4, 4);  // Address 65, Value 1, Size 1 byte
  mot_5.writeRegister(116, pos_mot_5, 4);  // Address 65, Value 1, Size 1 byte
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
        pos_mot_Left_1 -= step; // Incrementa la posizione del motore 1
pos_mot_Right_1 += step; // Incrementa la posizione del motore 2
  mot_Left_1.writeRegister(116, pos_mot_Left_1, 4);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.writeRegister(116, pos_mot_Right_1, 4);  // Address 65, Value 1, Size 1 byte
         Serial.print("mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;

       case 's': // S -> Motore 1 indietro
       case 'S':
pos_mot_Left_1 += step; // Decrementa la posizione del motore 1
pos_mot_Right_1 -= step; // Decrementa la posizione del motore 2

  mot_Left_1.writeRegister(116, pos_mot_Left_1, 4);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.writeRegister(116, pos_mot_Right_1, 4);  // Address 65, Value 1, Size 1 byte
       Serial.print("pos_mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;

       case 'd': // D -> Motore 2 avanti
       case 'D':
pos_mot_Left_1 -= step; // Incrementa la posizione del motore 1
pos_mot_Right_1 -= step; // Incrementa la posizione del motore 2

  mot_Left_1.writeRegister(116, pos_mot_Left_1, 4);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.writeRegister(116, pos_mot_Right_1, 4);  // Address 65, Value 1, Size 1 byte
         Serial.print("pos_mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;

       case 'a': // A -> Motore 2 indietro
       case 'A':
pos_mot_Left_1 += step; // Decrementa la posizione del motore 1
pos_mot_Right_1 += step; // Decrementa la posizione del motore 2

       mot_Left_1.writeRegister(116, pos_mot_Left_1, 4);  // Address 65, Value 1, Size 1 byte
       mot_Right_1.writeRegister(116, pos_mot_Right_1, 4);  // Address 65, Value 1, Size 1 byte
       Serial.print("pos_mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;
        case 'y':
        case 'Y':
        pos_mot_2 += step; // Incrementa la posizione del motore 3
        mot_2.writeRegister(116, pos_mot_2, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'h':
        case 'H':
        pos_mot_2 -= step; // Decrementa la posizione del motore 3
        mot_2.writeRegister(116, pos_mot_2, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'u':
        case 'U':
        pos_mot_3 += step; // Incrementa la posizione del motore 4
        mot_3.writeRegister(116, pos_mot_3, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'j':
        case 'J':
        pos_mot_3 -= step; // Decrementa la posizione del motore 4
        mot_3.writeRegister(116, pos_mot_3, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'i':
        case 'I':
        pos_mot_4 += step; // Incrementa la posizione del motore 5
        mot_4.writeRegister(116, pos_mot_4, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'k':
        case 'K':
        pos_mot_4 -= step; // Decrementa la posizione del motore 5
        mot_4.writeRegister(116, pos_mot_4, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'o':
        case 'O':
        pos_mot_5 += step; // Incrementa la posizione del motore 6
        mot_5.writeRegister(116, pos_mot_5, 4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'l':
        case 'L':
        pos_mot_5 -= step; // Decrementa la posizione del motore 6
        mot_5.writeRegister(116, pos_mot_5, 4);  // Address 65, Value 1, Size 1 byte
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

