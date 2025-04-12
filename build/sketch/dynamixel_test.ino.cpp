#include <Arduino.h>
#line 1 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
#include "Dynamixel_ll.h"

DynamixelLL motor1(Serial1, 211);  // ID = 1
DynamixelLL motor2(Serial1, 212);  // ID = 2
int pos1 = 2047; // Posizione iniziale del motore 1
int pos2 = 2048; // Posizione iniziale del motore 2
int step = 50;  // Incremento/decremento per ogni pressione del tasto
int val = 110;
uint32_t value = 110;
#line 10 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
void setup();
#line 40 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
void loop();
#line 10 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\dynamixel_test.ino"
void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial1.setTX(0);
  Serial1.setRX(1);
  motor1.begin(57600);
  motor2.begin(57600);

  delay(1000);


  motor1.writeRegister(64, 0, 1);  // Address 65, Value 1, Size 1 byte
  delay(1000);


/*
  val = motor1.readRegister(65, value , 1);
  Serial.println("valore letto : " + String(value));
  Serial.println("valore status : " + String(val));
delay(2000);
Serial.println("spento");
  motor1.writeRegister(65, 0, 1);  // Address 65, Value 1, Size 1 byte
  val = motor1.readRegister(65, value , 1);
  Serial.println("valore letto : " + String(value));
  Serial.println("valore status : " + String(val));
*/

}

void loop() {

  motor1.writeRegister(65, 1, 1);  // Address 65, Value 1, Size 1 byte
  delay(1000);
  val = motor1.readRegister(65, value , 1);
  Serial.println("valore letto : " + String(value));
  Serial.println("valore status : " + String(val));
  delay(1000);
  motor1.writeRegister(65, 0, 1);  // Address 65, Value 1, Size 1 byte
  delay(1000);
  val = motor1.readRegister(65, value , 1);
  Serial.println("valore letto : " + String(value));
  Serial.println("valore status : " + String(val));






}

