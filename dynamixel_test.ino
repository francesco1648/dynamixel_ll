#include "Dynamixel_ll.h"

// Create an instance that can be used for syncWrite (ID not used).
DynamixelLL dxl(Serial2, 0);

// Motor IDs for the two motors.
const uint8_t motorIDs[] = {210, 211}; // IDs for the motors
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// Arrays for positions, statuses, and LED settings.
uint32_t positions[numMotors];
uint32_t getpositions[numMotors];
uint32_t setLED[numMotors];






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

void setup() {

  Serial.begin(115200);
  while (!Serial);

  Serial2.setTX(4);
  Serial2.setRX(5);
  dxl.begin(57600);

  // Initialize a known present position for troubleshooting.
  getpositions[0] = 0;
  getpositions[1] = 0;
mot_Right_1.setTorqueEnable(false); // Disable torque for safety
  mot_Left_1.setTorqueEnable(false); // Disable torque for safety
  mot_2.setTorqueEnable(false); // Disable torque for safety
  mot_3.setTorqueEnable(false); // Disable torque for safety
  mot_4.setTorqueEnable(false); // Disable torque for safety
  mot_5.setTorqueEnable(false); // Disable torque for safety

  delay(2000);

  // Set the operating mode to Position Control Mode (Mode 3).
  mot_Left_1.setOperatingMode(3);
  mot_Right_1.setOperatingMode(3);
  mot_2.setOperatingMode(3);
  mot_3.setOperatingMode(3);
  mot_4.setOperatingMode(3);
  mot_5.setOperatingMode(3);

  delay(2000);



// Set Profile Velocity and Profile Acceleration for smooth motion.
mot_Left_1.setProfileVelocity(400);
mot_Left_1.setProfileAcceleration(20);
mot_Right_1.setProfileVelocity(200);
mot_Right_1.setProfileAcceleration(10);
mot_2.setProfileVelocity(200);
mot_2.setProfileAcceleration(10);
mot_3.setProfileVelocity(200);
mot_3.setProfileAcceleration(10);
mot_4.setProfileVelocity(200);
mot_4.setProfileAcceleration(10);
mot_5.setProfileVelocity(200);
mot_5.setProfileAcceleration(10);

  // Configure Drive Mode for each motor:
  mot_Left_1.setDriveMode(true, false, false);
  mot_Right_1.setDriveMode(true, false, false);
  mot_2.setDriveMode(true, false, false);
  mot_3.setDriveMode(true, false, false);
  mot_4.setDriveMode(true, false, false);
  mot_5.setDriveMode(true, false, false);

  // Enable or disable debug mode for troubleshooting
  mot_Left_1.setDebug(false);
  mot_Right_1.setDebug(false);
  mot_2.setDebug(false);
  mot_3.setDebug(false);
  mot_4.setDebug(false);
  mot_5.setDebug(false);




  Serial.print("\nThe motors are initialised.");
  // Enable Torque for each motor.
  mot_Left_1.setTorqueEnable(true);
  mot_Right_1.setTorqueEnable(true);
  mot_2.setTorqueEnable(true);
  mot_3.setTorqueEnable(true);
  mot_4.setTorqueEnable(true);
  mot_5.setTorqueEnable(true);

  delay(2000);
  positions[0] = pos_mot_Left_1; // Set the position for motor 1
  positions[1] = pos_mot_Right_1; // Set the position for motor 2
  dxl.syncWrite(116, 4, motorIDs, positions, numMotors);
mot_2.setGoalPosition_PCM( pos_mot_2);
mot_3.setGoalPosition_PCM( pos_mot_3);
mot_4.setGoalPosition_PCM( pos_mot_4);
mot_5.setGoalPosition_PCM( pos_mot_5);
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
        positions[0] = pos_mot_Left_1; // Set the position for motor 1
        positions[1] = pos_mot_Right_1; // Set the position for motor 2
        dxl.syncWrite(116, 4, motorIDs, positions, numMotors);
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

  mot_Left_1.setGoalPosition_PCM(pos_mot_Left_1);  // Address 65, Value 1, Size 1 byte
  mot_Right_1.setGoalPosition_PCM( pos_mot_Right_1);  // Address 65, Value 1, Size 1 byte
       Serial.print("pos_mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;

       case 'd': // D -> Motore 2 avanti
       case 'D':
       pos_mot_Left_1 -= step;
       pos_mot_Right_1 -= step;
       mot_Left_1.setGoalPosition_PCM( pos_mot_Left_1);
       mot_Right_1.setGoalPosition_PCM( pos_mot_Right_1);
         Serial.print("pos_mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;

       case 'a': // A -> Motore 2 indietro
       case 'A':
       pos_mot_2 += step;
       pos_mot_Right_1 += step;

       mot_Left_1.setGoalPosition_PCM( pos_mot_Left_1);
       mot_Right_1.setGoalPosition_PCM( pos_mot_Right_1);
       Serial.print("pos_mot_Left_1 ");
         Serial.print(pos_mot_Left_1);
         Serial.print("\t");
         Serial.print("pos_mot_Right_1 ");
         Serial.println(pos_mot_Right_1);
         break;
        case 'y':
        case 'Y':
        pos_mot_2 += step; // Incrementa la posizione del motore 3
        mot_2.setGoalPosition_PCM( pos_mot_2);  // Address 65, Value 1, Size 1 byte
        break;
        case 'h':
        case 'H':
        pos_mot_2 -= step; // Decrementa la posizione del motore 3
        mot_2.setGoalPosition_PCM( pos_mot_2);  // Address 65, Value 1, Size 1 byte
        break;
        case 'u':
        case 'U':
        pos_mot_3 += step; // Incrementa la posizione del motore 4
        mot_3.setGoalPosition_PCM( pos_mot_3);  // Address 65, Value 1, Size 1 byte
        break;
        case 'j':
        case 'J':
        pos_mot_3 -= step; // Decrementa la posizione del motore 4
        mot_3.setGoalPosition_PCM( pos_mot_3);  // Address 65, Value 1, Size 1 byte
        break;
        case 'i':
        case 'I':
        pos_mot_4 += step; // Incrementa la posizione del motore 5
        mot_4.setGoalPosition_PCM( pos_mot_4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'k':
        case 'K':
        pos_mot_4 -= step; // Decrementa la posizione del motore 5
        mot_4.setGoalPosition_PCM( pos_mot_4);  // Address 65, Value 1, Size 1 byte
        break;
        case 'o':
        case 'O':
        pos_mot_5 += step; // Incrementa la posizione del motore 6
        mot_5.setGoalPosition_PCM( pos_mot_5);  // Address 65, Value 1, Size 1 byte
        break;
        case 'l':
        case 'L':
        pos_mot_5 -= step; // Decrementa la posizione del motore 6
        mot_5.setGoalPosition_PCM( pos_mot_5);  // Address 65, Value 1, Size 1 byte
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
