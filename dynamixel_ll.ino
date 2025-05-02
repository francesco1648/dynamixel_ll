#include "Dynamixel_ll.h"

// Create an instance that can be used for syncWrite (ID not used).
DynamixelLL dxl(Serial2, 0);

// Motor IDs for the two motors.
const uint8_t motorIDs[] = {210, 211}; // IDs for the motors
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// Arrays for positions, statuses, and LED settings.
uint16_t positions[numMotors];
uint32_t getpositions[2];

uint32_t setLED[numMotors];
 #define ProfileAcceleration 10
  #define ProfileVelocity 20




DynamixelLL mot_Left_1(Serial2, 210);  // ID = 1
DynamixelLL mot_Right_1(Serial2, 211);  // ID = 2
DynamixelLL mot_2(Serial2, 112);  // ID = 3
DynamixelLL mot_3(Serial2, 113);  // ID = 4
DynamixelLL mot_4(Serial2, 214);  // ID = 5
DynamixelLL mot_5(Serial2, 215);  // ID = 6
DynamixelLL mot_6(Serial2, 216);
uint32_t pos_mot_Left_1 = 2920; // Posizione iniziale del motore 1
uint32_t pos_mot_Right_1 = 1364; // Posizione iniziale del motore 2
uint32_t pos_mot_2 = 2017; // Posizione iniziale del motore 3
uint32_t pos_mot_3 = 1995; // Posizione iniziale del motore 4
uint32_t pos_mot_4 = 1066; // Posizione iniziale del motore 5
uint32_t pos_mot_5 = 2025; // Posizione iniziale del motore 6
uint32_t pos_mot_6 = 200;

int step = 15;  // Incremento/decremento per ogni pressione del tasto
int step2 = 45; // Incremento/decremento per i motori 3, 4, 5, 6
void print_motor_status(); // Function prototype

void setup() {

  Serial.begin(115200);
  while (!Serial);

  Serial2.setTX(4);
  Serial2.setRX(5);
  dxl.begin(57600);
  dxl.enableSync(motorIDs, numMotors);

mot_Right_1.setTorqueEnable(false); // Disable torque for safety
  mot_Left_1.setTorqueEnable(false); // Disable torque for safety
  mot_2.setTorqueEnable(false); // Disable torque for safety
  mot_3.setTorqueEnable(false); // Disable torque for safety
  mot_4.setTorqueEnable(false); // Disable torque for safety
  mot_5.setTorqueEnable(false); // Disable torque for safety
  mot_6.setTorqueEnable(false);

  delay(10);

  // Set the operating mode to Position Control Mode (Mode 3).
  mot_Left_1.setOperatingMode(3);
  mot_Right_1.setOperatingMode(3);
  mot_2.setOperatingMode(3);
  mot_3.setOperatingMode(3);
  mot_4.setOperatingMode(3);
  mot_5.setOperatingMode(3);
  mot_6.setOperatingMode(3);

  delay(10);



// Set Profile Velocity and Profile Acceleration for smooth motion.
mot_Left_1.setProfileVelocity(ProfileVelocity);
mot_Left_1.setProfileAcceleration(ProfileAcceleration);
mot_Right_1.setProfileVelocity(ProfileVelocity);
mot_Right_1.setProfileAcceleration(ProfileAcceleration);
mot_2.setProfileVelocity(ProfileVelocity);
mot_2.setProfileAcceleration(ProfileAcceleration);
mot_3.setProfileVelocity(ProfileVelocity);
mot_3.setProfileAcceleration(ProfileAcceleration);
mot_4.setProfileVelocity(ProfileVelocity);
mot_4.setProfileAcceleration(ProfileAcceleration);
mot_5.setProfileVelocity(ProfileVelocity);
mot_5.setProfileAcceleration(ProfileAcceleration);
mot_6.setProfileVelocity(ProfileVelocity);
mot_6.setProfileAcceleration(ProfileAcceleration);

  // Configure Drive Mode for each motor:
  mot_Left_1.setDriveMode(false, false, false);
  mot_Right_1.setDriveMode(false, false, false);
  mot_2.setDriveMode(false, false, false);
  mot_3.setDriveMode(false, false, false);
  mot_4.setDriveMode(false, false, false);
  mot_5.setDriveMode(false, false, false);
  mot_6.setDriveMode(false, false, false);

  // Enable or disable debug mode for troubleshooting
  mot_Left_1.setDebug(false);
  mot_Right_1.setDebug(false);
  mot_2.setDebug(false);
  mot_3.setDebug(false);
  mot_4.setDebug(false);
  mot_5.setDebug(false);
  mot_6.setDebug(false);



  // Enable Torque for each motor.
  dxl.setTorqueEnable(true);
  mot_2.setTorqueEnable(true);
  mot_3.setTorqueEnable(true);
  mot_4.setTorqueEnable(true);
  mot_5.setTorqueEnable(true);
  mot_6.setTorqueEnable(true);

  delay(10);
  // Initialize a known present position for troubleshooting.
  getpositions[0] = 0;
  getpositions[1] = 0;
  dxl.getPresentPosition(getpositions);
mot_2.getPresentPosition(pos_mot_2);
mot_3.getPresentPosition(pos_mot_3);
mot_4.getPresentPosition(pos_mot_4);
mot_5.getPresentPosition(pos_mot_5);
mot_6.getPresentPosition(pos_mot_6);


delay(10);
Serial.print("\nThe motors are initialised.");

}

void loop() {
  if (Serial.available()>0)
  {

    char c = Serial.read();

       switch (c)
       {
       case 'w': // W -> Motore 1 avanti
       case 'W':

        getpositions[0] += step;; // Set the position for motor 1
        getpositions[1] -= step; // Set the position for motor 2
        dxl.setGoalPosition_PCM((uint16_t[2]){(uint16_t)getpositions[0], (uint16_t)getpositions[1]});
        print_motor_status();
         break;

       case 's': // S -> Motore 1 indietro
       case 'S':

       getpositions[0] -= step;; // Set the position for motor 1
       getpositions[1] += step; // Set the position for motor 2
       dxl.setGoalPosition_PCM((uint16_t[2]){(uint16_t)getpositions[0], (uint16_t)getpositions[1]});
       print_motor_status();
         break;

       case 'd': // D -> Motore 2 avanti
       case 'D':

       getpositions[0] -= step;; // Set the position for motor 1
       getpositions[1] -= step; // Set the position for motor 2
       dxl.setGoalPosition_PCM((uint16_t[2]){(uint16_t)getpositions[0], (uint16_t)getpositions[1]});
       print_motor_status();
         break;

       case 'a': // A -> Motore 2 indietro
       case 'A':

       getpositions[0] += step;; // Set the position for motor 1
       getpositions[1] += step; // Set the position for motor 2
       dxl.setGoalPosition_PCM((uint16_t[2]){(uint16_t)getpositions[0], (uint16_t)getpositions[1]});
       print_motor_status();
         break;
        case 'y':
        case 'Y':
        pos_mot_2 += step; // Incrementa la posizione del motore 3
        mot_2.setGoalPosition_PCM( pos_mot_2);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'h':
        case 'H':
        pos_mot_2 -= step; // Decrementa la posizione del motore 3
        mot_2.setGoalPosition_PCM( pos_mot_2);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'u':
        case 'U':
        pos_mot_3 += step2; // Incrementa la posizione del motore 4
        mot_3.setGoalPosition_PCM( pos_mot_3);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'j':
        case 'J':
        pos_mot_3 -= step2; // Decrementa la posizione del motore 4
        mot_3.setGoalPosition_PCM( pos_mot_3);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'i':
        case 'I':
        pos_mot_4 -= step2; // Incrementa la posizione del motore 5
        mot_4.setGoalPosition_PCM( pos_mot_4);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'k':
        case 'K':
        pos_mot_4 += step2; // Decrementa la posizione del motore 5
        mot_4.setGoalPosition_PCM( pos_mot_4);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'o':
        case 'O':
        pos_mot_5 += step2; // Incrementa la posizione del motore 6
        mot_5.setGoalPosition_PCM( pos_mot_5);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'l':
        case 'L':
        pos_mot_5 -= step2; // Decrementa la posizione del motore 6
        mot_5.setGoalPosition_PCM( pos_mot_5);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
        break;
        case 'x':
        case 'X':
        pos_mot_6 +=step2;
        mot_6.setGoalPosition_PCM(pos_mot_6);
        print_motor_status();
        break;
        case 'z':
        case 'Z':
        pos_mot_6 -= step2; // Decrementa la posizione del motore 6
        mot_6.setGoalPosition_PCM(pos_mot_6);  // Address 65, Value 1, Size 1 byte
        print_motor_status();
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

void print_motor_status() {
  Serial.print("getpositions[0]");
  Serial.println(getpositions[0]);
  Serial.print("getpositions[1]");
  Serial.println(getpositions[1]);
  Serial.print("pos_mot_2 ");
  Serial.println(pos_mot_2);
  Serial.print("pos_mot_3 ");
  Serial.println(pos_mot_3);
  Serial.print("pos_mot_4 ");
  Serial.println(pos_mot_4);
  Serial.print("pos_mot_5 ");
  Serial.println(pos_mot_5);
  Serial.print("pos_mot_6 ");
  Serial.println(pos_mot_6);
}