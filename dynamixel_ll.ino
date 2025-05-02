#include "Dynamixel_ll.h"

// Create an instance that can be used for syncWrite (ID not used).
DynamixelLL dxl(Serial2, 0); 

// Motor IDs for the two motors.
const uint8_t motorIDs[] = {10, 11};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// Arrays for positions, statuses, and LED settings.
float homingOffset[numMotors];
uint16_t positions[numMotors];
uint32_t getpositions[numMotors];
bool setLED[numMotors];

// Create individual motor objects for setup (if needed for individual writes).
DynamixelLL motor1(Serial2, motorIDs[0]);
DynamixelLL motor2(Serial2, motorIDs[1]);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial2.setTX(4);
  Serial2.setRX(5);
  dxl.begin(57600);

  // Initialize a known present position for troubleshooting.
  homingOffset[0] = -10.0f;
  homingOffset[1] = 60.0f;
  getpositions[0] = 0;
  getpositions[1] = 0;

  // Enable sync mode for multiple motor control.
  dxl.enableSync(motorIDs, numMotors);

  // Configure Drive Mode for each motor:
  motor1.setDriveMode(false, false, true);
  motor2.setDriveMode(false, false, false);

  // Set Operating Mode for each motor:
  dxl.setOperatingMode(3);

  // Set Homing Offset for each motor:
  dxl.setHomingOffset(homingOffset);

  // Enable torque for both motors.
  dxl.setTorqueEnable(true);

  // Enable or disable debug mode for troubleshooting
  motor1.setDebug(true);
  motor2.setDebug(true);
  dxl.setDebug(true);

  // Set Profile Velocity and Profile Acceleration for smooth motion.
  motor1.setProfileVelocity(400);
  motor1.setProfileAcceleration(20);
  motor2.setProfileVelocity(200);
  motor2.setProfileAcceleration(10);

  Serial.print("\nThe motors are initialised.");

  delay(2000);
}

void loop() {
  // Set initial positions and LED states.
  positions[0] = 0;
  positions[1] = 0;
  setLED[0] = 0;
  setLED[1] = 1;
  
  // Sync write Goal Position (register 116, 4 bytes) and LED (register 65, 1 byte) for all motors.
  dxl.setGoalPosition_PCM(positions);
  dxl.setLED(setLED);
  Serial.println("\nInitial Position sent to motors\n");
  delay(3000);

  // Read and show present position from the both motors.
  dxl.getPresentPosition(getpositions);
  Serial.print("Initial Position of the first motor: ");
  Serial.println(getpositions[0]);
  Serial.print("Initial Position of the second motor: ");
  Serial.println(getpositions[1]);

  // Change positions and update LED states.
  positions[0] = 4095;
  positions[1] = 4095;
  setLED[0] = 1;
  setLED[1] = 0;
  dxl.setGoalPosition_PCM(positions);
  dxl.setLED(setLED);

  Serial.println("\nFinal Position sent to motors");
  delay(3000);

  dxl.getPresentPosition(getpositions);
  Serial.print("Final Position of the first motor: ");
  Serial.println(getpositions[0]);
  Serial.print("Final Position of the second motor: ");
  Serial.println(getpositions[1]);

  // Read and display the moving status of the first motor.
  MovingStatus status = motor1.getMovingStatus();
  Serial.print("Profile Type: ");
  switch(status.profileType) {
      case TRAPEZOIDAL: Serial.println("Trapezoidal"); break;
      case TRIANGULAR:  Serial.println("Triangular"); break;
      case RECTANGULAR: Serial.println("Rectangular"); break;
      case PROFILE_NOT_USED: Serial.println("Not used (Step)"); break;
  }
  Serial.print("Following Error: ");
  Serial.println(status.followingError ? "Following" : "Not following");
  Serial.print("Profile Ongoing: ");
  Serial.println(status.profileOngoing ? "In progress" : "Completed");
  Serial.print("In-Position: ");
  Serial.println(status.inPosition ? "Arrived" : "Not arrived");


  delay(1000); // Wait for a bit before the next loop.

  // Repeat loop...
}
