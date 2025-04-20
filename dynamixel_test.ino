#include "Dynamixel_ll.h"

// Create an instance that can be used for syncWrite (ID not used).
DynamixelLL dxl(Serial2, 0); 

// Motor IDs for the two motors.
const uint8_t motorIDs[] = {4, 5};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// Arrays for positions, statuses, and LED settings.
uint32_t positions[numMotors];
uint32_t getpositions[numMotors];
uint32_t setLED[numMotors];

// Create individual motor objects for setup (if needed for individual writes).
DynamixelLL motor1(Serial2, 4);
DynamixelLL motor2(Serial2, 5);

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial2.setTX(4);
  Serial2.setRX(5);
  dxl.begin(57600);

  // Initialize a known present position for troubleshooting.
  getpositions[0] = 0;
  getpositions[1] = 0;

  // Configure Drive Mode for each motor:
  motor1.setDriveMode(true, false, false);
  motor2.setDriveMode(true, false, false);

  // Enable or disable debug mode for troubleshooting
  motor1.setDebug(true);
  motor2.setDebug(true);

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
  dxl.syncWrite(116, 4, motorIDs, positions, numMotors);
  dxl.syncWrite(65, 1, motorIDs, setLED, numMotors);
  Serial.println("\nInitial Position sent to motors\n");
  delay(3000);

  motor1.getPresentPosition(getpositions[0]);
  Serial.print("Initial Position of the first motor: ");
  Serial.println(getpositions[0]);
  motor2.getPresentPosition(getpositions[1]);
  Serial.print("Initial Position of the second motor: ");
  Serial.println(getpositions[1]);

  // Change positions and update LED states.
  positions[0] = 4095;
  positions[1] = 4095;
  setLED[0] = 1;
  setLED[1] = 0;
  dxl.syncWrite(116, 4, motorIDs, positions, numMotors);
  dxl.syncWrite(65, 1, motorIDs, setLED, numMotors);
  Serial.println("\nFinal Position sent to motors");
  delay(3000);

  motor1.getPresentPosition(getpositions[0]);
  Serial.print("Final Position of the first motor: ");
  Serial.println(getpositions[0]);
  motor2.getPresentPosition(getpositions[1]);
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
