#ifndef DYNAMIXEL_LL_H
#define DYNAMIXEL_LL_H

// define a special error code for sendSyncReadPacket return
#define SYNC_READ_ERR_SEND 0xFF

#include <Arduino.h>
struct StatusPacket
{
    bool valid;
    uint8_t error;
    uint8_t data[4]; // Max 4 byte
    uint8_t dataLength;
};

// Define an enumeration for the Velocity Profile type.
enum VelocityProfileType
{
    PROFILE_NOT_USED = 0,  // 00
    RECTANGULAR      = 1,  // 01
    TRIANGULAR       = 2,  // 10
    TRAPEZOIDAL      = 3   // 11
};

// Define a structure to hold the decoded Moving Status.
struct MovingStatus
{
    uint8_t raw;                      // The raw value from register 123.
    VelocityProfileType profileType;  // Bit 5 and Bit 4: velocity profile type.
    bool followingError;              // Bit 3: 0 = following, 1 = not following.
    bool profileOngoing;              // Bit 1: 0 = completed, 1 = profile in progress.
    bool inPosition;                  // Bit 0: 0 = not arrived, 1 = arrived.
};

// The Main Class for Dynamixel Communication
class DynamixelLL
{
public:
    DynamixelLL(HardwareSerial &serial, uint8_t servoID);

    void begin(long baudrate = 57600);
    void ledOff();

    void printResponse();

    void setDebug(bool enable);
    uint8_t setOperatingMode(uint8_t mode); // EEPROM address 11, 1 byte, default value 3
    uint8_t setGoalPosition_PCM(uint16_t goalPosition); // RAM address 116, 4 bytes, mode 3
    uint8_t setGoalPosition_A_PCM(float angleDegrees); // RAM address 116, 4 bytes, mode 3
    uint8_t setGoalPosition_EPCM(int32_t extendedPosition); // RAM address 116, 4 bytes, mode 4
    uint8_t setTorqueEnable(bool enable); // RAM address 64, 1 byte, default value 0
    uint8_t setLED(bool enable); // RAM address 65, 1 byte, default value 0
    uint8_t setStatusReturnLevel(uint8_t level); // RAM address 68, 1 byte, default value 2
    uint8_t setBaudRate(uint8_t baudRate); // EEPROM address 8, 1 byte, default value 34 (57600 bps)
    uint8_t setDriveMode(bool torqueOnByGoalUpdate, bool timeBasedProfile, bool reverseMode); // EEPROM address 10, 1 byte
    uint8_t setProfileAcceleration(uint32_t profileAcceleration); // RAM address 108, 4 bytes, default value 0
    uint8_t setProfileVelocity(uint32_t profileVelocity); // RAM address 112, 4 bytes, default value 0
    uint8_t setReturnDelayTime(uint32_t delayTime); // EEPROM address 9, 1 byte, default value 250 (500 μsec)
    uint8_t setID(uint8_t newID); // EEPROM address 7, 1 byte

    uint8_t getPresentPosition(uint32_t &presentPosition); // RAM address 132, 4 bytes
    MovingStatus getMovingStatus(); // RAM address 123, 1 byte

    uint8_t ping(uint32_t &value);
    bool syncWrite(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count);
    uint8_t syncRead(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count);

private:
    HardwareSerial &_serial;
    uint8_t _servoID;
    bool _debug = false;
    uint8_t _error;

    StatusPacket receivePacket(uint8_t expectedParams);
    uint16_t calculateCRC(const uint8_t *data, size_t length);
    uint8_t writeRegister(uint16_t address, uint32_t value, uint8_t size, uint8_t sizeResponse = 11);
    uint8_t readRegister(uint16_t address, uint32_t &value, uint8_t size);

    void sendPacket(const uint8_t *packet, size_t length);
    bool sendSyncWritePacket(const uint8_t* parameters, uint16_t parametersLength);
    bool sendSyncReadPacket(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint8_t count);
    bool sendRawPacket(const uint8_t* packet, uint16_t length);

};

#endif