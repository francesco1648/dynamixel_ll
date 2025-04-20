#ifndef DYNAMIXEL_LL_H
#define DYNAMIXEL_LL_H

#include <Arduino.h>
struct StatusPacket
{
    bool valid;
    uint8_t error;
    uint8_t data[4]; // Max 4 byte
    uint8_t dataLength;
};

class DynamixelLL
{
public:
    DynamixelLL(HardwareSerial &serial, uint8_t servoID);

    void begin(long baudrate = 57600);
    void ledOff();

    void printResponse();

    void setDebug(bool enable);
    uint8_t setOperatingMode(uint8_t mode); // EEPROM address 11, 1 byte, default value 3
    uint8_t setGoalPosition(uint16_t goalPosition); // RAM address 116, 4 bytes, mode 3
    uint8_t setGoalPosition(float angleDegrees); // RAM address 116, 4 bytes, mode 3
    uint8_t setGoalPosition(int32_t extendedPosition); // RAM address 116, 4 bytes, mode 4
    uint8_t setTorqueEnable(bool enable); // RAM address 64, 1 byte, default value 0
    uint8_t setLED(bool enable); // RAM address 65, 1 byte, default value 0
    uint8_t setStatusReturnLevel(uint8_t level); // RAM address 68, 1 byte, default value 2
    uint8_t setBaudRate(uint8_t baudRate); // EEPROM address 8, 1 byte, default value 34 (57600 bps)
    uint8_t setID(uint8_t newID); // EEPROM address 7, 1 byte

    uint8_t getPresentPosition(uint32_t &presentPosition); // RAM address 132, 4 bytes
    uint8_t setID(uint8_t newID); // EEPROM address 7, 1 byte
    uint8_t ping(uint32_t &value);
    uint8_t setVelocity(uint32_t velocity); // RAM address 112, 4 bytes
    void setGoalPositionsSync(uint8_t* ids, uint32_t* positions, int count);
    bool syncWriteGoalPositions(const uint8_t* ids, const uint32_t* positions, uint8_t length);

private:
    HardwareSerial &_serial;
    uint8_t _servoID;
    bool _debug = false;
    uint8_t _error;

    StatusPacket recivePacket(uint8_t expectedParams);
    uint16_t calculateCRC(const uint8_t *data, size_t length);
    uint8_t writeRegister(uint16_t address, uint32_t value, uint8_t size, uint8_t sizeResponse = 11);
    uint8_t readRegister(uint16_t address, uint32_t &value, uint8_t size);

    void sendPacket(const uint8_t *packet, size_t length);
    bool sendSyncWritePacket(uint16_t address, uint16_t dataLength, const uint8_t* params, uint16_t paramLength);
    bool sendRawPacket(const uint8_t* packet, uint16_t length);

};

#endif
