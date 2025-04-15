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
    uint8_t setGoalPosition(uint32_t goalPosition);
    uint8_t setTorqueEnable( bool enable);
    uint8_t setLED( bool enable);
    uint8_t setStatusReturnLevel(uint8_t level);
    uint8_t setBaudRate(uint32_t baudRate);

    uint8_t setReturnDelayTime(uint32_t delayTime);
    uint8_t getPresentPosition(uint32_t &presentPosition);
    uint8_t setID(uint8_t newID);
    uint8_t ping( uint32_t &value);
    uint8_t setVelocity(uint32_t velocity);
    void setGoalPositionsSync(uint8_t* ids, uint32_t* positions, int count);
    bool syncWriteGoalPositions(const uint8_t* ids, const uint32_t* positions, uint8_t length);

private :
    HardwareSerial &_serial;
    uint8_t _servoID;
    bool _debug = false;
    uint8_t _error;
    StatusPacket recivePacket(uint8_t expectedParams);
    uint16_t calculateCRC(const uint8_t *data, size_t length);
    uint8_t writeRegister(uint16_t address, uint32_t value, uint8_t size, uint8_t sizeResponse = 11);
    uint8_t readRegister(uint16_t address, uint32_t &value, uint8_t size);

    void sendPacket(const uint8_t *packet, size_t length);
    bool sendSyncWritePacket(uint16_t address, uint16_t dataLength, const uint8_t* params, uint16_t paramLength) ;
    bool sendRawPacket(const uint8_t* packet, uint16_t length) ;

};

#endif