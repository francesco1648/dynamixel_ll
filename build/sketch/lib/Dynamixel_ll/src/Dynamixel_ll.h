#line 1 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\lib\\Dynamixel_ll\\src\\Dynamixel_ll.h"
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
    void sendPacket(const uint8_t *packet, size_t length);
    void writeRegister(uint16_t address, uint32_t value, uint8_t size);
    bool readRegister(uint16_t address, uint32_t &value, uint8_t size);
    void readResponse();
    StatusPacket readStatusPacket(uint8_t expectedParams);
    void setDebug(bool enable);
    void setGoalPosition(uint32_t goalPosition);
    void setTorqueEnable( bool enable);
    void setLED( bool enable);
    void setStatusReturnLevel(uint8_t level);
    void setBaudRate(uint32_t baudRate);

    void setReturnDelayTime(uint32_t delayTime);
    void getPresentPosition(uint32_t &presentPosition);
private : HardwareSerial &_serial;
    uint8_t _servoID;
    uint16_t calculateCRC(const uint8_t *data, size_t length);
    bool _debug = false;
};

#endif
