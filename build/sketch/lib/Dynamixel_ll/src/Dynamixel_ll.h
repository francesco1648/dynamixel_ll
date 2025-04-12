#line 1 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\lib\\Dynamixel_ll\\src\\Dynamixel_ll.h"
#ifndef DYNAMIXEL_LL_H
#define DYNAMIXEL_LL_H

#include <Arduino.h>
struct StatusPacket {
    bool valid;
    uint8_t error;
    uint8_t data[4]; // Max 4 byte
    uint8_t dataLength;
};

class DynamixelLL {
public:
    DynamixelLL(HardwareSerial& serial, uint8_t servoID);

    void begin(long baudrate = 57600);
    void ledOff();
    void sendPacket(const uint8_t* packet, size_t length);
    void writeRegister(uint16_t address, uint32_t value, uint8_t size);
    bool readRegister(uint16_t address, uint32_t &value, uint8_t size) ;
    void readResponse();
    StatusPacket readStatusPacket(uint8_t expectedParams);
    void setDebug(bool enable);

private:

    HardwareSerial& _serial;
    uint8_t _servoID;
    uint16_t calculateCRC(const uint8_t* data, size_t length);
    bool _debug = false;

};

#endif
