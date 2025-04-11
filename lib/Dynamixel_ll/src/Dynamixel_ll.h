#ifndef DYNAMIXEL_LL_H
#define DYNAMIXEL_LL_H

#include <Arduino.h>

class DynamixelLL {
public:
    DynamixelLL(HardwareSerial& serial, uint8_t servoID);

    void begin(long baudrate = 57600);
    void ledOff();
    void sendPacket(const uint8_t* packet, size_t length);
    void writeRegister(uint16_t address, uint32_t value, uint8_t size);
private:

    HardwareSerial& _serial;
    uint8_t _servoID;
    uint16_t calculateCRC(const uint8_t* data, size_t length);

};

#endif
