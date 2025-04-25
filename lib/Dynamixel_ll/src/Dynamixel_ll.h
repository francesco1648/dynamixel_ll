// e-manual for DYNAMIXEL protocol 2.0: https://emanual.robotis.com/docs/en/dxl/protocol2/
// e-Manual for DYNAMIXEL XL430-W250: https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/

#ifndef DYNAMIXEL_LL_H
#define DYNAMIXEL_LL_H

/** 
 * @file Dynamixel_ll.h
 * @brief Interface for low‐level Dynamixel servo communication (Protocol 2.0).
 */

#include <Arduino.h>

/**
 * @struct StatusPacket
 * @brief Represents a response packet from a Dynamixel servo.
 */
struct StatusPacket {
    bool valid;         ///< True if the packet is valid.
    uint8_t error;      ///< Error code from the response.
    uint8_t data[4];    ///< Data bytes (max 4 bytes).
    uint8_t dataLength; ///< Number of data bytes returned.
};

/**
 * @enum VelocityProfileType
 * @brief Enumerates the supported velocity profile types.
 */
enum VelocityProfileType {
    PROFILE_NOT_USED = 0,  ///< Step mode (profile not used).
    RECTANGULAR      = 1,  ///< Rectangular profile.
    TRIANGULAR       = 2,  ///< Triangular profile.
    TRAPEZOIDAL      = 3   ///< Trapezoidal profile.
};

/**
 * @struct MovingStatus
 * @brief Holds the decoded status from the servo's moving status register.
 */
struct MovingStatus {
    uint8_t raw;                      ///< Raw status value.
    VelocityProfileType profileType;  ///< Velocity profile type (from bits 5-4).
    bool followingError;              ///< True if there's a following error (bit 3).
    bool profileOngoing;              ///< True if the motion profile is in progress (bit 1).
    bool inPosition;                  ///< True if the actuator is in position (bit 0).
};

/**
 * @class DynamixelLL
 * @brief Low-level driver for Dynamixel servos.
 */
class DynamixelLL {
public:
    /**
     * @brief Constructs a DynamixelLL object.
     * @param serial Reference to a HardwareSerial interface.
     * @param servoID The Dynamixel servo ID.
     */
    DynamixelLL(HardwareSerial &serial, uint8_t servoID);

    /**
     * @brief Initializes the Dynamixel interface.
     * @param baudrate The baud rate (default is 57600).
     */
    void begin(long baudrate = 57600);

    /**
     * @brief Turns off the servo LED.
     */
    void ledOff();

    /**
     * @brief Prints the last received response.
     */
    void printResponse();

    /**
     * @brief Enables or disables debugging output.
     * @param enable True to enable debug messages.
     */
    void setDebug(bool enable);

    /**
     * @brief Sets the operating mode of the servo.
     * 
     * Allowed: 1 = Velocity, 3 = Position, 4 = Extended Position, 16 = PWM.
     * @param mode The desired mode (1, 3, 4, or 16).
     * @return uint8_t 0 on success, nonzero if unsupported.
     */
    uint8_t setOperatingMode(uint8_t mode);

    /**
     * @brief Sets the actuator’s desired output position (pulses) for Position Control Mode.
     * @param goalPosition Desired position (0 to 4095 pulses).
     * @return uint8_t 0 on success.
     */
    uint8_t setGoalPosition(uint16_t goalPosition);

    /**
     * @brief Sets the actuator’s desired output position (angle in degrees) for Position Control Mode.
     * @param angleDegrees Desired angle in degrees (0 to 360 degrees).
     * @return uint8_t 0 on success.
     */
    uint8_t setGoalPosition(float angleDegrees);

    /**
     * @brief Sets the actuator’s desired output position (pulses) for Extended Position Control Mode.
     * @param extendedPosition Desired position (from -1,048,575 to +1,048,575 pulses).
     * @return uint8_t 0 on success.
     */
    uint8_t setGoalPosition(int32_t extendedPosition);

    /**
     * @brief Enables or disables torque for the DYNAMIXEL’s internal motor.
     * Lock (true) or unlock (false) all data in the servo’s EEPROM.
     * @param enable true to enable torque, false to disable.
     * @return uint8_t 0 on success.
     */
    uint8_t setTorqueEnable(bool enable);

    /**
     * @brief Turns the servo LED on or off.
     * @param enable True to turn on.
     * @return uint8_t 0 on success.
     */
    uint8_t setLED(bool enable);

    /**
     * @brief Set the DYNAMIXEL’s response policy to define which instructions will generate a status packet.
     * 
     * - 0: Return status packet for PING instructions only.
     * 
     * - 1: Return status packet for PING and READ instructions.
     * 
     * - 2: Return status packet for all instructions
     * @param level Desired level (0, 1, or 2).
     * @return uint8_t 0 on success.
     */
    uint8_t setStatusReturnLevel(uint8_t level);

    /**
     * @brief Sets the servo's new unique ID. 254 (0xFE) reserved for use as the Broadcast ID.
     * @param newID New servo ID (0-253).
     * @return uint8_t 0 on success.
     */
    uint8_t setID(uint8_t newID);

    /**
     * @brief Sets the baud rate.
     * Allowed codes: 
     * 
     * 7 -> 4.5M, 6 -> 4M, 5 -> 3M, 4 -> 2M, 3 -> 1M, 2 -> 115200, 
     * 1 (default) -> 57600, and 0 -> 9600.
     * @param baudRate Baud rate code (0-7).
     * @return uint8_t 0 on success.
     */
    uint8_t setBaudRate(uint8_t baudRate);

    /**
     * @brief Sets the delay time after which a Status Packet response will be returned.
     * @param delayTime Delay time in 2 µsec units, valid range: 0 - 254 (0-508 μsec).
     * @return uint8_t 0 on success.
     */
    uint8_t setReturnDelayTime(uint8_t delayTime);

    /**
     * @brief Configures the drive mode of the servo.
     * Constructs a 1-byte mode value where:
     * 
     * - Bit 3 (0x08): Movements are executed regardless of the current torque state.
     * 
     * - Bit 2 (0x04): Time-based profile (true) vs. velocity-based (false).
     * 
     * - Bit 0 (0x01): Reverse mode.
     * @param torqueOnByGoalUpdate True to enable torque on goal update.
     * @param timeBasedProfile True for time-based profile.
     * @param reverseMode True for reverse motion.
     * @return uint8_t 0 on success.
     */
    uint8_t setDriveMode(bool torqueOnByGoalUpdate, bool timeBasedProfile, bool reverseMode);

    /**
     * @brief Sets the Profile Acceleration.
     * 
     * Configures the motion profile’s acceleration (or allowed acceleration time).
     * In time-based mode, the value is further limited to not exceed 50% of the current Profile Velocity.
     *
     * @param profileAcceleration Desired profile acceleration.
     * @return uint8_t 0 on success, nonzero on error.
     */
    uint8_t setProfileAcceleration(uint32_t profileAcceleration);

    /**
     * @brief Sets the Profile Velocity.
     *
     * Configures the motion profile’s maximum velocity (or, in time-based mode, the total movement time).
     * For time-based profiles, remember that Profile Acceleration (register 108) must not exceed half
     * of this value.
     *
     * @param profileVelocity Desired profile velocity.
     * @return uint8_t 0 on success, nonzero on error.
     */
    uint8_t setProfileVelocity(uint32_t profileVelocity);

    /**
     * @brief Retrieves the current present position.
     * @param presentPosition Reference to store the 4-byte position.
     * @return uint8_t 0 on success.
     */
    uint8_t getPresentPosition(uint32_t &presentPosition);

    /**
     * @brief Gets the moving status of the servo.
     * @return MovingStatus Status information.
     */
    MovingStatus getMovingStatus();

    /**
     * @brief Sends a ping instruction to the Dynamixel servo.
     * Checks for the device's presence and retrieves basic information via its status packet.
     * @param[out] value Returns the model number (and firmware version bytes) as a 32‐bit value.
     * @return uint8_t 0 on success, or a nonzero error code if a problem occurs.
     */
    uint8_t ping(uint32_t &value);


    /**
     * @brief Performs a synchronous write to multiple devices.
     * @param address Starting register address.
     * @param dataLength Number of bytes to write per device.
     * @param ids Array of device IDs.
     * @param values Array of 32-bit values (only lower bytes used as specified).
     * @param count Number of devices.
     * @return bool True if the packet was sent successfully.
     */
    bool syncWrite(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count);

    /**
     * @brief Performs a synchronous read from multiple devices.
     * @param address Starting register address.
     * @param dataLength Number of bytes to read per device.
     * @param ids Array of device IDs.
     * @param values Array to store 32-bit output values.
     * @param count Number of devices.
     * @return uint8_t 0 if all responses are received successfully.
     */
    uint8_t syncRead(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count);

private:
    HardwareSerial &_serial; ///< Reference to the serial interface.
    uint8_t _servoID;        ///< Servo ID.
    bool _debug = false;     ///< Debug mode flag.
    uint8_t _error;          ///< Last error code.

    /**
     * @brief Receives a status packet from the servo.
     * @param expectedParams Expected parameter byte count.
     * @return StatusPacket The received packet.
     */
    StatusPacket receivePacket(uint8_t expectedParams);

    /**
     * @brief Calculates the 16-bit CRC checksum.
     * Computes the CRC over a given data block to detect communication errors.
     * @param data Pointer to the data over which to compute the CRC.
     * @param length Number of bytes in the data block.
     * @return uint16_t The computed 16-bit CRC value.
     */
    uint16_t calculateCRC(const uint8_t *data, uint8_t length);

    /**
     * @brief Writes a value to a register.
     * @param address Register address.
     * @param value Value to write.
     * @param size Number of bytes to write.
     * @param sizeResponse (Optional) Expected response length.
     * @return uint8_t 0 on success.
     */
    uint8_t writeRegister(uint16_t address, uint32_t value, uint8_t size, uint8_t sizeResponse = 11);

    /**
     * @brief Reads a register from the servo.
     * @param address Register address.
     * @param value Reference to store the read value.
     * @param size Number of bytes to read.
     * @return uint8_t 0 on success.
     */
    uint8_t readRegister(uint16_t address, uint32_t &value, uint8_t size);

    /**
     * @brief Performs a bulk write to multiple devices.
     * @param ids Array of device IDs.
     * @param addresses Array of register addresses.
     * @param dataLengths Array of data lengths (in bytes) for each device.
     * @param values Array of 32-bit values.
     * @param count Number of devices.
     * @return bool True if the packet was sent successfully.
     */
    bool bulkWrite(const uint8_t* ids, uint16_t* addresses, uint8_t* dataLengths, uint32_t* values, uint8_t count);

    /**
     * @brief Performs a bulk read from multiple devices.
     * @param ids Array of device IDs.
     * @param addresses Array of register addresses.
     * @param dataLengths Array of data lengths (in bytes) for each device.
     * @param values Array to store 32-bit output values.
     * @param count Number of devices.
     * @return uint8_t 0 if all responses are received successfully.
     */
    uint8_t bulkRead(const uint8_t* ids, uint16_t* addresses, uint8_t* dataLengths, uint32_t* values, uint8_t count);

    /**
     * @brief Sends a packet over the serial interface.
     * @param packet Pointer to the packet data.
     * @param length Length of the packet.
     */
    void sendPacket(const uint8_t *packet, uint8_t length);

    /**
     * @brief Sends a synchronous write packet.
     * @param parameters Parameter block for the write.
     * @param parametersLength Length of the parameter block.
     * @return bool True if sent successfully.
     */
    bool sendSyncWritePacket(const uint8_t* parameters, uint16_t parametersLength);

    /**
     * @brief Sends a synchronous read command.
     * @param address Starting address to read.
     * @param dataLength Number of bytes to read per device.
     * @param ids Array of device IDs.
     * @param count Number of devices.
     * @return bool True if sent successfully.
     */
    bool sendSyncReadPacket(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint8_t count);

    /**
     * @brief Sends a bulk write command.
     * @param parameters Parameter block for the write.
     * @param parametersLength Length of the parameter block.
     * @return bool True if sent successfully.
     */
    bool sendBulkWritePacket(const uint8_t* parameters, uint16_t parametersLength);

    /**
     * @brief Sends a bulk read command.
     * @param ids Array of device IDs.
     * @param addresses Array of register addresses.
     * @param dataLengths Array of data lengths (in bytes) for each device.
     * @param count Number of devices.
     * @return bool True if sent successfully.
     */
    bool sendBulkReadPacket(const uint8_t* ids, uint16_t* addresses, uint8_t* dataLengths, uint8_t count);

    /**
     * @brief Sends a raw packet over the serial interface.
     * @param packet Pointer to packet data.
     * @param length Length of the packet.
     * @return bool True if sent successfully.
     */
    bool sendRawPacket(const uint8_t* packet, uint16_t length);
};

#endif
