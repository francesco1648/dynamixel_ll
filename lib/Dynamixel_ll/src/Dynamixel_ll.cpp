#include "Dynamixel_ll.h"
#define time_delay 0
DynamixelLL::DynamixelLL(HardwareSerial &serial, uint8_t servoID)
    : _serial(serial), _servoID(servoID) {}

void DynamixelLL::begin(long baudrate)
{
    _serial.begin(baudrate);
    delay(time_delay); // Attendi che la seriale si stabilizzi
}
void DynamixelLL::ledOff()
{
    uint8_t packet[13] = {
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x06, 0x00, 0x03, 0x41, 0x00, 0x00

    };

    uint16_t crc = calculateCRC(packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    sendPacket(packet, 13);
    delay(time_delay);
}



uint16_t DynamixelLL::calculateCRC(const uint8_t *data_blk_ptr, size_t data_blk_size)
{
    unsigned short crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202};

    uint16_t crc_accum = 0;
    for (size_t j = 0; j < data_blk_size; j++)
    {
        uint16_t i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }
    return crc_accum;
}


void DynamixelLL::printResponse()
{
    // Pulisce eventuali byte rimasti nel buffer prima di leggere la risposta
    while (_serial.available())
    {
        _serial.read(); // Consuma i byte nel buffer
    }

    unsigned long startMillis = millis();
    unsigned long timeout = 1000; // Timeout di 1 secondo per la lettura

        Serial.println("Inizio lettura risposta:");

    // Legge i byte dalla seriale
    while (millis() - startMillis < timeout)
    {
        if (_serial.available())
        {
            uint8_t byte = _serial.read();

                Serial.print("0x");
                if (byte < 0x10)
                    Serial.print("0");
                Serial.print(byte, HEX);
                Serial.print(" ");

        }
    }

        Serial.println("\nFine lettura risposta");

}


void DynamixelLL::setDebug(bool enable)
{
    _debug = enable;
}


// ===============================
// ==   Instruction Functions   ==
// ===============================


uint8_t DynamixelLL::writeRegister(uint16_t address, uint32_t value, uint8_t size, uint8_t sizeResponse)
{
    // Calcolo lunghezza del pacchetto: header (4) + ID (1) + lunghezza (2) + istruzione (1) + indirizzo (2) + valore (1/2/4)
    uint16_t length = 5 + size;
    uint8_t packet[10 + size]; // header + id + length + instruction + address + data + crc

    // Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // ID
    packet[4] = _servoID;

    // Length LSB/MSB
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;

    // Instruction: WRITE (0x03)
    packet[7] = 0x03;

    // Address (LSB, MSB)
    packet[8] = address & 0xFF;
    packet[9] = (address >> 8) & 0xFF;

    // Value (1, 2 o 4 byte little endian)
    for (uint8_t i = 0; i < size; i++)
    {
        packet[10 + i] = (value >> (8 * i)) & 0xFF;
    }

    // Calcolo CRC
    size_t lenNoCRC = 10 + size;                   // Lunghezza pacchetto senza il CRC
    uint16_t crc = calculateCRC(packet, lenNoCRC); // Calcolo CRC
    packet[lenNoCRC] = crc & 0xFF;                 // CRC LSB
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;      // CRC MSB

    // Invio del pacchetto
    sendPacket(packet, lenNoCRC + 2); // Invia il pacchetto con CRC
    delay(time_delay);
   StatusPacket response = receivePacket(sizeResponse); // Legge la risposta
    if (!response.valid || response.error != 0)
    {
        if (_debug)
        {
            Serial.print("Errore nella risposta: ");
            Serial.println(response.error, HEX);
        }

    }


    return response.error;

}


// Construct a READ instruction packet to read a register from the servo.
uint8_t DynamixelLL::readRegister(uint16_t address, uint32_t &value, uint8_t size)
{
    //   - Parameter 1 & 2: starting address (LSB, MSB)
    //   - Parameter 3 & 4: data length (LSB, MSB)
    uint8_t packet[14];
    uint16_t length = 7; // 4 parameter bytes + 3 = 7

    // Header (4 bytes): 0xFF, 0xFF, 0xFD, 0x00
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // ID (1 byte): _servoID
    packet[4] = _servoID;

    // Length (2 bytes): LSB and MSB of (parameter count + 3)
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;

    // Instruction (1 byte): 0x02 (READ)
    packet[7] = 0x02; // READ instruction

    // Parameters (4 bytes): starting address (2 bytes) + data length (2 bytes)
    packet[8] = address & 0xFF;
    packet[9] = (address >> 8) & 0xFF;
    packet[10] = size & 0xFF;
    packet[11] = (size >> 8) & 0xFF;
    
    // Compute CRC for the packet excluding the CRC bytes (first 12 bytes).
    uint16_t crc = calculateCRC(packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;
    
    // Send the packet over the serial interface.
    sendPacket(packet, 14);
    delay(time_delay);
    
    // Receive the response packet.
    StatusPacket response = receivePacket(size);
    
    // Check for errors in the status packet.
    if (!response.valid || response.error != 0) {
        if (_debug) {
            Serial.print("Error in response: ");
            Serial.println(response.error, HEX);
        }
    }

    // Convert the little-endian byte array to a uint32_t value.
    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++) {
        value |= (response.data[i] << (8 * i));
    }
    
    delay(time_delay);
    return response.error;
}


void DynamixelLL::sendPacket(const uint8_t *packet, size_t length)
{
    if (_debug)
    {
        Serial.print("Pacchetto inviato: ");
        for (size_t i = 0; i < length; ++i)
        {
            Serial.print("0x");
            if (packet[i] < 0x10)
                Serial.print("0");
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    _serial.write(packet, length);
}


/**
 * @brief Receives and parses a complete Status Packet from the Dynamixel bus.
 *
 * This function implements a sliding-window mechanism to locate the packet header
 * (0xFF, 0xFF, 0xFD, 0x00) in the incoming byte stream. Once the header is found,
 * it reads the next 3 bytes (ID and LENGTH field) to determine the expected packet size:
 * total packet length = 7 + LENGTH.
 *
 * It then waits until the entire packet is received or a timeout occurs. After this,
 * it checks the instruction, extracts the error and parameter bytes, and validates the CRC.
 *
 * @param expectedParams Number of parameter bytes expected in the packet.
 * @return StatusPacket A structure with members:
 *         - valid: true if the packet is correctly received and passes the CRC check.
 *         - error: the error code from the Status Packet.
 *         - data: up to 4 bytes of returned parameter data.
 *         - dataLength: the number of parameter bytes read.
 *         If any error occurs (timeout, header error, CRC error), valid remains false.
 */
StatusPacket DynamixelLL::receivePacket(uint8_t expectedParams)
{
    StatusPacket result = {false, 0, {0}, 0};

    const size_t maxPacketSize = 64;         // Maximum allowed packet size.
    uint8_t buffer[maxPacketSize];           // Buffer for incoming bytes.
    uint16_t index = 0;                      // Current number of bytes in the buffer.
    bool headerFound = false;                // Flag to indicate header detection.
    
    uint32_t start = millis();
    const uint32_t timeout = 1000;           // Timeout in milliseconds.

    // --- Step 1. Search for the 4-byte header ---
    // We use a sliding window to detect 0xFF, 0xFF, 0xFD, 0x00.
    while ((millis() - start) < timeout && index < maxPacketSize)
    {
        if (_serial.available())
        {
            uint8_t b = _serial.read();
            buffer[index++] = b;
            // When we have at least 4 bytes, check the last 4 bytes.
            if (index >= 4)
            {
                if (buffer[index - 4] == 0xFF &&
                    buffer[index - 3] == 0xFF &&
                    buffer[index - 2] == 0xFD &&
                    buffer[index - 1] == 0x00)
                {
                    headerFound = true;
                    break;  // We found the header.
                }
            }
        }
    }
    if (!headerFound)
    {
        if (_debug)
        {
            Serial.println("Header not found within timeout");
        }
        return result;
    }

    // The header start is at index-4.
    uint16_t headerStart = index - 4;

    // --- Step 2. Read the next 3 bytes to complete the fixed header fields (ID and LENGTH) ---
    while ((millis() - start) < timeout && (index - headerStart) < 7 && index < maxPacketSize)
    {
        if (_serial.available())
        {
            buffer[index++] = _serial.read();
        }
    }
    if ((index - headerStart) < 7)
    {
        if (_debug) { Serial.println("Timeout waiting for header extension"); }
        return result;
    }

    // At this point, buffer[headerStart..headerStart+3] contains the header:
    // and buffer[headerStart+4] is the device ID, buffer[headerStart+5] and buffer[headerStart+6]
    // give the LENGTH field (little-endian).
    uint8_t id = buffer[headerStart + 4];
    uint16_t lengthField = buffer[headerStart + 5] | (buffer[headerStart + 6] << 8);
    // Total packet length = 7 (header + ID + length field) + lengthField.
    uint16_t totalPacketLength = 7 + lengthField;

    // --- Step 3. Wait until the full packet is received ---
    while ((millis() - start) < timeout && (index - headerStart) < totalPacketLength && index < maxPacketSize)
    {
        if (_serial.available())
        {
            buffer[index++] = _serial.read();
        }
    }
    if ((index - headerStart) < totalPacketLength)
    {
        if (_debug)
        {
            Serial.println("Incomplete packet received (timeout)");
        }
        return result;
    }

    // --- Step 4. Debug: Print the received packet ---
    if (_debug)
    {
        Serial.print("Received Packet: ");
        for (uint16_t i = headerStart; i < headerStart + totalPacketLength; i++)
        {
            Serial.print("0x");
            if (buffer[i] < 0x10)
                Serial.print("0");
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // --- Step 5. Parse and validate the packet ---
    // Packet structure:
    // Bytes 0-3: Header (0xFF, 0xFF, 0xFD, 0x00)
    // Byte 4: Device ID
    // Bytes 5-6: LENGTH field (little-endian)
    // Byte 7: Instruction (should be 0x55 for a Status Packet)
    // Byte 8: Error
    // Bytes 9 to 9+(paramLength-1): Parameter Data (paramLength = lengthField - 4)
    // Final 2 bytes: CRC (little-endian)
    uint8_t instruction = buffer[headerStart + 7];
    if (instruction != 0x55)
    {
        if (_debug)
        {
            Serial.println("Invalid instruction value; expected status packet (0x55)");
        }
        return result;
    }
    result.error = buffer[headerStart + 8];

    // Expected parameter length is (lengthField - 4).
    uint8_t paramLength = lengthField - 4;
    result.dataLength = paramLength;
    for (uint8_t i = 0; i < paramLength && i < 4; i++)
    {
        result.data[i] = buffer[headerStart + 9 + i];
    }

    // Read the CRC from the packet.
    uint16_t receivedCRC = buffer[headerStart + 9 + paramLength] | (buffer[headerStart + 10 + paramLength] << 8);
    // Compute CRC over the complete packet excluding the 2 CRC bytes.
    uint16_t computedCRC = calculateCRC(&buffer[headerStart], 9 + paramLength);
    if (receivedCRC != computedCRC)
    {
        if (_debug)
        {
            Serial.println("CRC invalid");
        }
        return result;
    }

    result.valid = true;
    return result;
}


uint8_t DynamixelLL::ping(uint32_t &value)
{

    uint8_t packet[10]; // header + id + length + instruction + address + data + crc

    // Header
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;

    // ID
    packet[4] = _servoID;

    // Length LSB/MSB
    packet[5] =  0x03;
    packet[6] =  0x00;

    // Instruction: ping (0x01)
    packet[7] = 0x01;

    // Calcolo CRC
    size_t lenNoCRC = 8;                   // Lunghezza pacchetto senza il CRC
    uint16_t crc = calculateCRC(packet, lenNoCRC); // Calcolo CRC
    packet[lenNoCRC] = crc & 0xFF;                 // CRC LSB
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;      // CRC MSB

    // Invio del pacchetto
    sendPacket(packet, lenNoCRC + 2); // Invia il pacchetto con CRC
    delay(time_delay);
    StatusPacket response = receivePacket(14);

    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++)
    {
        value |= (response.data[i] << (8 * i));
    }
    return response.error;
}


/**
 * @brief Performs a synchronous write command to multiple devices.
 * 
 * This function constructs a parameter block by combining:
 *   - 4 fixed bytes: starting register address (2 bytes, LSB first)
 *     and data length (2 bytes, LSB first).
 *   - For each device, a block consisting of 1 byte for the device ID
 *     followed by the data bytes in little-endian order.
 * 
 * The complete parameter block is then passed to sendSyncWritePacket(),
 * which constructs and sends the corresponding Sync Write packet.
 *
 * @param address     Starting register address for the write operation.
 * @param dataLength  Number of data bytes per device.
 * @param ids         Array of device IDs to target.
 * @param values      Array of 32-bit values to write (only lower bytes used according to dataLength).
 * @param count       Number of devices (length of the ids and values arrays).
 * @return bool       Returns true if the Sync Write packet was successfully sent; false otherwise.
 */
bool DynamixelLL::syncWrite(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count)
{
    // The fixed part of the parameter block: 4 bytes for starting address (2 bytes)
    // and data length (2 bytes)
    const uint16_t fixedParamLength = 4;

    // Each device block consists of 1 byte (Device ID) + dataLength bytes.
    const uint16_t deviceBlockLength = dataLength + 1;

    // Total parameter block: fixed part + one device block per motor.
    const uint16_t paramBlockLength = fixedParamLength + (deviceBlockLength * count);

    // Create a local buffer to hold the entire parameter block.
    uint8_t params[paramBlockLength];
    uint16_t idx = 0;

    // Fixed Parameters (4 bytes):
    // Parameter 1-2: Starting address (LSB, MSB)
    params[idx++] = address & 0xFF;
    params[idx++] = (address >> 8) & 0xFF;
    // Parameter 3-4: Data Length (LSB, MSB)
    params[idx++] = dataLength & 0xFF;
    params[idx++] = (dataLength >> 8) & 0xFF;

    // Append each device's parameter block.
    for (uint8_t i = 0; i < count; i++) {
        // Add device ID.
        params[idx++] = ids[i];
        // Add the data bytes in little-endian order.
        uint32_t val = values[i];
        for (uint8_t j = 0; j < dataLength; j++) {
            params[idx++] = (val >> (8 * j)) & 0xFF;
        }
    }

    // With the complete parameter block built, send the full sync write packet.
    return sendSyncWritePacket(params, paramBlockLength);
}


/**
 * @brief Assembles and sends a Sync Write packet for multiple devices.
 * 
 * Given the complete parameter block for a Sync Write command, this function builds the full packet,
 * which consists of:
 *   - Header (4 bytes): 0xFF, 0xFF, 0xFD, 0x00
 *   - Packet ID (1 byte): Broadcast ID (0xFE)
 *   - Length (2 bytes): (Parameter Count + 3), in little-endian order.
 *   - Instruction (1 byte): Sync Write (0x83)
 *   - Parameter Block (n bytes): as constructed by syncWrite().
 *   - CRC (2 bytes): Computed over all preceding bytes.
 * 
 * It then sends the assembled packet using sendRawPacket().
 *
 * @param parameters        The complete parameter block (fixed parameters and device-specific data).
 * @param parametersLength  The length of the parameter block.
 * @return bool             Returns true if the packet was successfully sent; false otherwise.
 */
bool DynamixelLL::sendSyncWritePacket(const uint8_t* parameters, uint16_t parametersLength)
{
    // Calculate Length field = (Parameter Block length + 3).
    uint16_t lengthField = parametersLength + 3;
    
    // Total packet size:
    //   Header (4) + Packet ID (1) + Length (2) + Instruction (1) +
    //   Parameter Block (parametersLength) + CRC (2)
    uint16_t packetSize = 10 + parametersLength;
    uint8_t packet[packetSize];
    uint16_t idx = 0;
    
    // Build Header (4 bytes)
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;

    // Packet ID (Broadcast ID: 0xFE)
    packet[idx++] = 0xFE;

    // Length (2 bytes): (Parameter Count + 3) in little-endian.
    packet[idx++] = lengthField & 0xFF;          // LSB
    packet[idx++] = (lengthField >> 8) & 0xFF;     // MSB

    // Instruction (1 byte): Sync Write (0x83)
    packet[idx++] = 0x83;
    
    // Copy the parameter block.
    memcpy(&packet[idx], parameters, parametersLength);
    idx += parametersLength;
    
    // Compute and append CRC (2 bytes).
    uint16_t crc = calculateCRC(packet, packetSize - 2);
    packet[idx++] = crc & 0xFF;          // CRC LSB
    packet[idx++] = (crc >> 8) & 0xFF;     // CRC MSB
    
    // Optionally, print packet for debugging:
    if (_debug) {
        Serial.print("Sync Write Packet: ");
        for (uint16_t i = 0; i < packetSize; ++i) {
            Serial.print("0x");
            if (packet[i] < 0x10) Serial.print("0");
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Send the assembled packet.
    return sendRawPacket(packet, packetSize);
}


/**
 * @brief Sends a raw packet to the serial interface.
 *
 * This function flushes any residual data in the input buffer, writes the entire packet
 * to the serial port in one call, and then flushes the transmission buffer.
 * It returns true if the number of bytes written matches the expected length.
 *
 * @param packet Pointer to the packet data.
 * @param length The number of bytes in the packet.
 * @return true if the packet is sent successfully; false otherwise.
 */
bool DynamixelLL::sendRawPacket(const uint8_t* packet, uint16_t length)
{
    // Clear any pending data from the serial input buffer.
    while (_serial.available()) {
        _serial.read();
    }

    // Write the entire packet in one go.
    size_t bytesWritten = _serial.write(packet, length);

    // Ensure that the transmission is complete.
    _serial.flush();

    // Check that the number of bytes written equals the packet length.
    return (bytesWritten == length);
}


/**
 * @brief Builds and sends a Sync Read instruction packet.
 * 
 * Packet structure:
 *   Header (4 bytes):               0xFF, 0xFF, 0xFD, 0x00
 *   Packet ID (1 byte):             Broadcast ID (0xFE)
 *   Length (2 bytes):               (Fixed parameters + device IDs + 3), little-endian.
 *   Instruction (1 byte):           0x82 (Sync Read)
 *   Fixed Parameters (4 bytes):     starting address (2 bytes, LSB first) and data length (2 bytes, LSB first)
 *   Device ID(s) (1 per device):    The list of device IDs to read.
 *   CRC (2 bytes):                  Calculated over all previous bytes.
 * 
 * @param address     Starting register address to read.
 * @param dataLength  Number of bytes to read per device.
 * @param ids         Array of device IDs.
 * @param count       Count of devices.
 * @return true if the packet was sent successfully, false otherwise.
 */
bool DynamixelLL::sendSyncReadPacket(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint8_t count)
{
    // Fixed parameter block = 4 bytes (address (2) + dataLength (2))
    const uint16_t fixedParamLength = 4;
    const uint16_t paramBlockLength = fixedParamLength + count; // Add 1 byte per device.
    
    // LENGTH = (Parameter count + 3)
    uint16_t lengthField = paramBlockLength + 3;
    
    // Total packet size:
    //   Header (4) + Packet ID (1) + Length (2) + Instruction (1) +
    //   Parameter Block (parametersLength) + CRC (2)
    uint16_t packetSize = 10 + paramBlockLength;
    uint8_t packet[packetSize];
    uint16_t idx = 0;
    
    // Header (4 bytes)
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;
    
    // Packet ID (1 byte): Broadcast ID (0xFE)
    packet[idx++] = 0xFE;
    
    // Length (2 bytes) - little-endian.
    packet[idx++] = lengthField & 0xFF;
    packet[idx++] = (lengthField >> 8) & 0xFF;
    
    // Instruction (1 byte): Sync Read (0x82)
    packet[idx++] = 0x82;
    
    // Fixed Parameters (4 bytes):
    // Starting address (2 bytes, little-endian)
    packet[idx++] = address & 0xFF;
    packet[idx++] = (address >> 8) & 0xFF;
    // Data length (2 bytes, little-endian)
    packet[idx++] = dataLength & 0xFF;
    packet[idx++] = (dataLength >> 8) & 0xFF;
    
    // Device IDs (1 byte for each device)
    for (uint8_t i = 0; i < count; ++i) {
        packet[idx++] = ids[i];
    }
    
    // Compute CRC for the packet (excluding the final 2 CRC bytes)
    uint16_t crc = calculateCRC(packet, packetSize - 2);
    packet[idx++] = crc & 0xFF;       // CRC LSB
    packet[idx++] = (crc >> 8) & 0xFF;  // CRC MSB
    
    // Optionally, print packet for debugging.
    if (_debug) {
        Serial.print("Sync Read Packet: ");
        for (uint16_t i = 0; i < packetSize; ++i) {
            Serial.print("0x");
            if (packet[i] < 0x10) Serial.print("0");
            Serial.print(packet[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    // Send the assembled packet.
    return sendRawPacket(packet, packetSize);
}


/**
 * @brief Performs a Sync Read from multiple devices.
 * 
 * This function sends a Sync Read command and then reads a Status Packet from each device.
 * The received data is converted from little-endian format into a 32-bit value and stored in the provided array.
 *
 * @param address     Starting register address to read.
 * @param dataLength  Number of bytes to read per device.
 * @param ids         Array of device IDs.
 * @param values      Output array to store the 32-bit values read from each device.
 * @param count       Count of devices.
 * @return uint8_t  Returns 0 if all responses are OK.
 *                  If sending fails, returns SYNC_READ_ERR_SEND.
 *                  Or if any status packet indicates an error, returns the nonzero error code.
 */
uint8_t DynamixelLL::syncRead(uint16_t address, uint8_t dataLength, const uint8_t* ids, uint32_t* values, uint8_t count)
{
    // Send Sync Read Instruction Packet.
    if (!sendSyncReadPacket(address, dataLength, ids, count)) {
        if (_debug) {
            Serial.println("Error sending Sync Read packet.");
        }
        return SYNC_READ_ERR_SEND;
    }
    
    uint8_t retError = 0;
    // For each device, read its response.
    for (uint8_t i = 0; i < count; i++) {
        StatusPacket response = receivePacket(dataLength);
        if (!response.valid || response.error != 0) {
            if (_debug) {
                Serial.print("Error in status packet from device ");
                Serial.print(ids[i]);
                Serial.print(": 0x");
                Serial.println(response.error, HEX);
            }
            retError = response.error;
        }
        // Convert the little-endian data bytes into a 32-bit value.
        uint32_t value = 0;
        for (uint8_t j = 0; j < response.dataLength; j++) {
            value |= (response.data[j] << (8 * j));
        }
        values[i] = value;
    }
    
    return retError;
}


// ===============================
// ==  Control Table Functions  ==
// ===============================


/**
 * @brief Sets the Operating Mode of the Dynamixel servo.
 *
 * The allowed operating mode codes are:
 *   1 = Velocity Control Mode,
 *   3 = Position Control Mode,
 *   4 = Extended Position Control Mode,
 *   16 = PWM Control Mode.
 *
 * If an unsupported mode is provided, the function returns an error code (1)
 * without attempting to write to the register.
 *
 * @param mode The desired operating mode (1, 3, 4, or 16).
 * @return uint8_t Returns 0 on success, or a nonzero error code if the mode is unsupported.
 */
uint8_t DynamixelLL::setOperatingMode(uint8_t mode)
{
    // Allowed: 1 = Velocity, 3 = Position, 4 = Extended, 16 = PWM.
    if (!(mode == 1 || mode == 3 || mode == 4 || mode == 16))
    {
        return 1; // error: unsupported mode
    }
    return writeRegister(11, mode, 1);
}

/**
 * @brief Sets the Goal Position (Position Control Mode) using a raw pulse value.
 *
 * The valid range for a pulse value is from 0 to 4095.
 * If the provided value exceeds 4095, it is clamped, and the developer is notified
 * via debug messages (if enabled). Note: The servo internally also limits the value; the main purpose
 * of the clamping here is to aid debugging.
 *
 * @param goalPosition The target position in pulses (0 to 4095).
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setGoalPosition(uint16_t goalPosition)
{
    // Clamp within the valid range: 0 - 4095 pulses.
    if (goalPosition > 4095) {
        goalPosition = 4095;
        if (_debug)
        {
            Serial.println("Warning: Goal position clamped to 4095.");
        }
    }
    return writeRegister(116, goalPosition, 4); // Write 4 bytes to address 116.
}

/**
 * @brief Sets the Goal Position (Position Control Mode) using an angle in degrees.
 *
 * The function converts the given angle (in degrees) to the corresponding number of pulses
 * using a conversion factor of 0.088 deg/pulse. If the resulting value exceeds the maximum allowed (4095),
 * it is clamped, with a debug message to notify the user. The servo performs internal clamping as well,
 * but this function is intended to provide early debugging feedback for out-of-range values.
 *
 * @param angleDegrees The desired goal position in degrees.
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setGoalPosition(float angleDegrees)
{
    // Convert angle in degrees to pulses using conversion factor 0.088 [deg/pulse].
    uint32_t goalPosition = static_cast<uint32_t>(angleDegrees / 0.088);
    if (goalPosition > 4095) {
        goalPosition = 4095;
        if (_debug)
        {
            Serial.println("Warning: Angle conversion resulted in value exceeding 4095, clamped.");
        }
    }
    return writeRegister(116, goalPosition, 4);
}

/**
 * @brief Sets the Goal Position in Extended Position Control Mode.
 *
 * Extended Position Control Mode supports negative position values.
 * The valid range for extended positions is from -1,048,575 to +1,048,575 pulses.
 * If the provided value falls outside this range, it is clamped, and a debug message is output.
 *
 * @param extendedPosition The desired position as a 32-bit signed integer.
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setGoalPosition(int32_t extendedPosition)
{
    // Clamp within valid range: -1,048,575 to +1,048,575 pulses.
    if (extendedPosition > 1048575) {
        extendedPosition = 1048575;
        if (_debug)
        {
            Serial.println("Warning: Extended position clamped to 1048575.");
        }
    } else if (extendedPosition < -1048575) {
        extendedPosition = -1048575;
        if (_debug)
        {
            Serial.println("Warning: Extended position clamped to -1048575.");
        }
    }
    // Use two's complement representation for 4-byte register.
    return writeRegister(116, static_cast<uint32_t>(extendedPosition), 4);
}

/**
 * @brief Enables or disables torque on the Dynamixel servo.
 *
 * When Torque is enabled (value 1), the servo’s EEPROM is locked and the motor becomes active.
 * When Torque is disabled (value 0), the servo’s EEPROM is unlocked and the motor deactivates.
 *
 * @param enable true to enable torque, false to disable.
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setTorqueEnable(bool enable)
{
    uint8_t value = enable ? 1 : 0;
    return writeRegister(64, value, 1);
}

/**
 * @brief Turns the LED on or off on the Dynamixel servo.
 *
 * @param enable true to turn the LED on, false to turn it off.
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setLED(bool enable)
{
    uint8_t value = enable ? 1 : 0;
    return writeRegister(65, value, 1);
}

/**
 * @brief Sets the Status Return Level of the Dynamixel servo.
 *
 * The Status Return Level defines which instructions will generate a status packet:
 *   - 0: Return status packet for PING instructions only.
 *   - 1: Return status packet for PING and READ instructions.
 *   - 2: Return status packet for all instructions.
 *
 * Note: When broadcasting (ID 0xFE), no status packet is returned.
 *
 * @param level The desired status return level (0, 1, or 2).
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setStatusReturnLevel(uint8_t level)
{
    return writeRegister(68, level, 1);
}


/**
 * @brief Sets the unique ID of the Dynamixel actuator.
 * 
 * The actuator’s ID is used to uniquely address it on the Dynamixel bus. Valid values 
 * range from 0 to 253. Note that 254 (0xFE) is reserved as the Broadcast ID, which 
 * cannot be assigned to an individual actuator. In a network of Dynamixel devices, each 
 * device must have a unique ID to avoid collision.
 * 
 * This function writes the new ID to Control Table address 7.
 * 
 * @param newID The new ID to assign (0 <= newID <= 253).
 * @return uint8_t Returns 0 on success, or a nonzero error code if the write operation fails.
 */
uint8_t DynamixelLL::setID(uint8_t newID)
{
    return writeRegister(7, newID, 1);
}


/**
 * @brief Sets the baud rate for communication with the Dynamixel servo.
 *
 * The function verifies that the provided baud rate code is valid. The allowed codes are:
 *   1  -> 1,000,000 bps,
 *   3  -> 500,000 bps,
 *   4  -> 400,000 bps,
 *   7  -> 250,000 bps,
 *   9  -> 200,000 bps,
 *   16 -> 115,200 bps,
 *   34 -> 57,600 bps,
 *   103 -> 19,200 bps,
 *   207 -> 9,600 bps.
 *
 * If an invalid baud rate code is provided, the function outputs a debug error message
 * and returns an error code.
 *
 * @param baudRate The baud rate code selected.
 * @return uint8_t Returns 0 on success, or a nonzero error code if the baud rate is invalid or communication fails.
 */
uint8_t DynamixelLL::setBaudRate(uint8_t baudRate)
{
    const uint8_t allowed[] = { 1, 3, 4, 7, 9, 16, 34, 103, 207 };
    bool valid = false;
    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++)
    {
        if (allowed[i] == baudRate)
        {
            valid = true;
            break;
        }
    }
    if (!valid)
    {
        if (_debug)
        {
            Serial.print("Error: setBaudRate received an unrecognized baud rate code: ");
            Serial.println(baudRate);
        }
        return 1; // error code indicating invalid baud rate.
    }
    return writeRegister(8, baudRate, 1);
}

/**
 * @brief Sets the Return Delay Time for the Dynamixel servo.
 *
 * The Return Delay Time defines the delay between receiving an instruction and sending
 * a status packet, in units of 2 μsec. The valid range is 0 to 254.
 * If a value beyond 254 is provided, it is clamped to 254 (and a debug message is issued).
 *
 * @param delayTime The desired delay time (in units of 2 μsec).
 * @return uint8_t Returns 0 on success or a nonzero error code if communication fails.
 */
uint8_t DynamixelLL::setReturnDelayTime(uint32_t delayTime)
{
    if (delayTime > 254)
    {
        delayTime = 254;
        if (_debug)
        {
            Serial.println("Warning: setReturnDelayTime clamped to 254.");
        }
    }
    return writeRegister(9, delayTime, 1);
}


/**
 * @brief Configures the Drive Mode of the Dynamixel actuator.
 * 
 * Drive Mode (register 10, 1 byte) controls key behaviors of the actuator:
 *   - Bit 3 (0x08): Torque On by Goal Update  
 *       • false: Movements are executed only if Torque Enable (register 64) is already set.
 *       • true : Movements are executed regardless of the current torque state (torque auto‑enabled if needed).
 *
 *   - Bit 2 (0x04): Profile Configuration  
 *       • false: Uses a Velocity-based Profile (default, unit: 0.229 [rev/min], range: 0–32767, where 0 means unlimited).
 *       • true : Uses a Time-based Profile (requires firmware V42+; unit: 1 [msec], range: 0–32737, where 0 means unlimited).
 *
 *   - Bit 0 (0x01): Normal/Reverse Mode  
 *       • false: Normal mode (CCW: positive, CW: negative)
 *       • true : Reverse mode (inverts the rotation directions).
 *
 * The function constructs the 1-byte mode value based on the Boolean parameters and writes it 
 * to Control Table register 10. It returns the error code from writeRegister().
 *
 * @param torqueOnByGoalUpdate Set true to enable auto-torque on goal update.
 * @param timeBasedProfile     Set true to use time-based profiling (if supported by firmware).
 * @param reverseMode          Set true to invert the directional mapping.
 * @return uint8_t Returns 0 on success, nonzero if writing to register 10 fails.
 */
uint8_t DynamixelLL::setDriveMode(bool torqueOnByGoalUpdate, bool timeBasedProfile, bool reverseMode)
{
    uint8_t mode = 0;
    if (torqueOnByGoalUpdate) {
        mode |= 0x08; // Set Bit 3.
    }
    if (timeBasedProfile) {
        mode |= 0x04; // Set Bit 2.
    }
    if (reverseMode) {
        mode |= 0x01; // Set Bit 0.
    }
    return writeRegister(10, mode, 1);
}

/**
 * @brief Sets the Profile Velocity.
 * 
 * This function configures Profile Velocity (register 112, 4 bytes) used to generate smooth motion trajectories.
 *
 * When using a Velocity-based Profile:
 *   - Unit: 0.229 [rev/min]
 *   - Valid Range: 0 to 32767 (0 represents an infinite velocity limit)
 *
 * When using a Time-based Profile:
 *   - Unit: 1 [msec] (defines the execution time of the movement)
 *   - Valid Range: 0 to 32737 (0 represents an infinite velocity limit)
 *
 * The function first reads the Drive Mode (register 10) to determine whether a time-based profile is enabled.
 * It then clamps the input value to the maximum allowed for the current configuration.
 * If clamping occurs, a debug message is output. Finally, it writes the (possibly adjusted)
 * value to register 112.
 *
 * @param profileVelocity The desired profile velocity value.
 * @return uint8_t Returns 0 on success or a nonzero error code if writing fails.
 */
uint8_t DynamixelLL::setProfileVelocity(uint32_t profileVelocity)
{
    uint32_t driveModeTemp = 0;
    uint8_t error = readRegister(10, driveModeTemp, 1);
    uint8_t driveMode = driveModeTemp & 0xFF;
    bool timeBased = (error == 0) && ((driveMode & 0x04) != 0); 
  
    const uint32_t maxProfileVelocity = timeBased ? 32737UL : 32767UL;
    if (profileVelocity > maxProfileVelocity) {
        if (_debug) {
            Serial.print("Profile velocity value clamped to ");
            Serial.println(maxProfileVelocity);
        }
        profileVelocity = maxProfileVelocity;
    }
    return writeRegister(112, profileVelocity, 4);
}

/**
 * @brief Sets the Profile Acceleration.
 * 
 * This function configures Profile Acceleration (register 108, 4 bytes) for smooth transitions.
 *
 * When using a Velocity-based Profile:
 *   - Unit: 214.577 [rev/min^2]
 *   - Valid Range: 0 to 32767 (0 means infinite acceleration)
 *
 * When using a Time-based Profile:
 *   - Unit: 1 [msec]
 *   - Valid Range: 0 to 32737 (0 means infinite acceleration time)
 *
 * The function reads the Drive Mode (register 10) to select the correct maximum value.
 * It clamps the input acceleration value if it exceeds the allowed maximum.
 * Additionally, for time-based profiles, it checks that Profile Acceleration does not exceed
 * half of the current Profile Velocity (read from register 112). In case of clamping,
 * a debug message is output.
 *
 * @param profileAcceleration The desired acceleration value.
 * @return uint8_t Returns 0 on success, or a nonzero error code if writing fails.
 */
uint8_t DynamixelLL::setProfileAcceleration(uint32_t profileAcceleration)
{
    uint32_t driveModeTemp = 0;
    uint8_t error = readRegister(10, driveModeTemp, 1);
    uint8_t driveMode = driveModeTemp & 0xFF;
    bool timeBased = (error == 0) && ((driveMode & 0x04) != 0);
    
    const uint32_t maxProfileAcceleration = timeBased ? 32737UL : 32767UL;
    if (profileAcceleration > maxProfileAcceleration) {
        if (_debug) {
            Serial.print("Profile acceleration value clamped to ");
            Serial.println(maxProfileAcceleration);
        }
        profileAcceleration = maxProfileAcceleration;
    }
    
    // For time-based profile, ensure acceleration does not exceed half of the current profile velocity.
    uint32_t currentProfileVelocity = 0;
    error = readRegister(112, currentProfileVelocity, 4);
    if (timeBased && error == 0 && currentProfileVelocity > 0 && profileAcceleration > (currentProfileVelocity / 2)) {
        uint32_t clampedValue = currentProfileVelocity / 2;
        if (_debug) {
            Serial.print("Profile acceleration value clamped to half of current profile velocity: ");
            Serial.println(clampedValue);
        }
        profileAcceleration = clampedValue;
    } else if (error != 0 && _debug) {
        Serial.print("Error reading Profile Velocity: ");
        Serial.println(error);
    }
    
    return writeRegister(108, profileAcceleration, 4);
}


/**
 * @brief Reads the current present position of the Dynamixel actuator.
 * 
 * The present position is stored in a 4-byte register (address 132). Interpretation of the 
 * value depends on the operating mode and torque status:
 *   - With torque OFF, the value is continuous (in a 32-bit signed representation).
 *   - In Position Control Mode (Mode 3) with torque ON, the position is reset into a single 
 *     rotation range.
 * 
 * This function reads the 4-byte register and returns the result in the variable provided 
 * by reference.
 * 
 * @param presentPosition Reference variable to store the 4-byte present position value.
 * @return uint8_t Returns 0 on success, or a nonzero error code if the read operation fails.
 */
uint8_t DynamixelLL::getPresentPosition(uint32_t &presentPosition)
{
    return readRegister(132, presentPosition, 4);
}


/**
 * @brief Retrieves the Moving Status of the actuator.
 * 
 * This function reads the Moving Status (register 123, 1 byte) and decodes the following bits:
 *   - Bits 5 & 4: Velocity Profile Type, where:
 *       11 -> Trapezoidal Profile,
 *       10 -> Triangular Profile,
 *       01 -> Rectangular Profile,
 *       00 -> Profile not used (step mode).
 *   - Bit 3: Following Error (0: no error, 1: not following the desired trajectory).
 *   - Bit 1: Profile Ongoing (0: profile complete, 1: profile in progress).
 *   - Bit 0: In-Position (0: not arrived, 1: arrived at goal).
 *
 * The function returns a MovingStatus structure containing the raw value and the decoded flags.
 * If the read operation fails, the returned structure has raw value zero.
 *
 * @return MovingStatus A structure containing:
 *         - raw: The raw status byte.
 *         - profileType: The decoded velocity profile type.
 *         - followingError: True if a following error exists.
 *         - profileOngoing: True if a motion profile is still in progress.
 *         - inPosition: True if the actuator has reached the target position.
 */
MovingStatus DynamixelLL::getMovingStatus()
{
    MovingStatus status;
    status.raw = 0;  // Default value.
    
    uint32_t temp = 0;
    uint8_t error = readRegister(123, temp, 1);
    if (error != 0) {
        if (_debug) {
            Serial.print("Error reading Moving Status, error code: ");
            Serial.println(error, HEX);
        }
        return status;
    }
    
    // Extract the raw status byte (LSB) from the 4-byte value.
    status.raw = temp & 0xFF;
    
    // Decode velocity profile type (bits 5 and 4).
    uint8_t profileBits = (status.raw >> 4) & 0x03;
    status.profileType = static_cast<VelocityProfileType>(profileBits);
    
    // Decode Following Error (bit 3), Profile Ongoing (bit 1), and In-Position (bit 0).
    status.followingError = ((status.raw >> 3) & 0x01) != 0;
    status.profileOngoing = ((status.raw >> 1) & 0x01) != 0;
    status.inPosition = (status.raw & 0x01) != 0;
    
    return status;
}
