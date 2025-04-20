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
   StatusPacket response = recivePacket(sizeResponse); // Legge la risposta
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


uint8_t DynamixelLL::readRegister(uint16_t address, uint32_t &value, uint8_t size)
{
    uint8_t packet[14];
    uint16_t length = 7;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = 0xFD;
    packet[3] = 0x00;
    packet[4] = _servoID;
    packet[5] = length & 0xFF;
    packet[6] = (length >> 8) & 0xFF;
    packet[7] = 0x02; // READ
    packet[8] = address & 0xFF;
    packet[9] = (address >> 8) & 0xFF;
    packet[10] = size & 0xFF;
    packet[11] = (size >> 8) & 0xFF;

    uint16_t crc = calculateCRC(packet, 12);
    packet[12] = crc & 0xFF;
    packet[13] = (crc >> 8) & 0xFF;

    sendPacket(packet, 14);
    delay(time_delay);

    StatusPacket response = recivePacket(size);
    if (!response.valid || response.error != 0)
    {
        if (_debug)
        {
            Serial.print("Errore nella risposta: ");
            Serial.println(response.error, HEX);
        }

    }

    // Converte i dati in uint32_t (little-endian)
    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++)
    {
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


StatusPacket DynamixelLL::recivePacket(uint8_t expectedParams)
{
    StatusPacket result = {false, 0, {0}, 0};

    uint8_t response[20]; // Buffer per la risposta
    size_t index = 0;
    unsigned long start = millis();
    const unsigned long timeout = 1000;

    // Attesa risposta
    while (millis() - start < timeout && index < sizeof(response))
    {
        if (_serial.available())
        {
            response[index++] = _serial.read();
        }
    }

    if (index < 7)
    {
        if (_debug)
        {
            Serial.println("Risposta troppo corta");
        }
        return result;
    }

    // Controllo intestazione
    if (!(response[0] == 0xFF && response[1] == 0xFF && response[2] == 0xFD && response[3] == 0x00))
    {
        if (_debug)
        {
            Serial.println("Header non valido");
        }
        return result;
    }

    // ID, lunghezza, istruzione
    uint8_t id = response[4];
    uint16_t length = response[5] | (response[6] << 8);
    uint8_t instruction = response[7];

    if (instruction != 0x55)
    { // Status packet
        if (_debug)
        {
            Serial.println("Istruzione non valida nella risposta");
        }
        return result;
    }

    // Errore
    result.error = response[8];

    // Dati (se presenti)
    size_t paramLength = length - 4; // Escludi: instruction (1), error (1), CRC (2)
    if (paramLength != expectedParams)
    {
        if (_debug)
        {
            Serial.println("Numero di parametri inatteso");
        }
    }

    for (size_t i = 0; i < paramLength && i < 4; i++)
    {
        result.data[i] = response[9 + i];
    }
    result.dataLength = paramLength;

    // CRC
    uint16_t receivedCRC = response[9 + paramLength] | (response[10 + paramLength] << 8);
    uint16_t computedCRC = calculateCRC(response, 9 + paramLength);
    if (receivedCRC != computedCRC)
    {
        if (_debug)
        {
            Serial.println("CRC non valido");
        }
        return result;
    }

    result.valid = true;
    return result;
}


uint8_t DynamixelLL::ping( uint32_t &value){

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
    StatusPacket response = recivePacket(14);

    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++)
    {
        value |= (response.data[i] << (8 * i));
    }
    return response.error;
}


bool DynamixelLL::syncWriteGoalPositions(const uint8_t* ids, const uint32_t* positions, uint8_t length)
{
    const uint16_t address = 116; // Address for Goal Position
    const uint16_t dataLength = 4; // 4 bytes per Goal Position

    const uint16_t paramLength = (dataLength + 1) * length; // +1 for ID
    uint8_t params[paramLength];

    for (uint8_t i = 0; i < length; ++i) {
      params[i * (dataLength + 1)] = ids[i];
      uint32_t pos = positions[i];
      params[i * (dataLength + 1) + 1] = pos & 0xFF;
      params[i * (dataLength + 1) + 2] = (pos >> 8) & 0xFF;
      params[i * (dataLength + 1) + 3] = (pos >> 16) & 0xFF;
      params[i * (dataLength + 1) + 4] = (pos >> 24) & 0xFF;
    }

    return sendSyncWritePacket(address, dataLength, params, paramLength);
  }


bool DynamixelLL::sendSyncWritePacket(uint16_t address, uint16_t dataLength, const uint8_t* params, uint16_t paramLength)
{
    uint8_t packet[10 + paramLength];
    uint16_t idx = 0;

    packet[idx++] = 0xFF;
    packet[idx++] = 0xFF;
    packet[idx++] = 0xFD;
    packet[idx++] = 0x00;
    packet[idx++] = 0xFE; // Broadcast ID

    uint16_t length = 3 + 2 + 2 + paramLength; // INST + addr + len + data
    packet[idx++] = length & 0xFF;
    packet[idx++] = (length >> 8) & 0xFF;

    packet[idx++] = 0x83; // Sync Write instruction

    packet[idx++] = address & 0xFF;
    packet[idx++] = (address >> 8) & 0xFF;
    packet[idx++] = dataLength & 0xFF;
    packet[idx++] = (dataLength >> 8) & 0xFF;

    for (uint16_t i = 0; i < paramLength; ++i)
      packet[idx++] = params[i];

    uint16_t crc = calculateCRC(packet, idx);
    packet[idx++] = crc & 0xFF;
    packet[idx++] = (crc >> 8) & 0xFF;

    return sendRawPacket(packet, idx); // Funzione che invia il pacchetto sulla seriale
}

bool DynamixelLL::sendRawPacket(const uint8_t* packet, uint16_t length)
{
    // Svuota la seriale prima di inviare
    while (_serial.available()) {
      _serial.read();
    }

    // Invia il pacchetto byte per byte
    for (uint16_t i = 0; i < length; ++i) {
      _serial.write(packet[i]);
    }

    _serial.flush(); // Assicurati che la trasmissione sia completata

    return true; // Se vuoi, potresti controllare anche errori
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
{
    uint8_t value = enable ? 1 : 0;
    return writeRegister(64, value, 1);

}


uint8_t DynamixelLL::setLED( bool enable)
{
    uint8_t value = enable ? 1 : 0;
     return writeRegister(65, value, 1);

}


uint8_t DynamixelLL::setStatusReturnLevel(uint8_t level)
{
     return writeRegister(68, level, 1);

}


uint8_t DynamixelLL::setBaudRate(uint32_t baudRate)
{
     return writeRegister(8, baudRate, 1);

}


uint8_t DynamixelLL::setReturnDelayTime(uint32_t delayTime)
{
     return writeRegister(9, delayTime, 1);

}


uint8_t DynamixelLL::getPresentPosition(uint32_t &presentPosition)
{
    return  readRegister(132, presentPosition, 4);

}


uint8_t DynamixelLL::setID(uint8_t newID)
{
    return writeRegister(7, newID, 1);
}


uint8_t DynamixelLL::setVelocity(uint32_t velocity)
{
    return writeRegister(112, velocity, 4);
}
