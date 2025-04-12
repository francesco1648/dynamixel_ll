#line 1 "C:\\Users\\Titania\\Desktop\\isaac\\test\\dynamixel_test\\lib\\Dynamixel_ll\\src\\Dynamixel_ll.cpp"
#include "Dynamixel_ll.h"
#define time_delay 10
DynamixelLL::DynamixelLL(HardwareSerial& serial, uint8_t servoID)
    : _serial(serial), _servoID(servoID) {}

void DynamixelLL::begin(long baudrate) {
    _serial.begin(baudrate);
    delay(time_delay); // Attendi che la seriale si stabilizzi
}
void DynamixelLL::ledOff() {
    uint8_t packet[13] = {
        0xFF ,0xFF, 0xFD, 0x00, 0x01 ,0x06 ,0x00 ,0x03 ,0x41, 0x00, 0x00

    };

    uint16_t crc = calculateCRC(packet, 11);
    packet[11] = crc & 0xFF;
    packet[12] = (crc >> 8) & 0xFF;

    sendPacket(packet, 13);
    delay(time_delay);
}





void DynamixelLL::sendPacket(const uint8_t* packet, size_t length) {
    if (_debug) {
    Serial.print("Pacchetto inviato: ");
    for (size_t i = 0; i < length; ++i) {
        Serial.print("0x");
        if (packet[i] < 0x10) Serial.print("0");
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
    }
    _serial.write(packet, length);
}

uint16_t DynamixelLL::calculateCRC(const uint8_t* data_blk_ptr, size_t data_blk_size) {
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
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

      uint16_t crc_accum = 0;
      for (size_t j = 0; j < data_blk_size; j++) {
          uint16_t i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
          crc_accum = (crc_accum << 8) ^ crc_table[i];
      }
      return crc_accum;
  }




  void DynamixelLL::writeRegister(uint16_t address, uint32_t value, uint8_t size) {
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
    for (uint8_t i = 0; i < size; i++) {
        packet[10 + i] = (value >> (8 * i)) & 0xFF;
    }

    // Calcolo CRC
    size_t lenNoCRC = 10 + size; // Lunghezza pacchetto senza il CRC
    uint16_t crc = calculateCRC(packet, lenNoCRC);  // Calcolo CRC
    packet[lenNoCRC]     = crc & 0xFF;              // CRC LSB
    packet[lenNoCRC + 1] = (crc >> 8) & 0xFF;      // CRC MSB

    // Invio del pacchetto
    sendPacket(packet, lenNoCRC + 2);  // Invia il pacchetto con CRC

    readResponse();
    delay(time_delay); // Attendi un attimo per la risposta
}

bool DynamixelLL::readRegister(uint16_t address, uint32_t& value, uint8_t size) {
    uint8_t packet[14];
    uint16_t length = 7;

    packet[0] = 0xFF; packet[1] = 0xFF; packet[2] = 0xFD; packet[3] = 0x00;
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

    StatusPacket response = readStatusPacket(size);
    if (!response.valid || response.error != 0) {
        if (_debug) {
            Serial.print("Errore nella risposta: ");
            Serial.println(response.error, HEX);
        }
        return false;
    }

    // Converte i dati in uint32_t (little-endian)
    value = 0;
    for (uint8_t i = 0; i < response.dataLength; i++) {
        value |= (response.data[i] << (8 * i));
    }

    return true;
}


void DynamixelLL::readResponse() {
    // Pulisce eventuali byte rimasti nel buffer prima di leggere la risposta
    while (_serial.available()) {
        _serial.read();  // Consuma i byte nel buffer
    }

    unsigned long startMillis = millis();
    unsigned long timeout = 1000;  // Timeout di 1 secondo per la lettura
    if (_debug) {
    Serial.println("Inizio lettura risposta:");
}
    // Legge i byte dalla seriale
    while (millis() - startMillis < timeout) {
        if (_serial.available()) {
            uint8_t byte = _serial.read();
if(_debug) {
            Serial.print("0x");
            if (byte < 0x10) Serial.print("0");
            Serial.print(byte, HEX);
            Serial.print(" ");
        }
        }
    }
if(_debug) {
    Serial.println("\nFine lettura risposta");
}
}

StatusPacket DynamixelLL::readStatusPacket(uint8_t expectedParams) {
    StatusPacket result = { false, 0, {0}, 0 };

    uint8_t response[20]; // Buffer per la risposta
    size_t index = 0;
    unsigned long start = millis();
    const unsigned long timeout = 1000;

    // Attesa risposta
    while (millis() - start < timeout && index < sizeof(response)) {
        if (_serial.available()) {
            response[index++] = _serial.read();
        }
    }

    if (index < 7) {
        if (_debug) {
        Serial.println("Risposta troppo corta");
        }
        return result;
    }

    // Controllo intestazione
    if (!(response[0] == 0xFF && response[1] == 0xFF && response[2] == 0xFD && response[3] == 0x00)) {
        if (_debug) {
        Serial.println("Header non valido");
        }
        return result;
    }

    // ID, lunghezza, istruzione
    uint8_t id = response[4];
    uint16_t length = response[5] | (response[6] << 8);
    uint8_t instruction = response[7];

    if (instruction != 0x55) { // Status packet
        if (_debug) {
        Serial.println("Istruzione non valida nella risposta");
        }
        return result;
    }

    // Errore
    result.error = response[8];

    // Dati (se presenti)
    size_t paramLength = length - 4; // Escludi: instruction (1), error (1), CRC (2)
    if (paramLength != expectedParams) {
        if (_debug) {
        Serial.println("Numero di parametri inatteso");
        }
    }

    for (size_t i = 0; i < paramLength && i < 4; i++) {
        result.data[i] = response[9 + i];
    }
    result.dataLength = paramLength;

    // CRC
    uint16_t receivedCRC = response[9 + paramLength] | (response[10 + paramLength] << 8);
    uint16_t computedCRC = calculateCRC(response, 9 + paramLength);
    if (receivedCRC != computedCRC) {
        if (_debug) {
        Serial.println("CRC non valido");
        }
        return result;
    }

    result.valid = true;
    return result;
}



void DynamixelLL::setDebug(bool enable) {
    _debug = enable;
}
