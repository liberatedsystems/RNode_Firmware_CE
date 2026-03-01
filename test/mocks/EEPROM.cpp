#include "EEPROM.h"

EEPROMClass EEPROM;

void EEPROMClass::begin(size_t size) {
    data.resize(size, 0xFF);
}

uint8_t EEPROMClass::read(int address) {
    if (address >= 0 && address < data.size()) {
        return data[address];
    }
    return 0xFF;
}

void EEPROMClass::write(int address, uint8_t value) {
    if (address >= 0 && address < data.size()) {
        data[address] = value;
    }
}

void EEPROMClass::commit() {
    // Do nothing for mock
}
