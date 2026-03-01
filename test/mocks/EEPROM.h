#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>
#include <vector>

class EEPROMClass {
public:
    void begin(size_t size);
    uint8_t read(int address);
    void write(int address, uint8_t value);
    void commit();

    // Helper for tests
    std::vector<uint8_t> data;
};

extern EEPROMClass EEPROM;

#endif
