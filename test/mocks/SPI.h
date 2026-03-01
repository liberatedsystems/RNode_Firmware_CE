#ifndef SPI_H
#define SPI_H

#include <stdint.h>
#include "Arduino.h"

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

class SPISettings {
public:
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) {}
    SPISettings() {}
};

class SPIClass {
public:
    void begin() {}
    void beginTransaction(SPISettings settings) {}
    void endTransaction() {}
    void end() {}
    uint8_t transfer(uint8_t data) { return 0; }
    void transfer(void *buf, size_t count) {}
};

extern SPIClass SPI;

#endif
