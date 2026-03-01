#include "simple_test.h"
#include "mocks/Arduino.h"
#include "mocks/EEPROM.h"

// Define necessary macros for Utilities.h
#define HAS_EEPROM 1
#define BOARD_MODEL 0x31 // BOARD_RNODE
#define PRODUCT_RNODE 0x03
#define MODEL_A4 0xA4
#define ADDR_PRODUCT 0x00
#define ADDR_MODEL 0x01
#define ADDR_HW_REV 0x02
#define ADDR_CHKSUM 0x03
#define CHECKSUMMED_SIZE 1024 // Adjust as needed
#define EEPROM_RESERVED 512
#define EEPROM_OFFSET 0

// Mock RadioInterface
class RadioInterface {
public:
    virtual int getIndex() { return 0; }
    virtual bool getRadioOnline() { return true; }
    virtual uint8_t random() { return 0; }
    virtual int8_t getTxPower() { return 0; }
    virtual void setTxPower(int8_t p) {}
    virtual void setTxPower(int8_t p, int pin) {}
    virtual uint32_t getFrequency() { return 0; }
    virtual void setFrequency(uint32_t f) {}
    virtual uint32_t getSignalBandwidth() { return 0; }
    virtual void setSignalBandwidth(uint32_t bw) {}
    virtual uint8_t getSpreadingFactor() { return 0; }
    virtual void setSpreadingFactor(uint8_t sf) {}
    virtual uint8_t getCodingRate4() { return 0; }
    virtual void setCodingRate4(uint8_t cr) {}
    virtual void updateBitrate() {}
    virtual long getBitrate() { return 0; }
};

#define INTERFACE_COUNT 1

#include "../src/misc/MD5.h"
#include "../Utilities.h"

TEST(UtilitiesTest, EepromChecksumValid) {
    // Setup EEPROM with valid data
    EEPROM.begin(1024);

    // Fill with some data
    for (int i = 0; i < CHECKSUMMED_SIZE; i++) {
        EEPROM.write(i, i % 256);
    }

    // Calculate checksum
    char *data = (char*)malloc(CHECKSUMMED_SIZE);
    for (int i = 0; i < CHECKSUMMED_SIZE; i++) {
        data[i] = EEPROM.read(i);
    }
    unsigned char *hash = MD5::make_hash(data, CHECKSUMMED_SIZE);

    // Write checksum to EEPROM
    for (int i = 0; i < 16; i++) {
        EEPROM.write(ADDR_CHKSUM + i, hash[i]);
    }

    free(data);
    free(hash);

    EXPECT_TRUE(eeprom_checksum_valid());
}

TEST(UtilitiesTest, EepromChecksumInvalid) {
    // Setup EEPROM with valid data
    EEPROM.begin(1024);

    // Fill with some data
    for (int i = 0; i < CHECKSUMMED_SIZE; i++) {
        EEPROM.write(i, i % 256);
    }

    // Calculate checksum
    char *data = (char*)malloc(CHECKSUMMED_SIZE);
    for (int i = 0; i < CHECKSUMMED_SIZE; i++) {
        data[i] = EEPROM.read(i);
    }
    unsigned char *hash = MD5::make_hash(data, CHECKSUMMED_SIZE);

    // Write checksum to EEPROM but corrupt one byte
    for (int i = 0; i < 16; i++) {
        EEPROM.write(ADDR_CHKSUM + i, hash[i]);
    }
    EEPROM.write(ADDR_CHKSUM, hash[0] + 1);

    free(data);
    free(hash);

    EXPECT_FALSE(eeprom_checksum_valid());
}

int main() {
    return RUN_ALL_TESTS();
}
