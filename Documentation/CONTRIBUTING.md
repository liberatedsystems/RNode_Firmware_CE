# Board support
If you wish to add support for a specific board to the project, all you have to do (if it's ESP32 or nRF52), is write an additional entry for `Boards.h`.

This entry should include, at a minimum, the following:
* whether the device has bluetooth / BLE
* whether the device has a PMU
* whether the device has an EEPROM (false in all cases for nRF52, true for ESP32)
* pin mappings for SPI NSS, SCLK, MOSI, MISO, modem reset and dio0
* the type of modem on the board (if undefined it defaults to SX127x)
* whether the modem has a busy pin
* RX and TX leds (preferably LEDs of different colours)

# Check this area...
see https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-spi.h#L39
Effectively, there are multiple SPI buses we can map to pins on these
devices (including the hardware SPI bus)

An example of a minimal entry can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  const int pin_cs = 20;
  const int pin_reset = 19;
  // const int pin_cs = 1; not needed here
  // const int pin_sclk = 2; not needed here
  // const int pin_mosi = 3; not needed here
  // const int pin_miso = 4; not needed here
  const int pin_dio = 18;
  // const int pin_busy = 0; not present
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
```

In some cases the SPI pins will not be required, as they will be the default pins for the SPI library supporting the board anyway, and therefore do not need overriding in the config.

If the SX1262 is being used the following should also be considered:
* whether DIO2 should be used as the RF switch (DIO2_AS_RF_SWITCH)
* whether an rf on/off switch also has to be operated (through the pin pin_rxen)
* whether a TCXO is connected to the modem (HAS_TCXO)
* the enable pin for the TCXO (if present)

An example of an entry using the SX1262 modem can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  #define MODEM SX1262
  #define DIO2_AS_RF_SWITCH true
  #define HAS_TCXO true
  #define HAS_BUSY true
  const int pin_cs = 20;
  const int pin_reset = 19;
  const int pin_rxen = 10;
  // const int pin_cs = 1; not needed here
  // const int pin_sclk = 2; not needed here
  // const int pin_mosi = 3; not needed here
  // const int pin_miso = 4; not needed here
  const int pin_dio = 18;
  const int pin_busy = 7;
  const int pin_tcxo_enable = -1;
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
```

If the SX1280 is being used, the following should also be added:
* the TXEN and RXEN pins

An example of an entry using the SX1280 modem can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  #define MODEM SX1280
  #define HAS_BUSY true
  #define HAS_RF_SWITCH_RX_TX true
  const int pin_cs = 20;
  const int pin_reset = 19;
  const int pin_rxen = 10;
  const int pin_txen = 11;
  // const int pin_cs = 1; not needed here
  // const int pin_sclk = 2; not needed here
  // const int pin_mosi = 3; not needed here
  // const int pin_miso = 4; not needed here
  const int pin_dio = 18;
  const int pin_busy = 7;
  const int pin_tcxo_enable = -1;
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
```

Please submit this, and any other support in different areas of the project your board may require, as a PR for my consideration.

# Feature request
Feature requests are welcomed, given that those requesting it are happy to write it themselves, or a contributor considers it to be important enough to them to write it themselves. They must be written and **properly** tested before being proposed as a pull request for the project on [GitHub](https://github.com/liberatedsystems/RNode_Firmware_CE). **Manufacturers are encouraged to contribute support for their products back to this repository**, and such support will be received gladly, given it does not effect support for other products or boards.

# Caveat
All contributions must not be written using **any** LLM (ChatGPT, etc.), please handwrite them **only**. Any PRs with proposed contributions which have been discovered to be written using an LLM will **NOT** be merged. The contributor concerned may rewrite their entire pull request **by hand** and it may be reconsidered for merging in the future. 
