# Board support
If you wish to add support for a specific board to the project, all you have to do (if it's ESP32 or nRF52), is write an additional entry for `Boards.h` and `Utilities.h` and the `Makefile` .

### Boards.h
This entry in `Boards.h` should include, at a minimum, the following:
* whether the device has bluetooth / BLE
* whether the device has a PMU
* whether the device has an EEPROM (false in all cases for nRF52, true for ESP32)
* pin mappings for SPI NSS, SCLK, MOSI, MISO, modem reset and dio0
* the type of modem on the board
* the number of interfaces (modems)
* whether the modem has a busy pin
* RX and TX leds (preferably LEDs of different colours)

You should also define a unique name for your board (with a unique value) near the beginning of the file, along with a product and model value. For example:
```
#define BOARD_MY_WICKED_BOARD 0x3B
#define PRODUCT_MY_WICKED_BOARD 0x98
#define MODEL_99 0x99
```
**Check your chosen values are not in use** in `Boards.h` first! Please also ensure you have one model per variation of the board, such as 868mhz and 915mhz variants for example.
The board definition should look as follows:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
      #define HAS_BLUETOOTH false
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 9;
      const int pin_led_tx = 8;
      const uint8_t interfaces[INTERFACE_COUNT] = {SX127X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX127X
          {
              true, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX127X
          {
              7, // pin_ss
              4, // pin_sclk
              6, // pin_mosi
              5, // pin_miso
              -1, // pin_busy
              2, // pin_dio
              3, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

```
Note, this will have to be pasted in the section according to the MCU variant,
e.g. nRF52 or ESP32. Find the section by searching for the comparison where
`MCU_VARIANT` is checked for your MCU variant. **Do not change the order of the
pins or options in any of the interface_cfg or interface_pins arrays.** You
have been warned.

[There are multiple SPI
buses](https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/esp32-hal-spi.h#L39)
we can map to pins on these devices (including the hardware SPI bus).


In some cases the SPI pins will not be required, as they will be the default pins for the SPI library supporting the board anyway, and therefore do not need overriding in the config.

If the SX1262 is being used the following should also be considered:
* the modem busy pin
* whether DIO2 should be used as the RF switch (DIO2_AS_RF_SWITCH)
* whether an RF on/off switch also has to be operated (through the pin pin_rxen)
* whether a TCXO is connected to the modem (HAS_TCXO and pin_tcxo_enable to enable the TCXO if present)
* whether the SPI pins are the default used by the SPI library

An example of an entry using the SX1262 modem can be seen below:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
  #define HAS_BLUETOOTH true
  #define HAS_PMU true
  #define HAS_EEPROM true
  #define EEPROM_SIZE 296 // minimum EEPROM size
  #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
  #define INTERFACE_COUNT 1
  const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
  const bool interface_cfg[INTERFACE_COUNT][3] = { 
                // SX1262
      {
          false, // DEFAULT_SPI
          true, // HAS_TCXO
          true  // DIO2_AS_RF_SWITCH
      }
  };
  const int8_t interface_pins[INTERFACE_COUNT][10] = { 
              // SX1262
      {
          42, // pin_ss
          43, // pin_sclk
          44, // pin_mosi
          45, // pin_miso
          46, // pin_busy
          47, // pin_dio
          38, // pin_reset
          -1, // pin_txen
          37, // pin_rxen
          -1  // pin_tcxo_enable
      }
  };

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
  const int pin_led_rx = 5;
  const int pin_led_tx = 6;
  #define INTERFACE_COUNT 1
  const uint8_t interfaces[INTERFACE_COUNT] = {SX1280};
  const bool interface_cfg[INTERFACE_COUNT][3] = { 
                // SX1280
      {
          true, // DEFAULT_SPI
          false,// HAS_TCXO
          false // DIO2_AS_RF_SWITCH
      } 
  };
  const int8_t interface_pins[INTERFACE_COUNT][10] = { 
              // SX1280
      {
          24, // pin_ss
           3, // pin_sclk
          30, // pin_mosi
          29, // pin_miso
          25, // pin_busy
          15, // pin_dio
          16, // pin_reset
          20, // pin_txen
          19, // pin_rxen
          -1  // pin_tcxo_enable
      } 
  };
```

#### INTERFACE_SPI (nRF52 only)
If you are using non-default SPI pins on an nRF52 MCU variant, you **must** ensure that you add this section to the bottom of your board config:
```
      // Required because on nRF52, non-default SPI pins must be initialised when class is declared.
      const SPIClass interface_spi[1] = {
            // SX1262
            SPIClass(
                NRF_SPIM2, 
                interface_pins[0][3], 
                interface_pins[0][1], 
                interface_pins[0][2]
               )
      };
```
This will ensure the pins are set correctly in the SPI class.

### Utilities.h
You should add something similar to the following to drive the LEDs depending on your configuration:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
```
Note: this will again have to be pasted in the correct section according to
your MCU variant. Please search for the other definitions of `led_rx_on()` to
find the correct section, then find the final section by searching for the
comparison where `MCU_VARIANT` is checked for your MCU variant.

You also need to add entries to the `setTxPower()`, `eeprom_product_valid()` and `eeprom_model_valid()` functions in the same file.

In `setTxPower()`, you simply need to add an if statement for your model, e.g:
```
if (model == MODEL_99) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
```

There is no difference between `PA_OUTPUT_PA_BOOST_PIN` and `PA_OUTPUT_RFO_PIN` on boards which do not have an SX1276/8.

For `eeprom_product_valid()` you simply need to add your product value to the relevant if statment depending on what MCU variant your board uses. For example:
```
if (rval == PRODUCT_RAK4631 || rval == PRODUCT_HELTEC_T114 || rval == PRODUCT_OPENCOM_XL || rval == PRODUCT_TECHO || rval == PRODUCT_MY_WICKED_BOARD || rval == PRODUCT_HMBRW) {
```

Finally, for `eeprom_model_valid`, you must add an elif with the correct model number for the board, e.g:
```
#elif BOARD_MODEL == BOARD_MY_WICKED_BOARD
    if (model == MODEL_99) {
```

### Makefile
You can add the example target below to the makefile for your board, but **you must replace the FQBN** in the arduino-cli command with the correct one for your board.
```
firmware-wicked_esp32:
	arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3B\""
```
Pay attention the the DBOARD_MODEL= value as you must insert the one you chose earlier here.

Another entry to upload to the board. Again substitute your FQBN, and you may have to experiment with the commands to get it to flash:
#### ESP32
```
upload-wicked_esp32:
	arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32c3
	@sleep 1
	rnodeconf /dev/ttyACM0 --firmware-hash $$(./partition_hashes ./build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --chip esp32c3 --port /dev/ttyACM0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin
```
#### nRF52
```
upload-wicked_nrf52:
	arduino-cli upload -p /dev/ttyACM0 --fqbn rakwireless:nrf52:WisCoreRAK4631Board
	unzip -o build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.zip -d build/rakwireless.nrf52.WisCoreRAK4631Board
	rnodeconf /dev/ttyACM0 --firmware-hash $$(sha256sum ./build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.bin | grep -o '^\S*')
```

And one final entry to make a release for the firmware:
#### ESP32
```
release-wicked_esp32:
	arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3B\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ESP_IDF_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_wicked_esp32.boot_app0
	cp build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.bin build/rnode_firmware_wicked_esp32.bin
	cp build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_wicked_esp32.bootloader
	cp build/esp32.esp32.esp32c3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_wicked_esp32.partitions
	zip --junk-paths ./Release/rnode_firmware_wicked_esp32.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_wicked_esp32.boot_app0 build/rnode_firmware_wicked_esp32.bin build/rnode_firmware_wicked_esp32.bootloader build/rnode_firmware_wicked_esp32.partitions
	rm -r build
```
#### nRF52
```
release-wicked_nrf52:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3B\""
	cp build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.hex build/rnode_firmware_wicked_nrf52.hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application build/rnode_firmware_wicked_nrf52.hex Release/rnode_firmware_wicked_nrf52.zip
```
You can of course replace the ESP32 target with the nRF52 target, if you are building for that MCU variant, as seen in previous instructions.

Please submit this, and any other support in different areas of the project your board may require, as a PR for my consideration.

# Feature request
Feature requests are welcomed, given that those requesting it are happy to write it themselves, or a contributor considers it to be important enough to them to write it themselves. They must be written and **properly** tested before being proposed as a pull request for the project on [GitHub](https://github.com/liberatedsystems/RNode_Firmware_CE). **Manufacturers are encouraged to contribute support for their products back to this repository**, and such support will be received gladly, given it does not effect support for other products or boards.

# Caveat
All contributions must not be written using **any** LLM (ChatGPT, etc.), please handwrite them **only**. Any PRs with proposed contributions which have been discovered to be written using an LLM will **NOT** be merged. The contributor concerned may rewrite their entire pull request **by hand** and it may be reconsidered for merging in the future. 
