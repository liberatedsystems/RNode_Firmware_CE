// Copyright (C) 2024, Mark Qvist

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "Interfaces.h"
#include "ROM.h"

#ifndef BOARDS_H
  #define BOARDS_H

  #define PLATFORM_ESP32      0x80
  #define PLATFORM_NRF52      0x70

  #define MCU_ESP32           0x81
  #define MCU_NRF52           0x71

  // Boards
  #define BOARD_RNODE         0x31
  #define BOARD_HMBRW         0x32
  #define BOARD_TBEAM         0x33
  #define BOARD_HUZZAH32      0x34
  #define BOARD_GENERIC_ESP32 0x35
  #define BOARD_LORA32_V2_0   0x36
  #define BOARD_LORA32_V2_1   0x37
  #define BOARD_LORA32_V1_0   0x39
  #define BOARD_HELTEC32_V2   0x38
  #define BOARD_HELTEC32_V3   0x3A
  #define BOARD_RNODE_NG_20   0x40
  #define BOARD_RNODE_NG_21   0x41
  #define BOARD_T3S3   0x42
  #define BOARD_TECHO         0x43
  #define BOARD_GENERIC_NRF52 0x50
  #define BOARD_RAK4631       0x51

  // Displays
  #define OLED 0x01
  #define EINK_BW 0x02
  #define EINK_3C 0x03

  #if defined(ESP32)
    #define PLATFORM PLATFORM_ESP32
    #define MCU_VARIANT MCU_ESP32
  #elif defined(NRF52840_XXAA) || defined(_VARIANT_PCA10056_)
    #include <variant.h>
    #define PLATFORM PLATFORM_NRF52
    #define MCU_VARIANT MCU_NRF52
  #else
      #error "The firmware cannot be compiled for the selected MCU variant"
  #endif

  #define HAS_DISPLAY false
  #define HAS_BLUETOOTH false
  #define HAS_BLE false
  #define HAS_TCXO false
  #define HAS_PMU false
  #define HAS_NP false
  #define HAS_EEPROM false
  #define HAS_INPUT false
  #define HAS_SLEEP false
  #define PIN_DISP_SLEEP -1
  #define VALIDATE_FIRMWARE true

  #if defined(ENABLE_TCXO)
      #define HAS_TCXO true
  #endif

  #if MCU_VARIANT == MCU_ESP32

    // Board models for ESP32 based builds are
    // defined by the build target in the makefile.
    // If you are not using make to compile this
    // firmware, you can manually define model here.
    //
    // #define BOARD_MODEL BOARD_GENERIC_ESP32
    #define CONFIG_UART_BUFFER_SIZE 6144
    #define CONFIG_QUEUE_0_SIZE 6144
    #define CONFIG_QUEUE_MAX_LENGTH 200

    #define EEPROM_SIZE 1024
    #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED

    #define GPS_BAUD_RATE 9600
    #define PIN_GPS_TX 12
    #define PIN_GPS_RX 34

    #if BOARD_MODEL == BOARD_GENERIC_ESP32
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 14;
      const int pin_led_tx = 32;
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
              4, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              39, // pin_dio
              36, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_TBEAM
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_PMU true
      #define HAS_BLUETOOTH true
      #define HAS_BLE true
      #define HAS_CONSOLE true
      #define HAS_SD false
      #define HAS_EEPROM true
      #define I2C_SDA 21
      #define I2C_SCL 22
      #define PMU_IRQ 35
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 2;
      const int pin_led_tx = 4;

      const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              true, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              32, // pin_busy
              33, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_HUZZAH32
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_led_rx = 14;
      const int pin_led_tx = 32;

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
                  // SX1262
          {
               4, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              39, // pin_dio
              36, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_LORA32_V1_0
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_BLE true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_cs = 18;
      const int pin_reset = 14;
      const int pin_dio = 26;
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 25;
        const int pin_led_tx = 2;
      #else
        const int pin_led_rx = 2;
        const int pin_led_tx = 2;
      #endif

      const uint8_t interfaces[INTERFACE_COUNT] = {SX1276};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1276
          {
              true, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1276
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              14, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_LORA32_V2_0
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_BLE true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_cs = 18;
      const int pin_reset = 12;
      const int pin_dio = 26;
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 2;
        const int pin_led_tx = 0;
      #else
        const int pin_led_rx = 22;
        const int pin_led_tx = 22;
      #endif


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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              14, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_LORA32_V2_1
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_BLE true
      #define HAS_PMU true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1

      const uint8_t interfaces[INTERFACE_COUNT] = {SX127X};
      #if HAS_TCXO == true
        const bool interface_cfg[INTERFACE_COUNT][3] = { 
                        // SX127X
              {
                  true, // DEFAULT_SPI
                  true, // HAS_TCXO
                  false  // DIO2_AS_RF_SWITCH
              }, 
        };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX127X
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              33  // pin_tcxo_enable
          }
      };
      #endif
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 15;
        const int pin_led_tx = 4;
      #else
        const int pin_led_rx = 25;
        const int pin_led_tx = 25;
      #endif

      #if HAS_TCXO == false
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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };
      #endif

    #elif BOARD_MODEL == BOARD_HELTEC32_V2
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 36;
        const int pin_led_tx = 37;
      #else
        const int pin_led_rx = 25;
        const int pin_led_tx = 25;
      #endif

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
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              14, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_HELTEC32_V3
      #define IS_ESP32S3 true
      #define HAS_DISPLAY true
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_CONSOLE false
      #define HAS_EEPROM true
      #define HAS_INPUT true
      #define HAS_SLEEP true
      #define PIN_WAKEUP GPIO_NUM_0
      #define WAKEUP_LEVEL 0
      #define INTERFACE_COUNT 1

      const int pin_btn_usr1 = 0;

      #if defined(EXTERNAL_LEDS)
        const int pin_led_rx = 13;
        const int pin_led_tx = 14;
      #else
        const int pin_led_rx = 35;
        const int pin_led_tx = 35;
      #endif

      const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              true, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const int8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
              8, // pin_ss
              9, // pin_sclk
              10, // pin_mosi
              11, // pin_miso
              13, // pin_busy
              14, // pin_dio
              12, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_RNODE_NG_20
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_NP true
      #define HAS_CONSOLE true
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_cs = 18;
      const int pin_reset = 12;
      const int pin_dio = 26;
      const int pin_np = 4;
      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 2;
          const int pin_led_tx = 0;
        #else
          const int pin_led_rx = 22;
          const int pin_led_tx = 22;
        #endif
      #endif


      const uint8_t interfaces[INTERFACE_COUNT] = {SX1276};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1276
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      const uint8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1276
          {
              8, // pin_ss
              9, // pin_sclk
              10, // pin_mosi
              11, // pin_miso
              13, // pin_busy
              14, // pin_dio
              12, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_RNODE_NG_21
      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_BLUETOOTH true
      #define HAS_CONSOLE true
      #define HAS_PMU true
      #define HAS_NP true
      #define HAS_SD false
      #define HAS_EEPROM true
      #define INTERFACE_COUNT 1
      const int pin_np = 12;
      const int pin_dac = 25;
      const int pin_adc = 34;
      const int SD_MISO = 2;
      const int SD_MOSI = 15;
      const int SD_CLK = 14;
      const int SD_CS = 13;
      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 12;
          const int pin_led_tx = 4;
        #else
          const int pin_led_rx = 25;
          const int pin_led_tx = 25;
        #endif
      #endif


      const uint8_t interfaces[INTERFACE_COUNT] = {SX127X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX127X
          {
              true, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      const uint8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX127X
          {
              18, // pin_ss
              -1, // pin_sclk
              -1, // pin_mosi
              -1, // pin_miso
              -1, // pin_busy
              26, // pin_dio
              23, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };

    #elif BOARD_MODEL == BOARD_T3S3
      #define IS_ESP32S3 true

      #define HAS_DISPLAY true
      #define DISPLAY OLED
      #define HAS_CONSOLE false
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_PMU true
      #define HAS_NP false
      #define HAS_SD false
      #define HAS_EEPROM true

      #define HAS_INPUT true
      #define HAS_SLEEP true
      #define PIN_WAKEUP GPIO_NUM_0
      #define WAKEUP_LEVEL 0
      #define INTERFACE_COUNT 1
      // #define PIN_DISP_SLEEP 21
      // #define DISP_SLEEP_LEVEL HIGH
      const int pin_btn_usr1 = 0;
      
      const int pin_np = 38;
      const int pin_dac = 25;
      const int pin_adc = 1;

      const int SD_MISO = 2;
      const int SD_MOSI = 11;
      const int SD_CLK = 14;
      const int SD_CS = 13;
      #if HAS_NP == false
        #if defined(EXTERNAL_LEDS)
          const int pin_led_rx = 37;
          const int pin_led_tx = 37;
        #else
          const int pin_led_rx = 37;
          const int pin_led_tx = 37;
        #endif
      #endif

      #if BOARD_VARIANT == MODEL_A1
      const uint8_t interfaces[INTERFACE_COUNT] = {SX1262};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
      };
      #elif BOARD_VARIANT == MODEL_A5 // SX1280 with PA 
      const uint8_t interfaces[INTERFACE_COUNT] = {SX1280};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1280
          {
              false, // DEFAULT_SPI
              false, // HAS_TCXO
              false  // DIO2_AS_RF_SWITCH
          }, 
      };
      #endif
      const uint8_t interface_pins[INTERFACE_COUNT][10] = { 
                  // SX1262
          {
               7, // pin_ss
               5, // pin_sclk
               6, // pin_mosi
               3, // pin_miso
              34, // pin_busy
              33, // pin_dio
               8, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              -1  // pin_tcxo_enable
          }
      };
    #else
      #error An unsupported ESP32 board was selected. Cannot compile RNode firmware.
    #endif
  
  #elif MCU_VARIANT == MCU_NRF52
     #if BOARD_MODEL == BOARD_TECHO
      #define VALIDATE_FIRMWARE false
      #define HAS_INPUT true
      //#define GPS_BAUD_RATE 115200
      //#define PIN_GPS_TX 41
      //#define PIN_GPS_RX 40
      #define EEPROM_SIZE 296
      #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
      //#define HAS_EEPROM true
      //#define HAS_SD true
      #define HAS_DISPLAY true
      #define DISPLAY EINK_BW
      #define DISPLAY_MODEL GxEPD2_154_D67
      //#define HAS_CONSOLE true
      //#define HAS_TXCO true
      //#define HAS_BLE true
      //#define HAS_PMU true
      #define CONFIG_UART_BUFFER_SIZE 40000
      #define CONFIG_QUEUE_0_SIZE 6144
      #define CONFIG_QUEUE_MAX_LENGTH 200
      //#define BLE_MANUFACTURER "LilyGO"
      //#define BLE_MODEL "T-Echo"
      #define INTERFACE_COUNT 1
      //#define I2C_SDA 26
      //#define I2C_SCL 27
      #define CONFIG_QUEUE_1_SIZE 40000
      // first interface in list is the primary
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X};
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
              24, // pin_ss 
              19, // pin_sclk
              22, // pin_mosi
              23, // pin_miso
              17, // pin_busy
              20, // pin_dio
              25, // pin_reset
              -1, // pin_txen
              -1, // pin_rxen
              21  // pin_tcxo_enable
          }
      };

      const int pin_disp_cs = 30;
      const int pin_disp_dc = 28;
      const int pin_disp_reset = 2;
      const int pin_disp_busy = 3;
      const int pin_disp_en = 43;
      const int pin_disp_sck = 31;
      const int pin_disp_mosi = 29;
      const int pin_disp_miso = -1;

      #define HAS_BACKLIGHT true
      const int pin_btn_usr1 = 42;
      const int pin_backlight = 43;

      const int pin_led_rx = LED_BLUE;
      const int pin_led_tx = LED_RED;
    #elif BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_FREENODE
      #define HAS_EEPROM false
      #define HAS_DISPLAY true
      #define DISPLAY EINK_BW
      #define HAS_BLUETOOTH false
      #define HAS_BLE true
      #define HAS_CONSOLE false
      #define HAS_PMU true
      #define HAS_NP false
      #define HAS_SD false
      #define CONFIG_UART_BUFFER_SIZE 40000
      #define CONFIG_QUEUE_0_SIZE 6144
      #define CONFIG_QUEUE_MAX_LENGTH 200
      #define EEPROM_SIZE 296
      #define EEPROM_OFFSET EEPROM_SIZE-EEPROM_RESERVED
      #define BLE_MANUFACTURER "RAK Wireless"
      #define BLE_MODEL "RAK4640"

      #if BOARD_VARIANT == MODEL_11 || BOARD_VARIANT == MODEL_12
      #define INTERFACE_COUNT 1

      // first interface in list is the primary
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X};
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
      #elif BOARD_VARIANT == MODEL_13 || BOARD_VARIANT == MODEL_14 || BOARD_VARIANT == MODEL_21
      #define INTERFACE_COUNT 2

      #define CONFIG_QUEUE_1_SIZE 40000

      // first interface in list is the primary
      const uint8_t interfaces[INTERFACE_COUNT] = {SX126X, SX128X};
      const bool interface_cfg[INTERFACE_COUNT][3] = { 
                    // SX1262
          {
              false, // DEFAULT_SPI
              true, // HAS_TCXO
              true  // DIO2_AS_RF_SWITCH
          }, 
                    // SX1280
          {
              true, // DEFAULT_SPI
              false,// HAS_TCXO
              false // DIO2_AS_RF_SWITCH
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
          },
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
      #endif


      const int pin_disp_cs = SS;
      const int pin_disp_dc = WB_IO1;
      const int pin_disp_reset = -1;
      const int pin_disp_busy = WB_IO4;
      const int pin_disp_en = WB_IO2;

      const int pin_led_rx = LED_BLUE;
      const int pin_led_tx = LED_GREEN;

    #else
      #error An unsupported nRF board was selected. Cannot compile RNode firmware.
    #endif

  #endif
#endif
