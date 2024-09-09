# Building
## Prerequisites
The build system of this repository is based on GNU Make. The `Makefile` is in the base of the repository. Please ensure you have `arduino-cli`, `python3` and `make` installed before proceeding.

Firstly, figure out which MCU platform your supported board is based on. The table below can help you.

| Board name | Link | Transceiver | MCU | Description | 
| :--- | :---: | :---: | :---: | :---: |
| Handheld v2.x RNodes | [Buy here](https://unsigned.io/shop/product/handheld-rnode) | SX1276 | ESP32 |
| RAK4631 | [Buy here](https://store.rakwireless.com/products/rak4631-lpwan-node?m=5&h=wisblock-core) | SX1262 | nRF52 |
| LilyGO LoRa32 v1.0 | [Buy here](https://www.lilygo.cc/products/lora32-v1-0) | SX1276/8 | ESP32 |
| LilyGO T-BEAM v1.1 | [Buy here](https://www.lilygo.cc/products/t-beam-v1-1-esp32-lora-module) | SX1276/8 | ESP32 |
| LilyGO LoRa32 v2.0 | No link | SX1276/8 | ESP32 | Discontinued? |
| LilyGO LoRa32 v2.1 |  [Buy here](https://www.lilygo.cc/products/lora3) | SX1276/8 | ESP32 | With and without TCXO |
| Heltec LoRa32 v2 | No link | SX1276/8 | ESP32 | Discontinued? |
| Heltec LoRa32 v3 | [Buy here](https://heltec.org/project/wifi-lora-32-v3/) | SX1262 | ESP32 | 
| Homebrew ESP32 boards | | Any supported | ESP32 | This can be any board with an Adafruit Feather (or generic) ESP32 chip |

### ESP32
If your board is ESP32-based, please run `make prep-esp32` to install the required BSP and libraries for that target.

### nRF52
If your board is nRF52-based, please run `make prep-nrf` to install the required BSP and libraries for that target.

## Compiling
Next, you need to find the name of the target for your board. Please reference the table below to do so:
| Board name | Target | 
| :--- | :---: |
| Handheld v2.x RNodes | `rnode_ng_20` |
| RAK4631 | `rak4631` |
| LilyGO T-BEAM v1.1 | `tbeam` |
| LilyGO T-BEAM v1.1 (SX1262) | `tbeam_sx126x` |
| LilyGO LoRa32 v1.0 | `lora32_v10` |
| LilyGO LoRa32 v2.0 | `lora32_v20` |
| LilyGO LoRa32 v2.1 | `lora32_v21` |
| Heltec LoRa32 v2 | `heltec32_v2` |
| Heltec LoRa32 v3 | `heltec32_v3` | 
| Homebrew ESP32 boards | `genericesp32` |

After you've ascertained the target for the board simply run the following to compile for the board:

`make firmware-[target]`

Ensure you replace [target] with the target you selected. For example:

`make firmware-rak4631`

## Flashing
To flash the chosen board (if you have one), simply connect it to your computer over USB, and then run the following:

`make upload-[target]`

Ensure you replace [target] with the target you selected. For example:

`make upload-rak4631`

If you are flashing a custom board, you will need to generate a signing key in rnodeconf prior to flashing if you do not already have one by running:

`rnodeconf -k`

After flashing a custom board, you will also need to provision the EEPROM before use:

`rnodeconf /dev/ttyACM0 -r --platform ESP32 --model a9 --product f0 --hwrev 3`

- platform must either be AVR, ESP32 or NRF52
- hwrev is required (any integer between 1 and 255)
- model should be something from the list below without the leading `0x` and in lowercase (example `e8`):
```
0x11: [430000000, 510000000, 22, "430 - 510 MHz", "rnode_firmware_rak4631.zip", "SX1262"],
0x12: [779000000, 928000000, 22, "779 - 928 MHz", "rnode_firmware_rak4631.zip", "SX1262"],
0xA4: [410000000, 525000000, 14, "410 - 525 MHz", "rnode_firmware.hex", "SX1278"],
0xA9: [820000000, 1020000000, 17, "820 - 1020 MHz", "rnode_firmware.hex", "SX1276"],
0xA1: [410000000, 525000000, 22, "410 - 525 MHz", "rnode_firmware_t3s3.zip", "SX1268"],
0xA6: [820000000, 1020000000, 22, "820 - 960 MHz", "rnode_firmware_t3s3.zip", "SX1262"],
0xA2: [410000000, 525000000, 17, "410 - 525 MHz", "rnode_firmware_ng21.zip", "SX1278"],
0xA7: [820000000, 1020000000, 17, "820 - 1020 MHz", "rnode_firmware_ng21.zip", "SX1276"],
0xA3: [410000000, 525000000, 17, "410 - 525 MHz", "rnode_firmware_ng20.zip", "SX1278"],
0xA8: [820000000, 1020000000, 17, "820 - 1020 MHz", "rnode_firmware_ng20.zip", "SX1276"],
0xB3: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_lora32v20.zip", "SX1278"],
0xB8: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_lora32v20.zip", "SX1276"],
0xB4: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_lora32v21.zip", "SX1278"],
0xB9: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_lora32v21.zip", "SX1276"],
0x04: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_lora32v21_tcxo.zip", "SX1278"],
0x09: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_lora32v21_tcxo.zip", "SX1276"],
0xBA: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_lora32v10.zip", "SX1278"],
0xBB: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_lora32v10.zip", "SX1276"],
0xC4: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_heltec32v2.zip", "SX1278"],
0xC9: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_heltec32v2.zip", "SX1276"],
0xC5: [470000000, 510000000, 21, "470 - 510 MHz", "rnode_firmware_heltec32v3.zip", "SX1262"],
0xCA: [863000000, 928000000, 21, "863 - 928 MHz", "rnode_firmware_heltec32v3.zip", "SX1262"],
0xE4: [420000000, 520000000, 17, "420 - 520 MHz", "rnode_firmware_tbeam.zip", "SX1278"],
0xE9: [850000000, 950000000, 17, "850 - 950 MHz", "rnode_firmware_tbeam.zip", "SX1276"],
0xE3: [420000000, 520000000, 22, "420 - 520 MHz", "rnode_firmware_tbeam_sx1262.zip", "SX1268"],
0xE8: [850000000, 950000000, 22, "850 - 950 MHz", "rnode_firmware_tbeam_sx1262.zip", "SX1262"],
0xFE: [100000000, 1100000000, 17, "(Band capabilities unknown)", None, "Unknown"],
0xFF: [100000000, 1100000000, 14, "(Band capabilities unknown)", None, "Unknown"],
```
- product should be a code from the following list below without the leading `0x` and in lowercase (example `f0`):
```
PRODUCT_RAK4631 = 0x10
PRODUCT_RNODE  = 0x03
PRODUCT_T32_10 = 0xB2
PRODUCT_T32_20 = 0xB0
PRODUCT_T32_21 = 0xB1
PRODUCT_H32_V2 = 0xC0
PRODUCT_H32_V3 = 0xC1
PRODUCT_TBEAM  = 0xE0
PRODUCT_HMBRW  = 0xF0
```

**Please note**, you must re-compile the firmware each time you make changes **before** you flash it, else you will just be flashing the previous version of the firmware without the new changes!

These commands can also be run as a one liner. For example:

`make firmware-[target] && make upload-[target]`

This is especially helpful when making continuous changes to the firmware and testing them out.
