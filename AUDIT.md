# Audit Report

## Boards.h Constants

### BOARD_* Constants
- BOARD_RNODE (0x31)
- BOARD_RNODE_NG_20 (0x40)
- BOARD_RNODE_NG_21 (0x41)
- BOARD_T3S3 (0x42)
- BOARD_TBEAM (0x33)
- BOARD_TDECK (0x3B)
- BOARD_TBEAM_S_V1 (0x3D)
- BOARD_XIAO_S3 (0x3E)
- BOARD_LORA32_V1_0 (0x39)
- BOARD_LORA32_V2_0 (0x36)
- BOARD_LORA32_V2_1 (0x37)
- BOARD_HELTEC32_V2 (0x38)
- BOARD_HELTEC32_V3 (0x3A)
- BOARD_H_W_PAPER (0x3F)
- BOARD_RAK4631 (0x51)
- BOARD_OPENCOM_XL (0x52)
- BOARD_E22_ESP32 (0x45)
- BOARD_HELTEC_T114 (0x3C)
- BOARD_TECHO (0x44)
- BOARD_FAKETEC_V5 (0x55)
- BOARD_HMBRW (0x32)
- BOARD_HUZZAH32 (0x34)
- BOARD_GENERIC_ESP32 (0x35)
- BOARD_GENERIC_NRF52 (0x50)

### PRODUCT_* Constants
- PRODUCT_RNODE (0x03)
- PRODUCT_TBEAM (0xE0)
- PRODUCT_TDECK_V1 (0xD0)
- PRODUCT_TBEAM_S_V1 (0xEA)
- PRODUCT_XIAO_S3 (0xEB)
- PRODUCT_T32_10 (0xB2)
- PRODUCT_T32_20 (0xB0)
- PRODUCT_T32_21 (0xB1)
- PRODUCT_H32_V2 (0xC0)
- PRODUCT_H32_V3 (0xC1)
- PRODUCT_H_W_PAPER (0xC3)
- PRODUCT_RAK4631 (0x10)
- PRODUCT_OPENCOM_XL (0x20)
- PRODUCT_HELTEC_T114 (0xC2)
- PRODUCT_TECHO (0x15)
- PRODUCT_FAKETEC (0xFA)
- PRODUCT_HMBRW (0xF0)

### HAS_* Constants
- HAS_DISPLAY
- HAS_BLUETOOTH
- HAS_BLE
- HAS_TCXO
- HAS_PMU
- HAS_NP
- HAS_EEPROM
- HAS_INPUT
- HAS_SLEEP
- HAS_CONSOLE
- HAS_SD
- HAS_BACKLIGHT
- HAS_GPS
- HAS_BUSY

## rnodeconf.py Hardcoded Constants
*File not found in repository.*

## Functions Longer Than 100 Lines

### RNode_Firmware_CE.ino
- `setup` (approx. 270 lines)
- `serial_callback` (approx. 350 lines)
- `loop` (approx. 110 lines)

### Radio.cpp
- `sx126x::begin` (approx. 100 lines)
- `sx128x::setTxPower` (approx. 150 lines)

### Utilities.h
- `setTXPower` (approx. 100 lines)

### src/misc/MD5.cpp
- `body` (approx. 130 lines)

### Display.h
- `update_display` (approx. 100 lines)

## Toolchain Versions

### Arduino CLI
- Version: *To be determined manually*

### Board Packages
- Adafruit: https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
- RNode Firmware CE: https://liberatedsystems.co.uk/rnode-firmware-ce/esp-custom-package.json
- RAKwireless: https://raw.githubusercontent.com/RAKwireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless_index.json
- Heltec nRF52: https://github.com/HelTecAutomation/Heltec_nRF52/releases/download/1.7.0/package_heltec_nrf_index.json

### Python
- Version: 3.8+ (Required for f-strings in `esp32_btbufs.py`)
- Dependencies: `markdown>=3.3.0` (See `requirements.txt`)

## Build Verification Plan

1.  **Install Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

2.  **Setup Arduino CLI**:
    - Install Arduino CLI.
    - Configure board manager URLs from `arduino-cli.yaml`.
    - Install required cores (esp32, nrf52, etc.).

3.  **Compile for Each Board**:
    - Iterate through all `BOARD_*` constants defined in `Boards.h`.
    - For each board, run the compile command using `arduino-cli`.
    - Example: `arduino-cli compile --fqbn <board_fqbn> --build-property "build.extra_flags=-DBOARD_MODEL=<board_model>" .`
    - Verify exit code is 0 for all builds.

4.  **Report**:
    - Log success/failure for each board.
    - Investigate and fix any compilation errors.

## CI/CD Pipeline

A GitHub Actions workflow (`.github/workflows/ci.yml`) has been created to:
1.  **Build Firmware**: Compiles firmware for representative boards (T-Beam, RAK4631, LoRa32 v2.1).
2.  **Run C++ Tests**: Executes GoogleTest-based unit tests for MD5 and Utilities.
3.  **Run Python Tests**: Executes unit tests for the Python module.
4.  **Gate Merges**: Ensures all checks pass before merging.
