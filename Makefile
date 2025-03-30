# Copyright (C) 2024, Mark Qvist

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

# Version 2.0.17 of the Arduino ESP core is based on ESP-IDF v4.4.7
ARDUINO_ESP_CORE_VER = 2.0.17

V ?= 0
VFLAG =
ifeq "$(V)" "1"
VFLAG =-v
endif

COMMON_BUILD_FLAGS =  $(VFLAG) -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152"
COMMON_ESP_UPLOAD_FLAGS = $(VFLAG) --chip esp32 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000

all: release

clean:
	-rm -rf ./build
	-rm -f ./Release/rnode_firmware*

prep: prep-esp32 prep-nrf

prep-index:
	arduino-cli core update-index --config-file arduino-cli.yaml

prep-esp32:
	arduino-cli core install esp32:esp32@$(ARDUINO_ESP_CORE_VER) --config-file arduino-cli.yaml
	arduino-cli lib install "Adafruit SSD1306"
	arduino-cli lib install "Adafruit SH110X"
	arduino-cli lib install "Adafruit ST7735 and ST7789 Library"
	arduino-cli lib install "Adafruit NeoPixel"
	arduino-cli lib install "XPowersLib"
	arduino-cli lib install "Crypto"
	arduino-cli lib install "Adafruit NeoPixel"
	pip install pyserial rns --upgrade --user --break-system-packages # This looks scary, but it's actually just telling pip to install packages as a user instead of trying to install them systemwide, which bypasses the "externally managed environment" error.

prep-nrf:
	arduino-cli core install adafruit:nrf52 --config-file arduino-cli.yaml
	arduino-cli core install rakwireless:nrf52 --config-file arduino-cli.yaml
	arduino-cli core install Heltec_nRF52:Heltec_nRF52 --config-file arduino-cli.yaml
	arduino-cli lib install "Crypto"
	arduino-cli lib install "Adafruit GFX Library"
	arduino-cli lib install "GxEPD2"
	arduino-cli lib install "TinyGPSPlus"
	arduino-cli config set library.enable_unsafe_install true
	arduino-cli lib install --git-url https://github.com/liamcottle/esp8266-oled-ssd1306#e16cee124fe26490cb14880c679321ad8ac89c95
	pip install pyserial rns --upgrade --user --break-system-packages # This looks scary, but it's actually just telling pip to install packages as a user instead of trying to install them systemwide, which bypasses the "externally managed environment" error.
	pip install adafruit-nrfutil --upgrade --user --break-system-packages # This looks scary, but it's actually just telling pip to install packages as a user instead of trying to install them systemwide, which bypasses the "externally managed environment" error.

console-site:
	make -C Console clean site

spiffs: console-site spiffs-image

spiffs-image:
	python3 Release/esptool/spiffsgen.py 1966080 ./Console/build Release/console_image.bin

upload-spiffs:
	@echo Deploying SPIFFS image...
	python ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) $(COMMON_UPLOAD_FLAGS) ./Release/console_image.bin

firmware: $(shell grep ^firmware- Makefile | cut -d: -f1)

check_bt_buffers:
	@./esp32_btbufs.py ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/libraries/BluetoothSerial/src/BluetoothSerial.cpp

firmware-tbeam: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:t-beam $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x33\" \"-DBOARD_VARIANT=0xE4\""

firmware-tbeam_sx1262: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:t-beam $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x33\" \"-DBOARD_VARIANT=0xE8\""

firmware-techo:
	arduino-cli compile --fqbn adafruit:nrf52:pca10056 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x44\""

firmware-t3s3:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x42\" \"-DBOARD_VARIANT=0xAB\""

firmware-t3s3_sx126x:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x42\" \"-DBOARD_VARIANT=0xA1\""

firmware-t3s3_sx1280_pa:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x42\" \"-DBOARD_VARIANT=0xAC\""

firmware-e22_esp32:
	arduino-cli compile --fqbn esp32:esp32:esp32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x45\" \"-DEXTERNAL_LEDS=true\""

firmware-lora32_v10: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x39\""

firmware-lora32_v10_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x39\" \"-DEXTERNAL_LEDS=true\""

firmware-lora32_v20: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x36\" \"-DEXTERNAL_LEDS=true\""

firmware-lora32_v21: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x37\""

firmware-lora32_v21_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x37\" \"-DEXTERNAL_LEDS=true\""

firmware-lora32_v21_tcxo: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x37\" \"-DENABLE_TCXO=true\""

firmware-heltec32_v2: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x38\""

firmware-heltec32_v2_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x38\" \"-DEXTERNAL_LEDS=true\""

firmware-heltec32_v3:
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V3 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3A\""

firmware-heltec_w_paper:
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V3 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3E\""

firmware-xiao_esp32s3:
	arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3E\""

firmware-rnode_ng_20: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x40\""

firmware-rnode_ng_21: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x41\""

firmware-featheresp32: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:featheresp32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x34\""

firmware-genericesp32: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:esp32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x35\""

firmware-rak4631:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x51\" \"-DBOARD_VARIANT=0x12\""

firmware-rak4631_sx1280:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x51\" \"-DBOARD_VARIANT=0x14\""

firmware-opencom-xl:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x52\" \"-DBOARD_VARIANT=0x21\""

firmware-heltec_t114:
	python3 ./Scripts/heltec_nrf52_bsp_prep.py
	arduino-cli compile --log --fqbn Heltec_nRF52:Heltec_nRF52:HT-n5262 -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3C\""

firmware-heltec_t114_gps:
	arduino-cli compile --log --fqbn Heltec_nRF52:Heltec_nRF52:HT-n5262 -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3C\" \"-DBOARD_VARIANT=0xCB\""

upload-tbeam:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:t-beam
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-tbeam_sx1262:
	arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:t-beam
	@sleep 1
	rnodeconf /dev/ttyACM0 --firmware-hash $$(./partition_hashes ./build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.bin)
	#@sleep 3
	#python ./Release/esptool/esptool.py --chip esp32 --port /dev/ttyACM0 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin

upload-lora32_v10:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:ttgo-lora32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-lora32_v20:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:ttgo-lora32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-lora32_v21:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:ttgo-lora32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-heltec32_v2:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:heltec_wifi_lora_32_V2
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-heltec_w_paper upload-heltec32_v3:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:heltec_wifi_lora_32_V3
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) --chip esp32-s3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin

upload-xiao_esp32s3:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:XIAO_ESP32S3
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.XIAO_ESP32S3/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) --chip esp32-s3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 8MB 0x210000 ./Release/console_image.bin


upload-tdeck:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:esp32s3
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) --chip esp32-s3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin

upload-tbeam_supreme:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:esp32s3
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) --chip esp32-s3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin

upload-rnode_ng_20:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:ttgo-lora32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-rnode_ng_21:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:ttgo-lora32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) $(COMMON_ESP_UPLOAD_FLAGS) ./Release/console_image.bin

upload-t3s3:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn esp32:esp32:esp32s3
	@sleep 1
	python ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyACM0) --chip esp32-s3 --baud 921600 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB 0x210000 ./Release/console_image.bin
	@sleep 3
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin)

upload-featheresp32:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:featheresp32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.featheresp32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) $(COMMON_UPLOAD_FLAGS) ./Release/console_image.bin

upload-opencom-xl upload-rak4631:
	arduino-cli upload -p $(or $(port), /dev/ttyACM0) --fqbn rakwireless:nrf52:WisCoreRAK4631Board
	unzip -o build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.zip -d build/rakwireless.nrf52.WisCoreRAK4631Board
	rnodeconf $(or $(port), /dev/ttyACM0) --firmware-hash $$(sha256sum ./build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.bin | grep -o '^\S*')

upload-e22_esp32:
	arduino-cli upload -p $(or $(port), /dev/ttyUSB0) --fqbn esp32:esp32:esp32
	@sleep 1
	rnodeconf $(or $(port), /dev/ttyUSB0) --firmware-hash $$(./partition_hashes ./build/esp32.esp32.esp32/RNode_Firmware_CE.ino.bin)
	@sleep 3
	python3 ./Release/esptool/esptool.py --port $(or $(port), /dev/ttyUSB0) $(COMMON_ESP_UPLOAD_FLAGS)  ./Release/console_image.bin

upload-heltec_t114:
	arduino-cli upload -p /dev/ttyACM0 --fqbn Heltec_nRF52:Heltec_nRF52:HT-n5262
	@sleep 1
	rnodeconf /dev/ttyACM0 --firmware-hash $$(./partition_hashes from_device /dev/ttyACM0)

upload-techo:
	arduino-cli upload -p /dev/ttyACM0 --fqbn adafruit:nrf52:pca10056
	@sleep 6
	rnodeconf /dev/ttyACM0 --firmware-hash $$(./partition_hashes from_device /dev/ttyACM0)

release:  console-site spiffs-image $(shell grep ^release- Makefile | cut -d: -f1)

release-hashes:
	python3 ./release_hashes.py > ./Release/release.json

release-tbeam: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:t-beam $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x33\" \"-DBOARD_VARIANT=0xE4\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_tbeam.boot_app0
	cp build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.bin build/rnode_firmware_tbeam.bin
	cp build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_tbeam.bootloader
	cp build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_tbeam.partitions
	zip --junk-paths ./Release/rnode_firmware_tbeam.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_tbeam.boot_app0 build/rnode_firmware_tbeam.bin build/rnode_firmware_tbeam.bootloader build/rnode_firmware_tbeam.partitions
	rm -r build

release-tbeam_sx1262: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:t-beam $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x33\" \"-DBOARD_VARIANT=0xE8\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_tbeam_sx1262.boot_app0
	cp build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.bin build/rnode_firmware_tbeam_sx1262.bin
	cp build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_tbeam_sx1262.bootloader
	cp build/esp32.esp32.t-beam/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_tbeam_sx1262.partitions
	zip --junk-paths ./Release/rnode_firmware_tbeam_sx1262.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_tbeam_sx1262.boot_app0 build/rnode_firmware_tbeam_sx1262.bin build/rnode_firmware_tbeam_sx1262.bootloader build/rnode_firmware_tbeam_sx1262.partitions
	rm -r build

release-lora32_v10: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x39\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v10.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v10.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v10.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v10.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v10.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v10.boot_app0 build/rnode_firmware_lora32v10.bin build/rnode_firmware_lora32v10.bootloader build/rnode_firmware_lora32v10.partitions
	rm -r build

release-lora32_v20: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x36\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v20.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v20.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v20.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v20.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v20.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v20.boot_app0 build/rnode_firmware_lora32v20.bin build/rnode_firmware_lora32v20.bootloader build/rnode_firmware_lora32v20.partitions
	rm -r build

release-lora32_v21: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x37\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v21.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v21.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v21.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v21.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v21.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v21.boot_app0 build/rnode_firmware_lora32v21.bin build/rnode_firmware_lora32v21.bootloader build/rnode_firmware_lora32v21.partitions
	rm -r build

release-lora32_v10_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x39\" \"-DEXTERNAL_LEDS=true\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v10.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v10.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v10.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v10.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v10.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v10.boot_app0 build/rnode_firmware_lora32v10.bin build/rnode_firmware_lora32v10.bootloader build/rnode_firmware_lora32v10.partitions
	rm -r build

release-lora32_v20_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x36\" \"-DEXTERNAL_LEDS=true\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v20.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v20.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v20.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v20.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v20_extled.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v20.boot_app0 build/rnode_firmware_lora32v20.bin build/rnode_firmware_lora32v20.bootloader build/rnode_firmware_lora32v20.partitions
	rm -r build

release-lora32_v21_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x37\" \"-DEXTERNAL_LEDS=true\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v21.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v21.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v21.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v21.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v21_extled.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v21.boot_app0 build/rnode_firmware_lora32v21.bin build/rnode_firmware_lora32v21.bootloader build/rnode_firmware_lora32v21.partitions
	rm -r build

release-lora32_v21_tcxo: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x37\" \"-DENABLE_TCXO=true\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_lora32v21_tcxo.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_lora32v21_tcxo.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_lora32v21_tcxo.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_lora32v21_tcxo.partitions
	zip --junk-paths ./Release/rnode_firmware_lora32v21_tcxo.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_lora32v21_tcxo.boot_app0 build/rnode_firmware_lora32v21_tcxo.bin build/rnode_firmware_lora32v21_tcxo.bootloader build/rnode_firmware_lora32v21_tcxo.partitions
	rm -r build

release-heltec32_v2: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x38\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_heltec32v2.boot_app0
	cp build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.bin build/rnode_firmware_heltec32v2.bin
	cp build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_heltec32v2.bootloader
	cp build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_heltec32v2.partitions

release-heltec32_v3:
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V3 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3A\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_heltec32v3.boot_app0
	cp build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.bin build/rnode_firmware_heltec32v3.bin
	cp build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_heltec32v3.bootloader
	cp build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_heltec32v3.partitions
	zip --junk-paths ./Release/rnode_firmware_heltec32v3.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_heltec32v3.boot_app0 build/rnode_firmware_heltec32v3.bin build/rnode_firmware_heltec32v3.bootloader build/rnode_firmware_heltec32v3.partitions
	rm -r build

release-heltec_w_paper:
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V3 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3E\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_heltecwpaper.boot_app0
	cp build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.bin build/rnode_firmware_heltecwpaper.bin
	cp build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_heltecwpaper.bootloader
	cp build/esp32.esp32.heltec_wifi_lora_32_V3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_heltecwpaper.partitions
	zip --junk-paths ./Release/rnode_firmware_heltecwpaper.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_heltecwpaper.boot_app0 build/rnode_firmware_heltecwpaper.bin build/rnode_firmware_heltecwpaper.bootloader build/rnode_firmware_heltecwpaper.partitions
	rm -r build

release-xiao_esp32s3:
	arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32S3 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3E\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_xiao_esp32s3.boot_app0
	cp build/esp32.esp32.XIAO_ESP32S3/RNode_Firmware_CE.ino.bin build/rnode_firmware_xiao_esp32s3.bin
	cp build/esp32.esp32.XIAO_ESP32S3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_xiao_esp32s3.bootloader
	cp build/esp32.esp32.XIAO_ESP32S3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_xiao_esp32s3.partitions
	zip --junk-paths ./Release/rnode_firmware_xiao_esp32s3.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_xiao_esp32s3.boot_app0 build/rnode_firmware_xiao_esp32s3.bin build/rnode_firmware_xiao_esp32s3.bootloader build/rnode_firmware_xiao_esp32s3.partitions
	rm -r build

release-heltec32_v2_extled: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V2 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x38\" \"-DEXTERNAL_LEDS=true\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_heltec32v2.boot_app0
	cp build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.bin build/rnode_firmware_heltec32v2.bin
	cp build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_heltec32v2.bootloader
	cp build/esp32.esp32.heltec_wifi_lora_32_V2/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_heltec32v2.partitions
	zip --junk-paths ./Release/rnode_firmware_heltec32v2.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_heltec32v2.boot_app0 build/rnode_firmware_heltec32v2.bin build/rnode_firmware_heltec32v2.bootloader build/rnode_firmware_heltec32v2.partitions
	rm -r build

release-rnode_ng_20: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x40\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_ng20.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_ng20.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_ng20.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_ng20.partitions
	zip --junk-paths ./Release/rnode_firmware_ng20.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_ng20.boot_app0 build/rnode_firmware_ng20.bin build/rnode_firmware_ng20.bootloader build/rnode_firmware_ng20.partitions
	rm -r build

release-rnode_ng_21: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:ttgo-lora32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x41\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_ng21.boot_app0
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bin build/rnode_firmware_ng21.bin
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_ng21.bootloader
	cp build/esp32.esp32.ttgo-lora32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_ng21.partitions
	zip --junk-paths ./Release/rnode_firmware_ng21.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_ng21.boot_app0 build/rnode_firmware_ng21.bin build/rnode_firmware_ng21.bootloader build/rnode_firmware_ng21.partitions
	rm -r build

release-techo:
	arduino-cli compile --fqbn adafruit:nrf52:pca10056 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x44\""
	cp build/adafruit.nrf52.pca10056/RNode_Firmware_CE.ino.hex build/rnode_firmware_techo.hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application build/rnode_firmware_techo.hex Release/rnode_firmware_techo.zip
	rm -r build

release-t3s3:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x42\" \"-DBOARD_VARIANT=0xA1\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_t3s3_sx126x.boot_app0
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin build/rnode_firmware_t3s3_sx126x.bin
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_t3s3_sx126x.bootloader
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_t3s3_sx126x.partitions
	zip --junk-paths ./Release/rnode_firmware_t3s3_sx126x.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_t3s3_sx126x.boot_app0 build/rnode_firmware_t3s3_sx126x.bin build/rnode_firmware_t3s3_sx126x.bootloader build/rnode_firmware_t3s3_sx126x.partitions
	rm -r build

release-t3s3_sx127x:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x42\" \"-DBOARD_VARIANT=0xA5\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_t3s3_sx127x.boot_app0
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin build/rnode_firmware_t3s3_sx127x.bin
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_t3s3_sx127x.bootloader
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_t3s3_sx127x.partitions
	zip --junk-paths ./Release/rnode_firmware_t3s3_sx127x.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_t3s3_sx127x.boot_app0 build/rnode_firmware_t3s3_sx127x.bin build/rnode_firmware_t3s3_sx127x.bootloader build/rnode_firmware_t3s3_sx127x.partitions
	rm -r build

release-t3s3_sx1280_pa:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" -e --build-property "build.partitions=no_ota" --build-property "upload.maximum_size=2097152" --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x42\" \"-DBOARD_VARIANT=0xAC\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_t3s3_sx1280_pa.boot_app0
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin build/rnode_firmware_t3s3_sx1280_pa.bin
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_t3s3_sx1280_pa.bootloader
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_t3s3_sx1280_pa.partitions
	zip --junk-paths ./Release/rnode_firmware_t3s3_sx1280_pa.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_t3s3_sx1280_pa.boot_app0 build/rnode_firmware_t3s3_sx1280_pa.bin build/rnode_firmware_t3s3_sx1280_pa.bootloader build/rnode_firmware_t3s3_sx1280_pa.partitions
	rm -r build

release-e22_esp32:
	arduino-cli compile --fqbn esp32:esp32:esp32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x45\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_esp32_e22.boot_app0
	cp build/esp32.esp32.esp32/RNode_Firmware_CE.ino.bin build/rnode_firmware_esp32_e22.bin
	cp build/esp32.esp32.esp32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_esp32_e22.bootloader
	cp build/esp32.esp32.esp32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_esp32_e22.partitions
	zip --junk-paths ./Release/rnode_firmware_esp32_e22.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_esp32_e22.boot_app0 build/rnode_firmware_esp32_e22.bin build/rnode_firmware_esp32_e22.bootloader build/rnode_firmware_esp32_e22.partitions
	rm -r build

release-tdeck:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3B\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_tdeck.boot_app0
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin build/rnode_firmware_tdeck.bin
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_tdeck.bootloader
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_tdeck.partitions
	zip --junk-paths ./Release/rnode_firmware_tdeck.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_tdeck.boot_app0 build/rnode_firmware_tdeck.bin build/rnode_firmware_tdeck.bootloader build/rnode_firmware_tdeck.partitions
	rm -r build


release-tbeam_supreme:
	arduino-cli compile --fqbn "esp32:esp32:esp32s3:CDCOnBoot=cdc" $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3D\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_tbeam_supreme.boot_app0
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bin build/rnode_firmware_tbeam_supreme.bin
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_tbeam_supreme.bootloader
	cp build/esp32.esp32.esp32s3/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_tbeam_supreme.partitions
	zip --junk-paths ./Release/rnode_firmware_tbeam_supreme.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_tbeam_supreme.boot_app0 build/rnode_firmware_tbeam_supreme.bin build/rnode_firmware_tbeam_supreme.bootloader build/rnode_firmware_tbeam_supreme.partitions
	rm -r build


release-featheresp32: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:featheresp32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x34\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_featheresp32.boot_app0
	cp build/esp32.esp32.featheresp32/RNode_Firmware_CE.ino.bin build/rnode_firmware_featheresp32.bin
	cp build/esp32.esp32.featheresp32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_featheresp32.bootloader
	cp build/esp32.esp32.featheresp32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_featheresp32.partitions
	zip --junk-paths ./Release/rnode_firmware_featheresp32.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_featheresp32.boot_app0 build/rnode_firmware_featheresp32.bin build/rnode_firmware_featheresp32.bootloader build/rnode_firmware_featheresp32.partitions
	rm -r build

release-genericesp32: check_bt_buffers
	arduino-cli compile --fqbn esp32:esp32:esp32 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x35\""
	cp ~/.arduino15/packages/esp32/hardware/esp32/$(ARDUINO_ESP_CORE_VER)/tools/partitions/boot_app0.bin build/rnode_firmware_esp32_generic.boot_app0
	cp build/esp32.esp32.esp32/RNode_Firmware_CE.ino.bin build/rnode_firmware_esp32_generic.bin
	cp build/esp32.esp32.esp32/RNode_Firmware_CE.ino.bootloader.bin build/rnode_firmware_esp32_generic.bootloader
	cp build/esp32.esp32.esp32/RNode_Firmware_CE.ino.partitions.bin build/rnode_firmware_esp32_generic.partitions
	zip --junk-paths ./Release/rnode_firmware_esp32_generic.zip ./Release/esptool/esptool.py ./Release/console_image.bin build/rnode_firmware_esp32_generic.boot_app0 build/rnode_firmware_esp32_generic.bin build/rnode_firmware_esp32_generic.bootloader build/rnode_firmware_esp32_generic.partitions
	rm -r build

release-rak4631:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x51\" \"-DBOARD_VARIANT=0x12\""
	cp build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.hex build/rnode_firmware_rak4631.hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application build/rnode_firmware_rak4631.hex Release/rnode_firmware_rak4631.zip
	rm -r build

release-rak4631_sx1280:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x51\" \"-DBOARD_VARIANT=0x14\""
	cp build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.hex build/rnode_firmware_rak4631_sx1280.hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application build/rnode_firmware_rak4631_sx1280.hex Release/rnode_firmware_rak4631_sx1280.zip
	rm -r build

release-opencom-xl:
	arduino-cli compile --fqbn rakwireless:nrf52:WisCoreRAK4631Board $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x52\" \"-DBOARD_VARIANT=0x21\""
	cp build/rakwireless.nrf52.WisCoreRAK4631Board/RNode_Firmware_CE.ino.hex build/rnode_firmware_opencom_xl.hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application build/rnode_firmware_opencom_xl.hex Release/rnode_firmware_opencom_xl.zip
	rm -r build

release-heltec_t114:
	arduino-cli compile --fqbn Heltec_nRF52:Heltec_nRF52:HT-n5262 $(COMMON_BUILD_FLAGS) --build-property "compiler.cpp.extra_flags=\"-DBOARD_MODEL=0x3C\""
	cp build/Heltec_nRF52.Heltec_nRF52.HT-n5262/RNode_Firmware_CE.ino.hex build/rnode_firmware_heltec_t114.hex
	adafruit-nrfutil dfu genpkg --dev-type 0x0052 --application build/rnode_firmware_heltec_t114.hex Release/rnode_firmware_heltec_t114.zip
	rm -r build

