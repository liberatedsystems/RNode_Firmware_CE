// Copyright (C) 2023, Mark Qvist

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

#include "Radio.hpp"
#include "Config.h"

// Included for sorting
#include <algorithm>
#include <iterator>

#if HAS_EEPROM 
    #include <EEPROM.h>
#elif PLATFORM == PLATFORM_NRF52
    #include <Adafruit_LittleFS.h>
    #include <InternalFileSystem.h>
    using namespace Adafruit_LittleFS_Namespace;
    #define EEPROM_FILE "eeprom"
    bool file_exists = false;
    int written_bytes = 4;
    File file(InternalFS);
#endif
#include <stddef.h>

#include "ROM.h"
#include "Framing.h"
#include "MD5.h"

#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
uint8_t eeprom_read(uint32_t mapped_addr);
#endif

#if HAS_DISPLAY == true
  #include "Display.h"
#endif

#if HAS_BLUETOOTH == true || HAS_BLE == true
	void kiss_indicate_btpin();
  #include "Bluetooth.h"
#endif

#if HAS_PMU == true
  #include "Power.h"
#endif

#if HAS_INPUT == true
	#include "Input.h"
#endif

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#include "Device.h"
#endif
#if MCU_VARIANT == MCU_ESP32
  #if BOARD_MODEL == BOARD_HELTEC32_V3
    //https://github.com/espressif/esp-idf/issues/8855
    #include "hal/wdt_hal.h"
	#elif BOARD_MODEL == BOARD_T3S3
		#include "hal/wdt_hal.h"
  #else BOARD_MODEL != BOARD_T3S3
	  #include "soc/rtc_wdt.h"
	#endif
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

uint8_t boot_vector = 0x00;

#if MCU_VARIANT == MCU_ESP32
	// TODO: Get ESP32 boot flags
#elif MCU_VARIANT == MCU_NRF52
	// TODO: Get NRF52 boot flags
#endif

#if HAS_NP == true
	#include <Adafruit_NeoPixel.h>
	#define NUMPIXELS 1
	#define NP_M 0.15
	Adafruit_NeoPixel pixels(NUMPIXELS, pin_np, NEO_GRB + NEO_KHZ800);

	uint8_t npr = 0;
  uint8_t npg = 0;
  uint8_t npb = 0;
  bool pixels_started = false;
  void npset(uint8_t r, uint8_t g, uint8_t b) {
  	if (pixels_started != true) {
  		pixels.begin();
  		pixels_started = true;
  	}

  	if (r != npr || g != npg || b != npb) {
  		npr = r; npg = g; npb = b;
  		pixels.setPixelColor(0, pixels.Color(npr*NP_M, npg*NP_M, npb*NP_M));
  		pixels.show();
  	}
  }

  void boot_seq() {
  	uint8_t rs[] = { 0x00, 0x00, 0x00 };
  	uint8_t gs[] = { 0x10, 0x08, 0x00 };
  	uint8_t bs[] = { 0x00, 0x08, 0x10 };
  	for (int i = 0; i < 1*sizeof(rs); i++) {
	  	npset(rs[i%sizeof(rs)], gs[i%sizeof(gs)], bs[i%sizeof(bs)]);
	  	delay(33);
	  	npset(0x00, 0x00, 0x00);
	  	delay(66);
  	}
  }
#else
  void boot_seq() { }
#endif

#if MCU_VARIANT == MCU_ESP32
	#if HAS_NP == true
		void led_rx_on()  { npset(0, 0, 0xFF); }
		void led_rx_off() {	npset(0, 0, 0); }
		void led_tx_on()  { npset(0xFF, 0x50, 0x00); }
		void led_tx_off() { npset(0, 0, 0); }
	#elif BOARD_MODEL == BOARD_RNODE_NG_20
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_RNODE_NG_21
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_T3S3
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_TBEAM
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, LOW); }
		void led_tx_off() { digitalWrite(pin_led_tx, HIGH); }
	#elif BOARD_MODEL == BOARD_LORA32_V1_0
		#if defined(EXTERNAL_LEDS)
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#else
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#endif
	#elif BOARD_MODEL == BOARD_LORA32_V2_0
		#if defined(EXTERNAL_LEDS)
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#else
			void led_rx_on()  { digitalWrite(pin_led_rx, LOW); }
			void led_rx_off() {	digitalWrite(pin_led_rx, HIGH); }
			void led_tx_on()  { digitalWrite(pin_led_tx, LOW); }
			void led_tx_off() { digitalWrite(pin_led_tx, HIGH); }
		#endif
	#elif BOARD_MODEL == BOARD_HELTEC32_V2
		#if defined(EXTERNAL_LEDS)
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#else
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
		#endif
	#elif BOARD_MODEL == BOARD_HELTEC32_V3
			void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
			void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
			void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
			void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_LORA32_V2_1
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_HUZZAH32
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#elif BOARD_MODEL == BOARD_GENERIC_ESP32
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
	#endif
#elif MCU_VARIANT == MCU_NRF52
	#if BOARD_MODEL == BOARD_TECHO
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
    #elif BOARD_MODEL == BOARD_RAK4631
		void led_rx_on()  { digitalWrite(pin_led_rx, HIGH); }
		void led_rx_off() {	digitalWrite(pin_led_rx, LOW); }
		void led_tx_on()  { digitalWrite(pin_led_tx, HIGH); }
		void led_tx_off() { digitalWrite(pin_led_tx, LOW); }
    #endif
#endif

void hard_reset(void) {
	#if MCU_VARIANT == MCU_ESP32
		ESP.restart();
	#elif MCU_VARIANT == MCU_NRF52
        NVIC_SystemReset();
	#endif
}

// LED Indication: Error
void led_indicate_error(int cycles) {
	#if HAS_NP == true
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
			npset(0xFF, 0x00, 0x00);
			delay(100);
			npset(0xFF, 0x50, 0x00);
			delay(100);
			if (!forever) cycles--;
		}
		npset(0,0,0);
	#else
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
	        digitalWrite(pin_led_rx, HIGH);
	        digitalWrite(pin_led_tx, LOW);
	        delay(100);
	        digitalWrite(pin_led_rx, LOW);
	        digitalWrite(pin_led_tx, HIGH);
	        delay(100);
	        if (!forever) cycles--;
	    }
	    led_rx_off();
	    led_tx_off();
	#endif
}

// LED Indication: Airtime Lock
void led_indicate_airtime_lock() {
	#if HAS_NP == true
		npset(32,0,2);
	#endif
}

// LED Indication: Boot Error
void led_indicate_boot_error() {
	#if HAS_NP == true
		while(true) {
			npset(0xFF, 0xFF, 0xFF);
		}
	#else
		while (true) {
		    led_tx_on();
		    led_rx_off();
		    delay(10);
		    led_rx_on();
		    led_tx_off();
		    delay(5);
		}
	#endif
}

// LED Indication: Warning
void led_indicate_warning(int cycles) {
	#if HAS_NP == true
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		while(cycles > 0) {
			npset(0xFF, 0x50, 0x00);
			delay(100);
			npset(0x00, 0x00, 0x00);
			delay(100);
			if (!forever) cycles--;
		}
		npset(0,0,0);
	#else
		bool forever = (cycles == 0) ? true : false;
		cycles = forever ? 1 : cycles;
		digitalWrite(pin_led_tx, HIGH);
		while(cycles > 0) {
      led_tx_off();
      delay(100);
      led_tx_on();
      delay(100);
      if (!forever) cycles--;
    }
    led_tx_off();
	#endif
}

// LED Indication: Info
#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#if HAS_NP == true
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    npset(0x00, 0x00, 0xFF);
  			delay(100);
  			npset(0x00, 0x00, 0x00);
  			delay(100);
  			if (!forever) cycles--;
		  }
		  npset(0,0,0);
		}
	#elif BOARD_MODEL == BOARD_LORA32_V2_1
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    led_rx_off();
		    delay(100);
		    led_rx_on();
		    delay(100);
		    if (!forever) cycles--;
		  }
		  led_rx_off();
		}
	#elif BOARD_MODEL == BOARD_LORA32_V2_0
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    led_rx_off();
		    delay(100);
		    led_rx_on();
		    delay(100);
		    if (!forever) cycles--;
		  }
		  led_rx_off();
		}
	#else
		void led_indicate_info(int cycles) {
			bool forever = (cycles == 0) ? true : false;
			cycles = forever ? 1 : cycles;
			while(cycles > 0) {
		    led_tx_off();
		    delay(100);
		    led_tx_on();
		    delay(100);
		    if (!forever) cycles--;
		  }
		  led_tx_off();
		}
	#endif
#endif


unsigned long led_standby_ticks = 0;
#if MCU_VARIANT == MCU_ESP32

	#if HAS_NP == true
		int led_standby_lng = 100;
		int led_standby_cut = 200;
		int led_standby_min = 0;
		int led_standby_max = 375+led_standby_lng;
		int led_notready_min = 0;
		int led_notready_max = led_standby_max;
		int led_notready_value = led_notready_min;
		int8_t  led_notready_direction = 0;
		unsigned long led_notready_ticks = 0;
		unsigned long led_standby_wait = 350;
		unsigned long led_console_wait = 1;
		unsigned long led_notready_wait = 200;
	
	#else
		uint8_t led_standby_min = 200;
		uint8_t led_standby_max = 255;
		uint8_t led_notready_min = 0;
		uint8_t led_notready_max = 255;
		uint8_t led_notready_value = led_notready_min;
		int8_t  led_notready_direction = 0;
		unsigned long led_notready_ticks = 0;
		unsigned long led_standby_wait = 1768;
		unsigned long led_notready_wait = 150;
	#endif

#elif MCU_VARIANT == MCU_NRF52
		uint8_t led_standby_min = 200;
		uint8_t led_standby_max = 255;
		uint8_t led_notready_min = 0;
		uint8_t led_notready_max = 255;
		uint8_t led_notready_value = led_notready_min;
		int8_t  led_notready_direction = 0;
		unsigned long led_notready_ticks = 0;
		unsigned long led_standby_wait = 1768;
		unsigned long led_notready_wait = 150;
#endif

unsigned long led_standby_value = led_standby_min;
int8_t  led_standby_direction = 0;

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#if HAS_NP == true
		void led_indicate_standby() {
			led_standby_ticks++;

			if (led_standby_ticks > led_standby_wait) {
				led_standby_ticks = 0;
				
				if (led_standby_value <= led_standby_min) {
					led_standby_direction = 1;
				} else if (led_standby_value >= led_standby_max) {
					led_standby_direction = -1;
				}

				uint8_t led_standby_intensity;
				led_standby_value += led_standby_direction;
				int led_standby_ti = led_standby_value - led_standby_lng;

				if (led_standby_ti < 0) {
					led_standby_intensity = 0;
				} else if (led_standby_ti > led_standby_cut) {
					led_standby_intensity = led_standby_cut;
				} else {
					led_standby_intensity = led_standby_ti;
				}
  			npset(0x00, 0x00, led_standby_intensity);
			}
		}

		void led_indicate_console() {
			npset(0x60, 0x00, 0x60);
			// led_standby_ticks++;

			// if (led_standby_ticks > led_console_wait) {
			// 	led_standby_ticks = 0;
				
			// 	if (led_standby_value <= led_standby_min) {
			// 		led_standby_direction = 1;
			// 	} else if (led_standby_value >= led_standby_max) {
			// 		led_standby_direction = -1;
			// 	}

			// 	uint8_t led_standby_intensity;
			// 	led_standby_value += led_standby_direction;
			// 	int led_standby_ti = led_standby_value - led_standby_lng;

			// 	if (led_standby_ti < 0) {
			// 		led_standby_intensity = 0;
			// 	} else if (led_standby_ti > led_standby_cut) {
			// 		led_standby_intensity = led_standby_cut;
			// 	} else {
			// 		led_standby_intensity = led_standby_ti;
			// 	}
  	// 		npset(led_standby_intensity, 0x00, led_standby_intensity);
			// }
		}

	#else
		void led_indicate_standby() {
			led_standby_ticks++;
			if (led_standby_ticks > led_standby_wait) {
				led_standby_ticks = 0;
				if (led_standby_value <= led_standby_min) {
					led_standby_direction = 1;
				} else if (led_standby_value >= led_standby_max) {
					led_standby_direction = -1;
				}
				led_standby_value += led_standby_direction;
				if (led_standby_value > 253) {
					led_tx_on();
				} else {
					led_tx_off();
				}
				#if BOARD_MODEL == BOARD_LORA32_V2_1
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#elif BOARD_MODEL == BOARD_LORA32_V2_0
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#else
					led_rx_off();
				#endif
			}
		}

		void led_indicate_console() {
			led_indicate_standby();
		}
  #endif
#endif

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	#if HAS_NP == true
    void led_indicate_not_ready() {
    	led_standby_ticks++;

			if (led_standby_ticks > led_notready_wait) {
				led_standby_ticks = 0;
				
				if (led_standby_value <= led_standby_min) {
					led_standby_direction = 1;
				} else if (led_standby_value >= led_standby_max) {
					led_standby_direction = -1;
				}

				uint8_t led_standby_intensity;
				led_standby_value += led_standby_direction;
				int led_standby_ti = led_standby_value - led_standby_lng;

				if (led_standby_ti < 0) {
					led_standby_intensity = 0;
				} else if (led_standby_ti > led_standby_cut) {
					led_standby_intensity = led_standby_cut;
				} else {
					led_standby_intensity = led_standby_ti;
				}

  			npset(led_standby_intensity, 0x00, 0x00);
			}
		}
	#else
		void led_indicate_not_ready() {
			led_notready_ticks++;
			if (led_notready_ticks > led_notready_wait) {
				led_notready_ticks = 0;
				if (led_notready_value <= led_notready_min) {
					led_notready_direction = 1;
				} else if (led_notready_value >= led_notready_max) {
					led_notready_direction = -1;
				}
				led_notready_value += led_notready_direction;
				if (led_notready_value > 128) {
					led_tx_on();
				} else {
					led_tx_off();
				}
				#if BOARD_MODEL == BOARD_LORA32_V2_1
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#elif BOARD_MODEL == BOARD_LORA32_V2_0
					#if defined(EXTERNAL_LEDS)
						led_rx_off();
					#endif
				#else
					led_rx_off();
				#endif
			}
		}
	#endif
#endif


bool interface_bitrate_cmp(RadioInterface* p, RadioInterface* q) {
    long p_bitrate = p->getBitrate();
    long q_bitrate = q->getBitrate();
    return p_bitrate > q_bitrate;
}

// Sort interfaces in descending order according to bitrate.
void sort_interfaces() {
    std::sort(std::begin(interface_obj_sorted), std::end(interface_obj_sorted), interface_bitrate_cmp);
}

void serial_write(uint8_t byte) {
	#if HAS_BLUETOOTH || HAS_BLE == true
		if (bt_state != BT_STATE_CONNECTED) {
			Serial.write(byte);
		} else {
			SerialBT.write(byte);
		}
	#else
		Serial.write(byte);
	#endif
}

void escaped_serial_write(uint8_t byte) {
	if (byte == FEND) { serial_write(FESC); byte = TFEND; }
    if (byte == FESC) { serial_write(FESC); byte = TFESC; }
    serial_write(byte);
}

void kiss_indicate_reset() {
	serial_write(FEND);
	serial_write(CMD_RESET);
	serial_write(CMD_RESET_BYTE);
	serial_write(FEND);
}

void kiss_indicate_error(uint8_t error_code) {
	serial_write(FEND);
	serial_write(CMD_ERROR);
	serial_write(error_code);
	serial_write(FEND);
}

void kiss_indicate_radiostate(RadioInterface* radio) {
	serial_write(FEND);
	serial_write(CMD_RADIO_STATE);
	serial_write(radio->getRadioOnline());
	serial_write(FEND);
}

void kiss_indicate_stat_rx() {
    // todo, implement
	//serial_write(FEND);
	//serial_write(CMD_STAT_RX);
	//escaped_serial_write(stat_rx>>24);
	//escaped_serial_write(stat_rx>>16);
	//escaped_serial_write(stat_rx>>8);
	//escaped_serial_write(stat_rx);
	//serial_write(FEND);
}

void kiss_indicate_stat_tx() {
    // todo, implement
	//serial_write(FEND);
	//serial_write(CMD_STAT_TX);
	//escaped_serial_write(stat_tx>>24);
	//escaped_serial_write(stat_tx>>16);
	//escaped_serial_write(stat_tx>>8);
	//escaped_serial_write(stat_tx);
	//serial_write(FEND);
}

void kiss_indicate_stat_rssi() {
    uint8_t packet_rssi_val = (uint8_t)(last_rssi+rssi_offset);
	serial_write(FEND);
	serial_write(CMD_STAT_RSSI);
	escaped_serial_write(packet_rssi_val);
	serial_write(FEND);
}

void kiss_indicate_stat_snr() {
	serial_write(FEND);
	serial_write(CMD_STAT_SNR);
	escaped_serial_write(last_snr_raw);
	serial_write(FEND);
}

void kiss_indicate_radio_lock(RadioInterface* radio) {
	serial_write(FEND);
	serial_write(CMD_RADIO_LOCK);
	serial_write(radio->getRadioLock());
	serial_write(FEND);
}

void kiss_indicate_spreadingfactor(RadioInterface* radio) {
	serial_write(FEND);
	serial_write(CMD_SF);
	serial_write(radio->getSpreadingFactor());
	serial_write(FEND);
}

void kiss_indicate_codingrate(RadioInterface* radio) {
	serial_write(FEND);
	serial_write(CMD_CR);
	serial_write(radio->getCodingRate4());
	serial_write(FEND);
}

void kiss_indicate_implicit_length() {
	serial_write(FEND);
	serial_write(CMD_IMPLICIT);
	serial_write(implicit_l);
	serial_write(FEND);
}

void kiss_indicate_txpower(RadioInterface* radio) {
    int8_t txp = radio->getTxPower();
	serial_write(FEND);
	serial_write(CMD_TXPOWER);
	serial_write(txp);
	serial_write(FEND);
}

void kiss_indicate_bandwidth(RadioInterface* radio) {
    uint32_t bw = radio->getSignalBandwidth();
	serial_write(FEND);
	serial_write(CMD_BANDWIDTH);
	escaped_serial_write(bw>>24);
	escaped_serial_write(bw>>16);
	escaped_serial_write(bw>>8);
	escaped_serial_write(bw);
	serial_write(FEND);
}

void kiss_indicate_frequency(RadioInterface* radio) {
    uint32_t freq = radio->getFrequency();
	serial_write(FEND);
	serial_write(CMD_FREQUENCY);
	escaped_serial_write(freq>>24);
	escaped_serial_write(freq>>16);
	escaped_serial_write(freq>>8);
	escaped_serial_write(freq);
	serial_write(FEND);
}

void kiss_indicate_interface(int index) {
    serial_write(FEND);
    serial_write(CMD_INTERFACES);
    // print the index to the interface and the interface type
    serial_write(index);
    serial_write(interfaces[index]);
	serial_write(FEND);
}

void kiss_indicate_st_alock(RadioInterface* radio) {
	uint16_t at = (uint16_t)(radio->getSTALock()*100*100);
	serial_write(FEND);
	serial_write(CMD_ST_ALOCK);
	escaped_serial_write(at>>8);
	escaped_serial_write(at);
	serial_write(FEND);
}

void kiss_indicate_lt_alock(RadioInterface* radio) {
	uint16_t at = (uint16_t)(radio->getLTALock()*100*100);
	serial_write(FEND);
	serial_write(CMD_LT_ALOCK);
	escaped_serial_write(at>>8);
	escaped_serial_write(at);
	serial_write(FEND);
}

void kiss_indicate_channel_stats(RadioInterface* radio) {
    uint16_t ats = (uint16_t)(radio->getAirtime()*100*100);
    uint16_t atl = (uint16_t)(radio->getLongtermAirtime()*100*100);
    uint16_t cls = (uint16_t)(radio->getTotalChannelUtil()*100*100);
    uint16_t cll = (uint16_t)(radio->getLongtermChannelUtil()*100*100);
    serial_write(FEND);
    serial_write(CMD_STAT_CHTM);
    escaped_serial_write(ats>>8);
    escaped_serial_write(ats);
    escaped_serial_write(atl>>8);
    escaped_serial_write(atl);
    escaped_serial_write(cls>>8);
    escaped_serial_write(cls);
    escaped_serial_write(cll>>8);
    escaped_serial_write(cll);
    serial_write(FEND);
}

void kiss_indicate_phy_stats(RadioInterface* radio) {
    uint16_t lst = (uint16_t)(radio->getSymbolTime()*1000);
    uint16_t lsr = (uint16_t)(radio->getSymbolRate());
    uint16_t prs = (uint16_t)(radio->getPreambleLength()+4);
    uint16_t prt = (uint16_t)((radio->getPreambleLength()+4)*radio->getSymbolTime());
    uint16_t cst = (uint16_t)(radio->getCSMASlotMS());
    serial_write(FEND);
    serial_write(CMD_STAT_PHYPRM);
    escaped_serial_write(lst>>8);
    escaped_serial_write(lst);
    escaped_serial_write(lsr>>8);
    escaped_serial_write(lsr);
    escaped_serial_write(prs>>8);
    escaped_serial_write(prs);
    escaped_serial_write(prt>>8);
    escaped_serial_write(prt);
    escaped_serial_write(cst>>8);
    escaped_serial_write(cst);
    serial_write(FEND);
}

void kiss_indicate_battery() {
	#if MCU_VARIANT == MCU_ESP32
		serial_write(FEND);
		serial_write(CMD_STAT_BAT);
		escaped_serial_write(battery_state);
		escaped_serial_write((uint8_t)int(battery_percent));
		serial_write(FEND);
	#endif
}

void kiss_indicate_btpin() {
	#if HAS_BLUETOOTH || HAS_BLE == true
		serial_write(FEND);
		serial_write(CMD_BT_PIN);
		escaped_serial_write(bt_ssp_pin>>24);
		escaped_serial_write(bt_ssp_pin>>16);
		escaped_serial_write(bt_ssp_pin>>8);
		escaped_serial_write(bt_ssp_pin);
		serial_write(FEND);
	#endif
}

void kiss_indicate_random(uint8_t byte) {
	serial_write(FEND);
	serial_write(CMD_RANDOM);
	serial_write(byte);
	serial_write(FEND);
}

void kiss_indicate_fbstate() {
	serial_write(FEND);
	serial_write(CMD_FB_EXT);
	#if HAS_DISPLAY
		if (disp_ext_fb) {
			serial_write(0x01);
		} else {
			serial_write(0x00);
		}
	#else
		serial_write(0xFF);
	#endif
	serial_write(FEND);
}

#if MCU_VARIANT == MCU_ESP32 || MCU_VARIANT == MCU_NRF52
	void kiss_indicate_device_hash() {
	  serial_write(FEND);
	  serial_write(CMD_DEV_HASH);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_target_fw_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x01);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_firmware_hash_target[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_fw_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x02);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_firmware_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_bootloader_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x03);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_bootloader_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}

	void kiss_indicate_partition_table_hash() {
	  serial_write(FEND);
	  serial_write(CMD_HASHES);
	  serial_write(0x04);
	  for (int i = 0; i < DEV_HASH_LEN; i++) {
	    uint8_t byte = dev_partition_table_hash[i];
	 		escaped_serial_write(byte);
	  }
	  serial_write(FEND);
	}
#endif

void kiss_indicate_fb() {
	serial_write(FEND);
	serial_write(CMD_FB_READ);
	#if HAS_DISPLAY
		for (int i = 0; i < 512; i++) {
			uint8_t byte = fb[i];
			escaped_serial_write(byte);
		}
	#else
		serial_write(0xFF);
	#endif
	serial_write(FEND);
}

void kiss_indicate_ready() {
	serial_write(FEND);
	serial_write(CMD_READY);
	serial_write(0x01);
	serial_write(FEND);
}

void kiss_indicate_not_ready() {
	serial_write(FEND);
	serial_write(CMD_READY);
	serial_write(0x00);
	serial_write(FEND);
}

void kiss_indicate_promisc() {
	serial_write(FEND);
	serial_write(CMD_PROMISC);
	if (promisc) {
		serial_write(0x01);
	} else {
		serial_write(0x00);
	}
	serial_write(FEND);
}

void kiss_indicate_detect() {
	serial_write(FEND);
	serial_write(CMD_DETECT);
	serial_write(DETECT_RESP);
	serial_write(FEND);
}

void kiss_indicate_version() {
	serial_write(FEND);
	serial_write(CMD_FW_VERSION);
	serial_write(MAJ_VERS);
	serial_write(MIN_VERS);
	serial_write(FEND);
}

void kiss_indicate_platform() {
	serial_write(FEND);
	serial_write(CMD_PLATFORM);
	serial_write(PLATFORM);
	serial_write(FEND);
}

void kiss_indicate_board() {
	serial_write(FEND);
	serial_write(CMD_BOARD);
	serial_write(BOARD_MODEL);
	serial_write(FEND);
}

void kiss_indicate_mcu() {
	serial_write(FEND);
	serial_write(CMD_MCU);
	serial_write(MCU_VARIANT);
	serial_write(FEND);
}

inline bool isSplitPacket(uint8_t header) {
	return (header & FLAG_SPLIT);
}

inline uint8_t packetSequence(uint8_t header) {
	return header >> 4;
}

void set_implicit_length(uint8_t len) {
	implicit_l = len;
	if (implicit_l != 0) {
		implicit = true;
	} else {
		implicit = false;
	}
}

void setTXPower(RadioInterface* radio, int txp) {
    // Todo, revamp this function. The current parameters for setTxPower are
    // suboptimal, as some chips have power amplifiers which means that the max
    // dBm is not always the same.
    if (model == MODEL_11) {
        if (interfaces[radio->getIndex()] == SX128X) {
            radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
        } else {
            radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
        }
    }
    if (model == MODEL_12) {
        if (interfaces[radio->getIndex()] == SX128X) {
            radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
        } else {
            radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
        }
    }

    if (model == MODEL_21) {
        if (interfaces[radio->getIndex()] == SX128X) {
            radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
        } else {
            radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
        }
    }

    if (model == MODEL_A1) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_A2) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_A3) radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
    if (model == MODEL_A4) radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
    if (model == MODEL_A5) radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
    if (model == MODEL_A6) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_A7) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_A8) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_A9) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);

    if (model == MODEL_B3) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_B4) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_B8) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_B9) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);

    if (model == MODEL_C4) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_C9) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);

    if (model == MODEL_E4) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_E9) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_E3) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_E8) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);

    if (model == MODEL_FE) radio->setTxPower(txp, PA_OUTPUT_PA_BOOST_PIN);
    if (model == MODEL_FF) radio->setTxPower(txp, PA_OUTPUT_RFO_PIN);
}

uint8_t getRandom(RadioInterface* radio) {
	if (radio->getRadioOnline()) {
		return radio->random();
	} else {
		return 0x00;
	}
}

uint8_t getInterfaceIndex(uint8_t byte) {
    switch (byte) {
        case CMD_INT0_DATA:
        case CMD_SEL_INT0:
            return 0;
        case CMD_INT1_DATA:
        case CMD_SEL_INT1:
            return 1;
        case CMD_INT2_DATA:
        case CMD_SEL_INT2:
            return 2;
        case CMD_INT3_DATA:
        case CMD_SEL_INT3:
            return 3;
        case CMD_INT4_DATA:
        case CMD_SEL_INT4:
            return 4;
        case CMD_INT5_DATA:
        case CMD_SEL_INT5:
            return 5;
        case CMD_INT6_DATA:
        case CMD_SEL_INT6:
            return 6;
        case CMD_INT7_DATA:
        case CMD_SEL_INT7:
            return 7;
        case CMD_INT8_DATA:
        case CMD_SEL_INT8:
            return 8;
        case CMD_INT9_DATA:
        case CMD_SEL_INT9:
            return 9;
        case CMD_INT10_DATA:
        case CMD_SEL_INT10:
            return 10;
        case CMD_INT11_DATA:
        case CMD_SEL_INT11:
            return 11;
        default:
            return 0;
    }
}

uint8_t getInterfaceCommandByte(uint8_t index) {
    switch (index) {
        case 0:
            return CMD_INT0_DATA;
        case 1:
            return CMD_INT1_DATA;
        case 2:
            return CMD_INT2_DATA;
        case 3:
            return CMD_INT3_DATA;
        case 4:
            return CMD_INT4_DATA;
        case 5:
            return CMD_INT5_DATA;
        case 6:
            return CMD_INT6_DATA;
        case 7:
            return CMD_INT7_DATA;
        case 8:
            return CMD_INT8_DATA;
        case 9:
            return CMD_INT9_DATA;
        case 10:
            return CMD_INT10_DATA;
        case 11:
            return CMD_INT11_DATA;
        default:
            return 0;
    }
}

uint16_t getQueueSize(uint8_t index) {
    switch (index) {
        case 0:
            return CONFIG_QUEUE_0_SIZE;
        #if INTERFACE_COUNT > 1
        case 1:
            return CONFIG_QUEUE_1_SIZE;
        #endif
        #if INTERFACE_COUNT > 2
        case 2:
            return CONFIG_QUEUE_2_SIZE;
        #endif
        #if INTERFACE_COUNT > 3
        case 3:
            return CONFIG_QUEUE_3_SIZE;
        #endif
        #if INTERFACE_COUNT > 4 
        case 4:
            return CONFIG_QUEUE_4_SIZE;
        #endif
        #if INTERFACE_COUNT > 5
        case 5:
            return CONFIG_QUEUE_5_SIZE;
        #endif
        #if INTERFACE_COUNT > 6
        case 6:
            return CONFIG_QUEUE_6_SIZE;
        #endif
        #if INTERFACE_COUNT > 7
        case 7:
            return CONFIG_QUEUE_7_SIZE;
        #endif
        #if INTERFACE_COUNT > 8
        case 8:
            return CONFIG_QUEUE_8_SIZE;
        #endif
        #if INTERFACE_COUNT > 9
        case 9:
            return CONFIG_QUEUE_9_SIZE;
        #endif
        #if INTERFACE_COUNT > 10
        case 10:
            return CONFIG_QUEUE_10_SIZE;
        #endif
        #if INTERFACE_COUNT > 11
        case 11:
            return CONFIG_QUEUE_11_SIZE;
        #endif
    }
}

void promisc_enable() {
	promisc = true;
}

void promisc_disable() {
	promisc = false;
}

#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
    bool eeprom_begin() {
        InternalFS.begin();

        file.open(EEPROM_FILE, FILE_O_READ);

        // if file doesn't exist
        if (!file) {
            if (file.open(EEPROM_FILE, FILE_O_WRITE)) {
                // initialise the file with empty content
                uint8_t empty_content[EEPROM_SIZE] = {0};
                file.write(empty_content, EEPROM_SIZE);
                return true;
            } else {
                return false;
            }
        } else {
            file.close();
            file.open(EEPROM_FILE, FILE_O_WRITE);
            return true;
        }
    }

    uint8_t eeprom_read(uint32_t mapped_addr) {
        uint8_t byte;
        void* byte_ptr = &byte;
        file.seek(mapped_addr);
        file.read(byte_ptr, 1);
        return byte;
    }
#endif

bool eeprom_info_locked() {
    #if HAS_EEPROM
	    uint8_t lock_byte = EEPROM.read(eeprom_addr(ADDR_INFO_LOCK));
    #elif MCU_VARIANT == MCU_NRF52
        uint8_t lock_byte = eeprom_read(eeprom_addr(ADDR_INFO_LOCK));
    #endif
	if (lock_byte == INFO_LOCK_BYTE) {
		return true;
	} else {
		return false;
	}
}

void eeprom_dump_info() {
	for (int addr = ADDR_PRODUCT; addr <= ADDR_INFO_LOCK; addr++) {
        #if HAS_EEPROM
            uint8_t byte = EEPROM.read(eeprom_addr(addr));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t byte = eeprom_read(eeprom_addr(addr));
        #endif
		escaped_serial_write(byte);
	}
}

void eeprom_dump_config() {
	for (int addr = ADDR_CONF_SF; addr <= ADDR_CONF_OK; addr++) {
        #if HAS_EEPROM
            uint8_t byte = EEPROM.read(eeprom_addr(addr));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t byte = eeprom_read(eeprom_addr(addr));
        #endif
		escaped_serial_write(byte);
	}
}

void eeprom_dump_all() {
	for (int addr = 0; addr < EEPROM_RESERVED; addr++) {
        #if HAS_EEPROM
            uint8_t byte = EEPROM.read(eeprom_addr(addr));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t byte = eeprom_read(eeprom_addr(addr));
        #endif
		escaped_serial_write(byte);
	}
}

void kiss_dump_eeprom() {
	serial_write(FEND);
	serial_write(CMD_ROM_READ);
	eeprom_dump_all();
	serial_write(FEND);
}

#if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
void eeprom_flush() {
    // sync file contents to flash
    file.close();
    file.open(EEPROM_FILE, FILE_O_WRITE);
    written_bytes = 0;
}
#endif

void eeprom_update(int mapped_addr, uint8_t byte) {
	#if MCU_VARIANT == MCU_ESP32
		if (EEPROM.read(mapped_addr) != byte) {
			EEPROM.write(mapped_addr, byte);
			EEPROM.commit();
		}
    #elif !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
        // todo: clean up this implementation, writing one byte and syncing
        // each time is really slow, but this is also suboptimal
        uint8_t read_byte;
        void* read_byte_ptr = &read_byte;
        file.seek(mapped_addr);
        file.read(read_byte_ptr, 1);
        file.seek(mapped_addr);
        if (read_byte != byte) {
            file.write(byte);
        }
        written_bytes++;
        
        if (((mapped_addr - eeprom_addr(0)) == ADDR_INFO_LOCK) || (mapped_addr - eeprom_addr(0)) == ADDR_CONF_OK) {
                // have to do a flush because we're only writing 1 byte and it syncs after 4
                eeprom_flush();
        }

        if (written_bytes >= 4) {
            file.close();
            file.open(EEPROM_FILE, FILE_O_WRITE);
            written_bytes = 0;
        }
	#endif
}

void eeprom_write(uint8_t addr, uint8_t byte) {
	if (!eeprom_info_locked() && addr >= 0 && addr < EEPROM_RESERVED) {
		eeprom_update(eeprom_addr(addr), byte);
	} else {
		kiss_indicate_error(ERROR_EEPROM_LOCKED);
	}
}

void eeprom_erase() {
	for (int addr = 0; addr < EEPROM_RESERVED; addr++) {
		eeprom_update(eeprom_addr(addr), 0xFF);
	}
	hard_reset();
}

bool eeprom_lock_set() {
    #if HAS_EEPROM
	    if (EEPROM.read(eeprom_addr(ADDR_INFO_LOCK)) == INFO_LOCK_BYTE) {
    #elif MCU_VARIANT == MCU_NRF52
        if (eeprom_read(eeprom_addr(ADDR_INFO_LOCK)) == INFO_LOCK_BYTE) {
    #endif
		return true;
	} else {
		return false;
	}
}

bool eeprom_product_valid() {
    #if HAS_EEPROM
	    uint8_t rval = EEPROM.read(eeprom_addr(ADDR_PRODUCT));
    #elif MCU_VARIANT == MCU_NRF52
	    uint8_t rval = eeprom_read(eeprom_addr(ADDR_PRODUCT));
    #endif

	#if PLATFORM == PLATFORM_ESP32
	if (rval == PRODUCT_RNODE || rval == BOARD_RNODE_NG_20 || rval == BOARD_RNODE_NG_21 || rval == PRODUCT_HMBRW || rval == PRODUCT_TBEAM || rval == PRODUCT_T32_10 || rval == PRODUCT_T32_20 || rval == PRODUCT_T32_21 || rval == PRODUCT_H32_V2 || rval == PRODUCT_H32_V3) {
	#elif PLATFORM == PLATFORM_NRF52
	if (rval == PRODUCT_RAK4631 || rval == PRODUCT_HMBRW || rval == PRODUCT_FREENODE) {
	#else
	if (false) {
	#endif
		return true;
	} else {
		return false;
	}
}

bool eeprom_model_valid() {
    #if HAS_EEPROM
        model = EEPROM.read(eeprom_addr(ADDR_MODEL));
    #elif MCU_VARIANT == MCU_NRF52
        model = eeprom_read(eeprom_addr(ADDR_MODEL));
    #endif
	#if BOARD_MODEL == BOARD_RNODE
	if (model == MODEL_A4 || model == MODEL_A9 || model == MODEL_FF || model == MODEL_FE) {
	#elif BOARD_MODEL == BOARD_RNODE_NG_20
	if (model == MODEL_A3 || model == MODEL_A8) {
	#elif BOARD_MODEL == BOARD_RNODE_NG_21
	if (model == MODEL_A2 || model == MODEL_A7) {
	#elif BOARD_MODEL == BOARD_T3S3
	if (model == MODEL_A1 || model == MODEL_A5 || model == MODEL_A6) {
	#elif BOARD_MODEL == BOARD_HMBRW
	if (model == MODEL_FF || model == MODEL_FE) {
	#elif BOARD_MODEL == BOARD_TBEAM
	if (model == MODEL_E4 || model == MODEL_E9 || model == MODEL_E3 || model == MODEL_E8) {
	#elif BOARD_MODEL == BOARD_LORA32_V1_0
	if (model == MODEL_BA || model == MODEL_BB) {
	#elif BOARD_MODEL == BOARD_LORA32_V2_0
	if (model == MODEL_B3 || model == MODEL_B8) {
	#elif BOARD_MODEL == BOARD_LORA32_V2_1
	if (model == MODEL_B4 || model == MODEL_B9) {
	#elif BOARD_MODEL == BOARD_HELTEC32_V2
	if (model == MODEL_C4 || model == MODEL_C9) {
	#elif BOARD_MODEL == BOARD_HELTEC32_V3
	if (model == MODEL_C5 || model == MODEL_CA) {
    #elif BOARD_MODEL == BOARD_RAK4631
    if (model == MODEL_11 || model == MODEL_12 || model == MODEL_13 || model == MODEL_14 || model == MODEL_21) {
	#elif BOARD_MODEL == BOARD_HUZZAH32
	if (model == MODEL_FF) {
	#elif BOARD_MODEL == BOARD_GENERIC_ESP32
	if (model == MODEL_FF || model == MODEL_FE) {
	#else
	if (false) {
	#endif
		return true;
	} else {
		return false;
	}
}

bool eeprom_hwrev_valid() {
    #if HAS_EEPROM
        hwrev = EEPROM.read(eeprom_addr(ADDR_HW_REV));
    #elif MCU_VARIANT == MCU_NRF52
        hwrev = eeprom_read(eeprom_addr(ADDR_HW_REV));
    #endif
	if (hwrev != 0x00 && hwrev != 0xFF) {
		return true;
	} else {
		return false;
	}
}

bool eeprom_checksum_valid() {
	char *data = (char*)malloc(CHECKSUMMED_SIZE);
	for (uint8_t  i = 0; i < CHECKSUMMED_SIZE; i++) {
        #if HAS_EEPROM
            char byte = EEPROM.read(eeprom_addr(i));
        #elif MCU_VARIANT == MCU_NRF52
            char byte = eeprom_read(eeprom_addr(i));
        #endif
		data[i] = byte;
	}
	
	unsigned char *hash = MD5::make_hash(data, CHECKSUMMED_SIZE);
	bool checksum_valid = true;
	for (uint8_t i = 0; i < 16; i++) {
        #if HAS_EEPROM
            uint8_t stored_chk_byte = EEPROM.read(eeprom_addr(ADDR_CHKSUM+i));
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t stored_chk_byte = eeprom_read(eeprom_addr(ADDR_CHKSUM+i));
        #endif
		uint8_t calced_chk_byte = (uint8_t)hash[i];
		if (stored_chk_byte != calced_chk_byte) {
			checksum_valid = false;
		}
	}

	free(hash);
	free(data);
	return checksum_valid;
}

void bt_conf_save(bool is_enabled) {
	if (is_enabled) {
		eeprom_update(eeprom_addr(ADDR_CONF_BT), BT_ENABLE_BYTE);
        #if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
            // have to do a flush because we're only writing 1 byte and it syncs after 8
            eeprom_flush();
        #endif
	} else {
		eeprom_update(eeprom_addr(ADDR_CONF_BT), 0x00);
        #if !HAS_EEPROM && MCU_VARIANT == MCU_NRF52
            // have to do a flush because we're only writing 1 byte and it syncs after 8
            eeprom_flush();
        #endif
	}
}

void di_conf_save(uint8_t dint) {
	eeprom_update(eeprom_addr(ADDR_CONF_DINT), dint);
}

void da_conf_save(uint8_t dadr) {
	eeprom_update(eeprom_addr(ADDR_CONF_DADR), dadr);
}

bool eeprom_have_conf() {
    #if HAS_EEPROM
	    if (EEPROM.read(eeprom_addr(ADDR_CONF_OK)) == CONF_OK_BYTE) {
    #elif MCU_VARIANT == MCU_NRF52
        if (eeprom_read(eeprom_addr(ADDR_CONF_OK)) == CONF_OK_BYTE) {
    #endif
		return true;
	} else {
		return false;
	}
}

void eeprom_conf_load(RadioInterface* radio) {
	if (eeprom_have_conf()) {
        if (!(radio->getRadioOnline())) {
        #if HAS_EEPROM
            uint8_t sf = EEPROM.read(eeprom_addr(ADDR_CONF_SF));
            uint8_t cr = EEPROM.read(eeprom_addr(ADDR_CONF_CR));
            uint8_t txp = EEPROM.read(eeprom_addr(ADDR_CONF_TXP));
            uint32_t freq = (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x00) << 24 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x01) << 16 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x02) << 8 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_FREQ)+0x03);
            uint32_t bw = (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x00) << 24 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x01) << 16 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x02) << 8 | (uint32_t)EEPROM.read(eeprom_addr(ADDR_CONF_BW)+0x03);
        #elif MCU_VARIANT == MCU_NRF52
            uint8_t sf = eeprom_read(eeprom_addr(ADDR_CONF_SF));
            uint8_t cr = eeprom_read(eeprom_addr(ADDR_CONF_CR));
            uint8_t txp = eeprom_read(eeprom_addr(ADDR_CONF_TXP));
            uint32_t freq = (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x00) << 24 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x01) << 16 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x02) << 8 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_FREQ)+0x03);
            uint32_t bw = (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x00) << 24 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x01) << 16 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x02) << 8 | (uint32_t)eeprom_read(eeprom_addr(ADDR_CONF_BW)+0x03);
        #endif
            radio->setSpreadingFactor(sf);
            radio->setCodingRate4(cr);
            setTXPower(radio, txp);
            radio->setFrequency(freq);
            radio->setSignalBandwidth(bw);
            radio->updateBitrate();
        }
	}
}

void eeprom_conf_save(RadioInterface* radio) {
	if (hw_ready && radio->getRadioOnline()) {
		eeprom_update(eeprom_addr(ADDR_CONF_SF), radio->getSpreadingFactor());
		eeprom_update(eeprom_addr(ADDR_CONF_CR), radio->getCodingRate4());
		eeprom_update(eeprom_addr(ADDR_CONF_TXP), radio->getTxPower());

        uint32_t bw = radio->getSignalBandwidth();

		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x00, bw>>24);
		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x01, bw>>16);
		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x02, bw>>8);
		eeprom_update(eeprom_addr(ADDR_CONF_BW)+0x03, bw);

        uint32_t freq = radio->getFrequency();

		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x00, freq>>24);
		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x01, freq>>16);
		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x02, freq>>8);
		eeprom_update(eeprom_addr(ADDR_CONF_FREQ)+0x03, freq);

		eeprom_update(eeprom_addr(ADDR_CONF_OK), CONF_OK_BYTE);
		led_indicate_info(10);
	} else {
		led_indicate_warning(10);
	}
}

void eeprom_conf_delete() {
	eeprom_update(eeprom_addr(ADDR_CONF_OK), 0x00);
}

void unlock_rom() {
	led_indicate_error(50);
	eeprom_erase();
}

typedef struct FIFOBuffer
{
  unsigned char *begin;
  unsigned char *end;
  unsigned char * volatile head;
  unsigned char * volatile tail;
} FIFOBuffer;

inline bool fifo_isempty(const FIFOBuffer *f) {
  return f->head == f->tail;
}

inline bool fifo_isfull(const FIFOBuffer *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo_push(FIFOBuffer *f, unsigned char c) {
  *(f->tail) = c;
  
  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline unsigned char fifo_pop(FIFOBuffer *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo_flush(FIFOBuffer *f) {
  f->head = f->tail;
}

#if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
	static inline bool fifo_isempty_locked(const FIFOBuffer *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo_isempty(f);
	  }
	  return result;
	}

	static inline bool fifo_isfull_locked(const FIFOBuffer *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo_isfull(f);
	  }
	  return result;
	}

	static inline void fifo_push_locked(FIFOBuffer *f, unsigned char c) {
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    fifo_push(f, c);
	  }
	}
#endif

/*
static inline unsigned char fifo_pop_locked(FIFOBuffer *f) {
  unsigned char c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = fifo_pop(f);
  }
  return c;
}
*/

inline void fifo_init(FIFOBuffer *f, unsigned char *buffer, size_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline size_t fifo_len(FIFOBuffer *f) {
  return f->end - f->begin;
}

typedef struct FIFOBuffer16
{
  uint16_t *begin;
  uint16_t *end;
  uint16_t * volatile head;
  uint16_t * volatile tail;
} FIFOBuffer16;

inline bool fifo16_isempty(const FIFOBuffer16 *f) {
  return f->head == f->tail;
}

inline bool fifo16_isfull(const FIFOBuffer16 *f) {
  return ((f->head == f->begin) && (f->tail == f->end)) || (f->tail == f->head - 1);
}

inline void fifo16_push(FIFOBuffer16 *f, uint16_t c) {
  *(f->tail) = c;

  if (f->tail == f->end) {
    f->tail = f->begin;
  } else {
    f->tail++;
  }
}

inline uint16_t fifo16_pop(FIFOBuffer16 *f) {
  if(f->head == f->end) {
    f->head = f->begin;
    return *(f->end);
  } else {
    return *(f->head++);
  }
}

inline void fifo16_flush(FIFOBuffer16 *f) {
  f->head = f->tail;
}

#if MCU_VARIANT != MCU_ESP32 && MCU_VARIANT != MCU_NRF52
	static inline bool fifo16_isempty_locked(const FIFOBuffer16 *f) {
	  bool result;
	  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    result = fifo16_isempty(f);
	  }

	  return result;
	}
#endif

/*
static inline bool fifo16_isfull_locked(const FIFOBuffer16 *f) {
  bool result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    result = fifo16_isfull(f);
  }
  return result;
}


static inline void fifo16_push_locked(FIFOBuffer16 *f, uint16_t c) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    fifo16_push(f, c);
  }
}

static inline size_t fifo16_pop_locked(FIFOBuffer16 *f) {
  size_t c;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    c = fifo16_pop(f);
  }
  return c;
}
*/

inline void fifo16_init(FIFOBuffer16 *f, uint16_t *buffer, uint16_t size) {
  f->head = f->tail = f->begin = buffer;
  f->end = buffer + size;
}

inline uint16_t fifo16_len(FIFOBuffer16 *f) {
  return (f->end - f->begin);
}
