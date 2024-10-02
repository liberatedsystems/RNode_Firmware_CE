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


#include <Adafruit_GFX.h>

#if DISPLAY == OLED
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#define DISP_W 128
#define DISP_H 64
#define DISPLAY_BLACK SSD1306_BLACK
#define DISPLAY_WHITE SSD1306_WHITE
#elif DISPLAY == EINK_BW || DISPLAY == EINK_3C
void (*display_callback)();
void display_add_callback(void (*callback)()) {
    display_callback = callback;
}
void busyCallback(const void* p) {
    display_callback();
}
#define DISPLAY_BLACK GxEPD_BLACK
#define DISPLAY_WHITE GxEPD_WHITE
#endif
#if DISPLAY == EINK_BW
// use GxEPD2 because adafruit EPD support for partial refresh is bad
#include <GxEPD2_BW.h>
#include <SPI.h>
#elif DISPLAY == EINK_3C
#include <GxEPD2_3C.h>
#include <SPI.h>
#endif

#include "Fonts/Org_01.h"
#if BOARD_MODEL == BOARD_RNODE_NG_20 || BOARD_MODEL == BOARD_LORA32_V2_0
  #if DISPLAY == OLED
  #define DISP_RST -1
  #define DISP_ADDR 0x3C
  #endif
#elif BOARD_MODEL == BOARD_TBEAM
  #if DISPLAY == OLED
  #define DISP_RST 13
  #define DISP_ADDR 0x3C
  #define DISP_CUSTOM_ADDR true
  #endif
#elif BOARD_MODEL == BOARD_HELTEC32_V2 || BOARD_MODEL == BOARD_LORA32_V1_0
  #if DISPLAY == OLED
  #define DISP_RST 16
  #define DISP_ADDR 0x3C
  #define SCL_OLED 15
  #define SDA_OLED 4
  #endif
#elif BOARD_MODEL == BOARD_HELTEC32_V3
  #define DISP_RST 21
  #define DISP_ADDR 0x3C
  #define SCL_OLED 18
  #define SDA_OLED 17
#elif BOARD_MODEL == BOARD_RNODE_NG_21
  #if DISPLAY == OLED
  #define DISP_RST -1
  #define DISP_ADDR 0x3C
  #endif
#elif BOARD_MODEL == BOARD_T3S3
  #if DISPLAY == OLED
  #define DISP_RST 21
  #define DISP_ADDR 0x3C
  #define SCL_OLED 17
  #define SDA_OLED 18
  #endif
#elif BOARD_MODEL == BOARD_RAK4631
  #if DISPLAY == OLED
  // todo: add support for OLED board
  #elif DISPLAY == EINK_BW
  #define DISP_W 250
  #define DISP_H 122
  #define DISP_ADDR -1
  #define DISPLAY_MODEL GxEPD2_213_BN
  #elif DISPLAY == EINK_3C
  #define DISP_W 250
  #define DISP_H 122
  #define DISP_ADDR -1
  #define DISPLAY_MODEL GxEPD2_213_Z98c 
  #endif
#elif BOARD_MODEL == BOARD_TECHO
  SPIClass displaySPI = SPIClass(NRF_SPIM0, pin_disp_miso, pin_disp_sck, pin_disp_mosi);
  #define DISP_W 128
  #define DISP_H 64
  #define DISP_ADDR -1
#else
  #define DISP_RST -1
  #define DISP_ADDR 0x3C
  #define DISP_CUSTOM_ADDR true
#endif

#define SMALL_FONT &Org_01

#include "Graphics.h"

#if BOARD_MODEL == BOARD_RAK4631
  #if DISPLAY == EINK_BW
  GxEPD2_BW<DISPLAY_MODEL, DISPLAY_MODEL::HEIGHT> display(DISPLAY_MODEL(pin_disp_cs, pin_disp_dc, pin_disp_reset, pin_disp_busy));
  float disp_target_fps = 0.2;
  uint32_t last_epd_refresh = 0;
  #define REFRESH_PERIOD 300000 // 5 minutes in ms
  #elif DISPLAY == EINK_3C
  GxEPD2_3C<DISPLAY_MODEL, DISPLAY_MODEL::HEIGHT> display(DISPLAY_MODEL(pin_disp_cs, pin_disp_dc, pin_disp_reset, pin_disp_busy));
  float disp_target_fps = 0.05; // refresh usually takes longer on 3C, hence this is 4x the BW refresh period
  uint32_t last_epd_refresh = 0;
  #define REFRESH_PERIOD 600000 // 10 minutes in ms
  #endif
#elif BOARD_MODEL == BOARD_TECHO
GxEPD2_BW<DISPLAY_MODEL, DISPLAY_MODEL::HEIGHT> display(DISPLAY_MODEL(pin_disp_cs, pin_disp_dc, pin_disp_reset, pin_disp_busy));
float disp_target_fps = 0.2;
uint32_t last_epd_refresh = 0;
#define REFRESH_PERIOD 300000 // 5 minutes in ms
#else
  #if DISPLAY == OLED
  Adafruit_SSD1306 display(DISP_W, DISP_H, &Wire, DISP_RST);
  float disp_target_fps = 7;
  #define SCREENSAVER_TIME 500 // ms
  uint32_t last_screensaver = 0;
  #define SCREENSAVER_INTERVAL 600000 // 10 minutes in ms
  bool screensaver_enabled = false;
  #endif
#endif

#define DISP_MODE_UNKNOWN   0x00
#define DISP_MODE_LANDSCAPE 0x01
#define DISP_MODE_PORTRAIT  0x02
#define DISP_PIN_SIZE   6
uint8_t disp_mode = DISP_MODE_UNKNOWN;
uint8_t disp_ext_fb = false;
unsigned char fb[512];
uint32_t last_disp_update = 0;
int disp_update_interval = 1000/disp_target_fps;

uint32_t last_page_flip = 0;
uint32_t last_interface_page_flip = 0;
int page_interval = 4000;
bool device_signatures_ok();
bool device_firmware_ok();

bool stat_area_initialised = false;
bool radio_online = false;

#define START_PAGE 0
const uint8_t pages = 3;
uint8_t disp_page = START_PAGE;
uint8_t interface_page = START_PAGE;

uint8_t online_interface_list[INTERFACE_COUNT] = {0};

uint8_t online_interfaces = 0;

#if DISP_H == 64
#define WATERFALL_SIZE 46
#elif DISP_H == 122
#define WATERFALL_SIZE 92
#else
#define WATERFALL_SIZE int(DISP_H * 0.75) // default to 75% of the display height
// add more eink compatible boards here
#endif

int waterfall[INTERFACE_COUNT][WATERFALL_SIZE] = {0};
int waterfall_head[INTERFACE_COUNT] = {0};

int p_ad_x = 0;
int p_ad_y = 0;
int p_as_x = 0;
int p_as_y = 0;

#if DISP_H == 122
GFXcanvas1 stat_area(DISP_H, DISP_W/2);
GFXcanvas1 disp_area(DISP_H, DISP_W/2);
#else
GFXcanvas1 stat_area(64, 64);
GFXcanvas1 disp_area(64, 64);
#endif

void update_area_positions() {
  if (disp_mode == DISP_MODE_PORTRAIT) {
    p_ad_x = 0;
    p_ad_y = 0;
    p_as_x = 0;
    p_as_y = DISP_H;
  } else if (disp_mode == DISP_MODE_LANDSCAPE) {
    p_ad_x = 0;
    p_ad_y = 0;
    p_as_x = DISP_H;
    p_as_y = 0;
  }
}

uint8_t display_contrast = 0x00;
#if DISPLAY == OLED
void set_contrast(Adafruit_SSD1306 *display, uint8_t contrast) {
    display->ssd1306_command(SSD1306_SETCONTRAST);
    display->ssd1306_command(contrast);
}
#endif

bool display_init() {
  #if HAS_DISPLAY
    #if BOARD_MODEL == BOARD_RNODE_NG_20 || BOARD_MODEL == BOARD_LORA32_V2_0
      int pin_display_en = 16;
      digitalWrite(pin_display_en, LOW);
      delay(50);
      digitalWrite(pin_display_en, HIGH);
    #elif BOARD_MODEL == BOARD_T3S3
      Wire.begin(SDA_OLED, SCL_OLED);
    #elif BOARD_MODEL == BOARD_HELTEC32_V2
      Wire.begin(SDA_OLED, SCL_OLED);
    #elif BOARD_MODEL == BOARD_HELTEC32_V3
      // enable vext / pin 36
      pinMode(Vext, OUTPUT);
      digitalWrite(Vext, LOW);
      delay(50);
      int pin_display_en = 21;
      pinMode(pin_display_en, OUTPUT);
      digitalWrite(pin_display_en, LOW);
      delay(50);
      digitalWrite(pin_display_en, HIGH);
      delay(50);
      Wire.begin(SDA_OLED, SCL_OLED);
    #elif BOARD_MODEL == BOARD_LORA32_V1_0
      int pin_display_en = 16;
      digitalWrite(pin_display_en, LOW);
      delay(50);
      digitalWrite(pin_display_en, HIGH);
      Wire.begin(SDA_OLED, SCL_OLED);
    #elif BOARD_MODEL == BOARD_TECHO
      pinMode(pin_disp_en, INPUT_PULLUP);
      digitalWrite(pin_disp_en, HIGH);

      display.init(0, true, 10, false, displaySPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
      display.setPartialWindow(0, 0, DISP_W, DISP_H);

      // Because refreshing this display can take some time, sometimes serial
      // commands will be missed. Therefore, during periods where the device is
      // waiting for the display to update, it will poll the serial buffer to
      // check for any commands from the host.
      display.epd2.setBusyCallback(busyCallback);
    #elif BOARD_MODEL == BOARD_RAK4631
      #if DISPLAY == OLED
      #elif DISPLAY == EINK_BW || DISPLAY == EINK_3C
      pinMode(pin_disp_en, INPUT_PULLUP);
      digitalWrite(pin_disp_en, HIGH);
      display.init(0, true, 10, false, SPI, SPISettings(4000000, MSBFIRST, SPI_MODE0));
      display.setPartialWindow(0, 0, DISP_W, DISP_H);

      // Because refreshing this display can take some time, sometimes serial
      // commands will be missed. Therefore, during periods where the device is
      // waiting for the display to update, it will poll the serial buffer to
      // check for any commands from the host.
      display.epd2.setBusyCallback(busyCallback);
      #endif
    #endif

    #if DISP_CUSTOM_ADDR == true
      #if HAS_EEPROM
      uint8_t display_address = EEPROM.read(eeprom_addr(ADDR_CONF_DADR));
      #elif MCU_VARIANT == MCU_NRF52
      uint8_t display_address = eeprom_read(eeprom_addr(ADDR_CONF_DADR));
      #endif
      if (display_address == 0xFF) display_address = DISP_ADDR;
    #else
      uint8_t display_address = DISP_ADDR;
    #endif


    #if DISPLAY == OLED
    if(!display.begin(SSD1306_SWITCHCAPVCC, display_address)) {
    #elif DISPLAY == EINK_BW || DISPLAY == EINK_3C
    // don't check if display is actually connected
    if(false) {
    #endif
      return false;
    } else {
      #if DISPLAY == OLED
        set_contrast(&display, display_contrast);
      #endif
      #if BOARD_MODEL == BOARD_RNODE_NG_20
        disp_mode = DISP_MODE_PORTRAIT;
        display.setRotation(3);
      #elif BOARD_MODEL == BOARD_RNODE_NG_21
        disp_mode = DISP_MODE_PORTRAIT;
        display.setRotation(3);
      #elif BOARD_MODEL == BOARD_LORA32_V1_0
        disp_mode = DISP_MODE_PORTRAIT;
        display.setRotation(3);
      #elif BOARD_MODEL == BOARD_LORA32_V2_0
        disp_mode = DISP_MODE_PORTRAIT;
        display.setRotation(3);
      #elif BOARD_MODEL == BOARD_LORA32_V2_1
        disp_mode = DISP_MODE_LANDSCAPE;
        display.setRotation(0);
      #elif BOARD_MODEL == BOARD_TBEAM
        disp_mode = DISP_MODE_LANDSCAPE;
        display.setRotation(0);
      #elif BOARD_MODEL == BOARD_HELTEC32_V2
        disp_mode = DISP_MODE_PORTRAIT;
        display.setRotation(1);
      #elif BOARD_MODEL == BOARD_RAK4631
        #if DISPLAY == OLED
        #elif DISPLAY == EINK_BW || DISPLAY == EINK_3C
        disp_mode = DISP_MODE_PORTRAIT;
        #endif
      #elif BOARD_MODEL == BOARD_TECHO
        disp_mode = DISP_MODE_LANDSCAPE;
        display.setRotation(3);
      #elif BOARD_MODEL == BOARD_HELTEC32_V3
        disp_mode = DISP_MODE_PORTRAIT;
        // Antenna conx up
        display.setRotation(1);
        // USB-C up
        // display.setRotation(3);
      #else
        disp_mode = DISP_MODE_PORTRAIT;
        display.setRotation(3);
      #endif

      update_area_positions();

      last_page_flip = millis();

      stat_area.cp437(true);
      disp_area.cp437(true);
      display.cp437(true);

      #if HAS_EEPROM
      display_intensity = EEPROM.read(eeprom_addr(ADDR_CONF_DINT));
      #elif MCU_VARIANT == MCU_NRF52
      display_intensity = eeprom_read(eeprom_addr(ADDR_CONF_DINT));
      #endif

      return true;
    }
  #else
    return false;
  #endif
}

void draw_cable_icon(int px, int py) {
  if (cable_state == CABLE_STATE_DISCONNECTED) {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_cable+0*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_cable+0*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  } else if (cable_state == CABLE_STATE_CONNECTED) {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_cable+1*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_cable+1*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  }
}

void draw_bt_icon(int px, int py) {
  if (bt_state == BT_STATE_OFF) {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_bt+0*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_bt+0*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  } else if (bt_state == BT_STATE_ON) {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_bt+1*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_bt+1*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  } else if (bt_state == BT_STATE_PAIRING) {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_bt+2*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_bt+2*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  } else if (bt_state == BT_STATE_CONNECTED) {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_bt+3*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_bt+3*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  } else {
    #if DISP_H == 122
        stat_area.drawBitmap(px, py, bm_bt+0*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
        stat_area.drawBitmap(px, py, bm_bt+0*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  }
}

void draw_lora_icon(RadioInterface* radio, int px, int py) {
  // todo: make display show other interfaces
  if (radio_online) {
    #if DISP_H == 122
    if (online_interface_list[interface_page] != radio->getIndex()) {
      stat_area.drawBitmap(px - 2, py - 2, bm_dot_sqr, 34, 36, DISPLAY_WHITE, DISPLAY_BLACK);

      // redraw stat area on next refresh
      stat_area_initialised = false;
    }
    if (radio->getRadioOnline()) {
      stat_area.drawBitmap(px, py, bm_rf+1*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    } else {
      stat_area.drawBitmap(px, py, bm_rf+0*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    }
    #else
    if (online_interface_list[interface_page] != radio->getIndex()) {
      stat_area.drawBitmap(px - 1, py - 1, bm_dot_sqr, 18, 19, DISPLAY_WHITE, DISPLAY_BLACK);

      // redraw stat area on next refresh
      stat_area_initialised = false;
    }
    if (radio->getRadioOnline()) {
      stat_area.drawBitmap(px, py, bm_rf+1*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    } else {
      stat_area.drawBitmap(px, py, bm_rf+0*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    }
    #endif
    } else {
      #if DISP_H == 122
      stat_area.drawBitmap(px, py, bm_rf+0*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
      #else
      stat_area.drawBitmap(px, py, bm_rf+0*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
      #endif
    }
}

void draw_mw_icon(int px, int py) {
  if (INTERFACE_COUNT >= 2) {
      if (interface_obj[1]->getRadioOnline()) {
        #if DISP_H == 122
            stat_area.drawBitmap(px, py, bm_rf+3*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
        #else
            stat_area.drawBitmap(px, py, bm_rf+3*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
        #endif
      } else {
        #if DISP_H == 122
            stat_area.drawBitmap(px, py, bm_rf+2*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
        #else
            stat_area.drawBitmap(px, py, bm_rf+2*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
        #endif
      }
  } else {
    #if DISP_H == 122
      stat_area.drawBitmap(px, py, bm_rf+2*128, 30, 32, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
      stat_area.drawBitmap(px, py, bm_rf+2*32, 16, 16, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  }
}

uint8_t charge_tick = 0;
void draw_battery_bars(int px, int py) {
  if (pmu_ready) {
    if (battery_ready) {
      if (battery_installed) {
      float battery_value = battery_percent;
        
        if (battery_state == BATTERY_STATE_CHARGING) {
          battery_value = charge_tick;
          charge_tick += 3;
          if (charge_tick > 100) charge_tick = 0;
        }

        if (battery_indeterminate && battery_state == BATTERY_STATE_CHARGING) {
          #if DISP_H == 122
              stat_area.fillRect(px-2, py-2, 24, 9, DISPLAY_BLACK);
              stat_area.drawBitmap(px-2, py-5, bm_plug, 34, 13, DISPLAY_WHITE, DISPLAY_BLACK);
          #else
              stat_area.fillRect(px-2, py-2, 18, 7, DISPLAY_BLACK);
              stat_area.drawBitmap(px-2, py-2, bm_plug, 17, 7, DISPLAY_WHITE, DISPLAY_BLACK);
          #endif
        } else {
          if (battery_state == BATTERY_STATE_CHARGED) {
            #if DISP_H == 122
                stat_area.fillRect(px-2, py-2, 24, 9, DISPLAY_BLACK);
                stat_area.drawBitmap(px-2, py-5, bm_plug, 34, 13, DISPLAY_WHITE, DISPLAY_BLACK);
            #else
                stat_area.fillRect(px-2, py-2, 18, 7, DISPLAY_BLACK);
                stat_area.drawBitmap(px-2, py-2, bm_plug, 17, 7, DISPLAY_WHITE, DISPLAY_BLACK);
            #endif
          } else {
            #if DISP_H == 122
                stat_area.fillRect(px, py, 20, 5, DISPLAY_BLACK);
                stat_area.fillRect(px-2, py-4, 34, 19, DISPLAY_BLACK);
                stat_area.drawRect(px-2, py-2, 23, 9, DISPLAY_WHITE);
                stat_area.drawLine(px+21, py, px+21, py+5, DISPLAY_WHITE);
                if (battery_value > 0) stat_area.drawLine(px, py, px, py+4, DISPLAY_WHITE);
                if (battery_value >= 10) stat_area.drawLine(px+1*2, py, px+1*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 20) stat_area.drawLine(px+2*2, py, px+2*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 30) stat_area.drawLine(px+3*2, py, px+3*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 40) stat_area.drawLine(px+4*2, py, px+4*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 50) stat_area.drawLine(px+5*2, py, px+5*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 60) stat_area.drawLine(px+6*2, py, px+6*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 70) stat_area.drawLine(px+7*2, py, px+7*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 80) stat_area.drawLine(px+8*2, py, px+8*2, py+4, DISPLAY_WHITE);
                if (battery_value >= 90) stat_area.drawLine(px+9*2, py, px+9*2, py+4, DISPLAY_WHITE);
              #else
                stat_area.fillRect(px, py, 14, 3, DISPLAY_BLACK);
                stat_area.fillRect(px-2, py-2, 18, 7, DISPLAY_BLACK);
                stat_area.drawRect(px-2, py-2, 17, 7, DISPLAY_WHITE);
                stat_area.drawLine(px+15, py, px+15, py+3, DISPLAY_WHITE);
                if (battery_value > 7) stat_area.drawLine(px, py, px, py+2, DISPLAY_WHITE);
                if (battery_value > 20) stat_area.drawLine(px+1*2, py, px+1*2, py+2, DISPLAY_WHITE);
                if (battery_value > 33) stat_area.drawLine(px+2*2, py, px+2*2, py+2, DISPLAY_WHITE);
                if (battery_value > 46) stat_area.drawLine(px+3*2, py, px+3*2, py+2, DISPLAY_WHITE);
                if (battery_value > 59) stat_area.drawLine(px+4*2, py, px+4*2, py+2, DISPLAY_WHITE);
                if (battery_value > 72) stat_area.drawLine(px+5*2, py, px+5*2, py+2, DISPLAY_WHITE);
                if (battery_value > 85) stat_area.drawLine(px+6*2, py, px+6*2, py+2, DISPLAY_WHITE);
            #endif
          }
        }
      } else {
        #if DISP_H == 122
        stat_area.fillRect(px-2, py-2, 24, 9, DISPLAY_BLACK);
        stat_area.drawBitmap(px-2, py-5, bm_plug, 34, 13, DISPLAY_WHITE, DISPLAY_BLACK);
        #else
        stat_area.fillRect(px-2, py-2, 18, 7, DISPLAY_BLACK);
        stat_area.drawBitmap(px-2, py-2, bm_plug, 17, 7, DISPLAY_WHITE, DISPLAY_BLACK);
        #endif
      }
    }
  } else {
    #if DISP_H == 122
    stat_area.fillRect(px-2, py-2, 24, 9, DISPLAY_BLACK);
    stat_area.drawBitmap(px-2, py-5, bm_plug, 34, 13, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
    stat_area.fillRect(px-2, py-2, 18, 7, DISPLAY_BLACK);
    stat_area.drawBitmap(px-2, py-2, bm_plug, 17, 7, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  }
}

#define Q_SNR_STEP 2.0
#define Q_SNR_MIN_BASE -9.0
#define Q_SNR_MAX 6.0

void draw_quality_bars(int px, int py) {
  signed char t_snr = (signed int)last_snr_raw;
  int snr_int = (int)t_snr;
  float snr_min = Q_SNR_MIN_BASE-(int)interface_obj[interface_page]->getSpreadingFactor()*Q_SNR_STEP;
  float snr_span = (Q_SNR_MAX-snr_min);
  float snr = ((int)snr_int) * 0.25;
  float quality = ((snr-snr_min)/(snr_span))*100;
  if (quality > 100.0) quality = 100.0;
  if (quality < 0.0) quality = 0.0;

  #if DISP_H == 122
  stat_area.fillRect(px, py, 26, 14, DISPLAY_BLACK);
  if (quality > 0) {
      stat_area.drawLine(px+0*4, py+14, px+0*4, py+6, DISPLAY_WHITE);
      stat_area.drawLine(px+0*4+1, py+14, px+0*4+1, py+6, DISPLAY_WHITE);
  }
  if (quality > 15) {
      stat_area.drawLine(px+1*4, py+14, px+1*4, py+5, DISPLAY_WHITE);
      stat_area.drawLine(px+1*4+1, py+14, px+1*4+1, py+5, DISPLAY_WHITE);
  }
  if (quality > 30) {
      stat_area.drawLine(px+2*4, py+14, px+2*4, py+4, DISPLAY_WHITE);
      stat_area.drawLine(px+2*4+1, py+14, px+2*4+1, py+4, DISPLAY_WHITE);
  }
  if (quality > 45) {
      stat_area.drawLine(px+3*4, py+14, px+3*4, py+3, DISPLAY_WHITE);
      stat_area.drawLine(px+3*4+1, py+14, px+3*4+1, py+3, DISPLAY_WHITE);
  }
  if (quality > 60) {
      stat_area.drawLine(px+4*4, py+14, px+4*4, py+2, DISPLAY_WHITE);
      stat_area.drawLine(px+4*4+1, py+14, px+4*4+1, py+2, DISPLAY_WHITE);
  }
  if (quality > 75) {
      stat_area.drawLine(px+5*4, py+14, px+5*4, py+1, DISPLAY_WHITE);
      stat_area.drawLine(px+5*4+1, py+14, px+5*4+1, py+1, DISPLAY_WHITE);
  }
  if (quality > 90) {
      stat_area.drawLine(px+6*4, py+14, px+6*4, py+0, DISPLAY_WHITE);
      stat_area.drawLine(px+6*4+1, py+14, px+6*4+1, py+0, DISPLAY_WHITE);
  }
  #else
  stat_area.fillRect(px, py, 13, 7, DISPLAY_BLACK);
  if (quality > 0)  stat_area.drawLine(px+0*2, py+7, px+0*2, py+6, DISPLAY_WHITE);
  if (quality > 15) stat_area.drawLine(px+1*2, py+7, px+1*2, py+5, DISPLAY_WHITE);
  if (quality > 30) stat_area.drawLine(px+2*2, py+7, px+2*2, py+4, DISPLAY_WHITE);
  if (quality > 45) stat_area.drawLine(px+3*2, py+7, px+3*2, py+3, DISPLAY_WHITE);
  if (quality > 60) stat_area.drawLine(px+4*2, py+7, px+4*2, py+2, DISPLAY_WHITE);
  if (quality > 75) stat_area.drawLine(px+5*2, py+7, px+5*2, py+1, DISPLAY_WHITE);
  if (quality > 90) stat_area.drawLine(px+6*2, py+7, px+6*2, py+0, DISPLAY_WHITE);
  #endif
  // Serial.printf("Last SNR: %.2f\n, quality: %.2f\n", snr, quality);
}

#define S_RSSI_MIN -135.0
#define S_RSSI_MAX -75.0
#define S_RSSI_SPAN (S_RSSI_MAX-S_RSSI_MIN)
void draw_signal_bars(int px, int py) {
  int rssi_val = last_rssi;
  if (rssi_val < S_RSSI_MIN) rssi_val = S_RSSI_MIN;
  if (rssi_val > S_RSSI_MAX) rssi_val = S_RSSI_MAX;
  int signal = ((rssi_val - S_RSSI_MIN)*(1.0/S_RSSI_SPAN))*100.0;

  if (signal > 100.0) signal = 100.0;
  if (signal < 0.0) signal = 0.0;

  #if DISP_H == 122
  stat_area.fillRect(px, py, 26, 14, DISPLAY_BLACK);
  if (signal > 85) {
      stat_area.drawLine(px+0*4, py+14, px+0*4, py+0, DISPLAY_WHITE);
      stat_area.drawLine(px+0*4+1, py+14, px+0*4+1, py+0, DISPLAY_WHITE);
  }
  if (signal > 72) {
      stat_area.drawLine(px+1*4, py+14, px+1*4, py+1, DISPLAY_WHITE);
      stat_area.drawLine(px+1*4+1, py+14, px+1*4+1, py+1, DISPLAY_WHITE);
  }
  if (signal > 59) {
      stat_area.drawLine(px+2*4, py+14, px+2*4, py+2, DISPLAY_WHITE);
      stat_area.drawLine(px+2*4+1, py+14, px+2*4+1, py+2, DISPLAY_WHITE);
  }
  if (signal > 46) {
      stat_area.drawLine(px+3*4, py+14, px+3*4, py+3, DISPLAY_WHITE);
      stat_area.drawLine(px+3*4+1, py+14, px+3*4+1, py+3, DISPLAY_WHITE);
  }
  if (signal > 33) {
      stat_area.drawLine(px+4*4, py+14, px+4*4, py+4, DISPLAY_WHITE);
      stat_area.drawLine(px+4*4+1, py+14, px+4*4+1, py+4, DISPLAY_WHITE);
  }
  if (signal > 20) {
      stat_area.drawLine(px+5*4, py+14, px+5*4, py+5, DISPLAY_WHITE);
      stat_area.drawLine(px+5*4+1, py+14, px+5*4+1, py+5, DISPLAY_WHITE);
  }
  if (signal > 7)  {
      stat_area.drawLine(px+6*4, py+14, px+6*4, py+6, DISPLAY_WHITE);
      stat_area.drawLine(px+6*4+1, py+14, px+6*4+1, py+6, DISPLAY_WHITE);
  }
  #else
  stat_area.fillRect(px, py, 13, 7, DISPLAY_BLACK);
  if (signal > 85) stat_area.drawLine(px+0*2, py+7, px+0*2, py+0, DISPLAY_WHITE);
  if (signal > 72) stat_area.drawLine(px+1*2, py+7, px+1*2, py+1, DISPLAY_WHITE);
  if (signal > 59) stat_area.drawLine(px+2*2, py+7, px+2*2, py+2, DISPLAY_WHITE);
  if (signal > 46) stat_area.drawLine(px+3*2, py+7, px+3*2, py+3, DISPLAY_WHITE);
  if (signal > 33) stat_area.drawLine(px+4*2, py+7, px+4*2, py+4, DISPLAY_WHITE);
  if (signal > 20) stat_area.drawLine(px+5*2, py+7, px+5*2, py+5, DISPLAY_WHITE);
  if (signal > 7)  stat_area.drawLine(px+6*2, py+7, px+6*2, py+6, DISPLAY_WHITE);
  #endif
  // Serial.printf("Last SNR: %.2f\n, quality: %.2f\n", snr, quality);
}

#define WF_RSSI_MAX -60
#define WF_RSSI_MIN -135
#define WF_RSSI_SPAN (WF_RSSI_MAX - WF_RSSI_MIN)
#if DISP_H == 122
#define WF_PIXEL_WIDTH 22
#elif disp_mode == DISP_MODE_LANDSCAPE
#define WF_PIXEL_WIDTH (DISP_H / WF_RSSI_SPAN)
#else
#define WF_PIXEL_WIDTH (DISP_W / WF_RSSI_SPAN)
#endif
void draw_waterfall(int px, int py) {
  int rssi_val = interface_obj[interface_page]->currentRssi();
  if (rssi_val < WF_RSSI_MIN) rssi_val = WF_RSSI_MIN;
  if (rssi_val > WF_RSSI_MAX) rssi_val = WF_RSSI_MAX;
  int rssi_normalised = ((rssi_val - WF_RSSI_MIN)*(1.0/WF_RSSI_SPAN))*WF_PIXEL_WIDTH;

  waterfall[interface_page][waterfall_head[interface_page]++] = rssi_normalised;
  if (waterfall_head[interface_page] >= WATERFALL_SIZE) waterfall_head[interface_page] = 0;

  stat_area.fillRect(px,py,WF_PIXEL_WIDTH, WATERFALL_SIZE, DISPLAY_BLACK);
  for (int i = 0; i < WATERFALL_SIZE; i++){
    int wi = (waterfall_head[interface_page]+i)%WATERFALL_SIZE;
    int ws = waterfall[interface_page][wi];
    if (ws > 0) {
      stat_area.drawLine(px, py+i, px+ws-1, py+i, DISPLAY_WHITE);
    }
  }
}

void draw_stat_area() {
  if (device_init_done) {
    if (!stat_area_initialised) {
      #if DISP_H == 122
        stat_area.drawBitmap(0, 0, bm_frame, stat_area.width(), stat_area.height(), DISPLAY_WHITE, DISPLAY_BLACK);
      #else
        stat_area.drawBitmap(0, 0, bm_frame, 64, 64, DISPLAY_WHITE, DISPLAY_BLACK);
      #endif
      stat_area_initialised = true;
    }

    if (millis()-last_interface_page_flip >= page_interval) {
      int online_interfaces_check = 0;

      // todo, is there a more efficient way of doing this?
      for (int i = 0; i < INTERFACE_COUNT; i++) {
          if (interface_obj[i]->getRadioOnline()) {
              online_interfaces_check++;
          }
      }

      if (online_interfaces != online_interfaces_check) {
          online_interfaces = online_interfaces_check;
      }

      // cap at two for now, as only two boxes to symbolise interfaces
      // available on display
      if (online_interfaces > 2) {
          online_interfaces = 2;
      }

      uint8_t index = 0;

      for (int i = 0; i < INTERFACE_COUNT; i++) {
          if (interface_obj[i]->getRadioOnline()) {
              online_interface_list[index] = i;
              index++;
          }
      }

      if (online_interfaces > 0) {
          interface_page = (++interface_page%online_interfaces);
      }
      last_interface_page_flip = millis();
    }

    #if DISP_H == 122
    draw_cable_icon(6, 18);
    draw_bt_icon(6, 60);
    draw_lora_icon(interface_obj[0], 86, 18);

    // todo, expand support to show more than two interfaces on screen
    if (INTERFACE_COUNT > 1) {
        draw_lora_icon(interface_obj[1], 86, 60);
    }
    draw_battery_bars(8, 113);
    #else
    draw_cable_icon(3, 8);
    draw_bt_icon(3, 30);
    draw_lora_icon(interface_obj[0], 45, 8);

    // todo, expand support to show more than two interfaces on screen
    if (INTERFACE_COUNT > 1) {
        draw_lora_icon(interface_obj[1], 45, 30);
    }
    draw_battery_bars(4, 58);
    #endif
    radio_online = false;
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        if (interface_obj[i]->getRadioOnline()) {
            radio_online = true;
            break;
        }
    }
    if (radio_online) {
      #if DISP_H == 122
      draw_quality_bars(53, 109);
      draw_signal_bars(83, 109);
      draw_waterfall(50, 8);
      #else
      draw_quality_bars(28, 56);
      draw_signal_bars(44, 56);
      draw_waterfall(27, 4);
      #endif
    }
  }
}

void update_stat_area() {
  if (eeprom_ok && !firmware_update_mode && !console_active) {
    draw_stat_area();
    if (disp_mode == DISP_MODE_PORTRAIT) {
      display.drawBitmap(p_as_x, p_as_y, stat_area.getBuffer(), stat_area.width(), stat_area.height(), DISPLAY_WHITE, DISPLAY_BLACK);
    } else if (disp_mode == DISP_MODE_LANDSCAPE) {
      display.drawBitmap(p_as_x+2, p_as_y, stat_area.getBuffer(), stat_area.width(), stat_area.height(), DISPLAY_WHITE, DISPLAY_BLACK);
      if (device_init_done && !disp_ext_fb) display.drawLine(p_as_x, 0, p_as_x, DISP_W/2, DISPLAY_WHITE);
    }

  } else {
    if (firmware_update_mode) {
      display.drawBitmap(p_as_x, p_as_y, bm_updating, stat_area.width(), stat_area.height(), DISPLAY_BLACK, DISPLAY_WHITE);
    } else if (console_active && device_init_done) {
      display.drawBitmap(p_as_x, p_as_y, bm_console, stat_area.width(), stat_area.height(), DISPLAY_BLACK, DISPLAY_WHITE);
      if (disp_mode == DISP_MODE_LANDSCAPE) {
        //#if DISP_H == 122
        display.drawLine(p_as_x, 0, p_as_x, DISP_W/2, DISPLAY_WHITE);
        //#else
        //display.drawLine(p_as_x, 0, p_as_x, 64, DISPLAY_WHITE);
        //#endif
      }
    }
  }
}

void draw_disp_area() {
  if (!device_init_done || firmware_update_mode) {
    uint8_t p_by = 37;
    if (disp_mode == DISP_MODE_LANDSCAPE || firmware_update_mode) {
      p_by = 18;
      disp_area.fillRect(0, 0, disp_area.width(), disp_area.height(), DISPLAY_BLACK);
    }
    #if DISP_H == 122
    if (!device_init_done) disp_area.drawBitmap(0, p_by, bm_boot, disp_area.width(), 54, DISPLAY_WHITE);
    if (firmware_update_mode) disp_area.drawBitmap(0, p_by, bm_fw_update, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
    #else
    if (!device_init_done) disp_area.drawBitmap(0, p_by, bm_boot, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
    if (firmware_update_mode) disp_area.drawBitmap(0, p_by, bm_fw_update, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
    #endif
  } else {
    if (!disp_ext_fb or bt_ssp_pin != 0) {
      if (radio_online && display_diagnostics) {
        #if DISP_H == 122
          selected_radio = interface_obj[online_interface_list[interface_page]];
          disp_area.fillRect(0,12,disp_area.width(),57, DISPLAY_BLACK); disp_area.fillRect(0,69,disp_area.width(),56, DISPLAY_WHITE);
          disp_area.setFont(SMALL_FONT); disp_area.setTextWrap(false); disp_area.setTextColor(DISPLAY_WHITE);
          disp_area.setTextSize(2); // scale text 2x

          disp_area.setCursor(2, 22);
          disp_area.print("On");
          disp_area.setCursor(14*2, 22);
          disp_area.print("@");
          disp_area.setCursor(21*2, 22);
          disp_area.printf("%.1fKbps", (float)(selected_radio->getBitrate())/1000.0);

          disp_area.setCursor(2, 36);
          disp_area.print("Airtime:");

          disp_area.setCursor(7+12, 53);
          if (selected_radio->getTotalChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getAirtime()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getAirtime()*100.0);
          }

          disp_area.drawBitmap(2, 41, bm_hg_low, 10, 18, DISPLAY_WHITE, DISPLAY_BLACK);

          disp_area.setCursor(64+17, 53);
          if (selected_radio->getLongtermChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getLongtermAirtime()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getLongtermAirtime()*100.0);
          }
          disp_area.drawBitmap(64, 41, bm_hg_high, 10, 18, DISPLAY_WHITE, DISPLAY_BLACK);

          disp_area.setTextColor(DISPLAY_BLACK);

          disp_area.setCursor(2, 88);
          disp_area.print("Channel");
          disp_area.setCursor(38*2, 88);
          disp_area.print("Load:");
        
          disp_area.setCursor(7+12, 110);
          if (selected_radio->getTotalChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getTotalChannelUtil()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getTotalChannelUtil()*100.0);
          }
          disp_area.drawBitmap(2, 98, bm_hg_low, 10, 18, DISPLAY_BLACK, DISPLAY_WHITE);

          disp_area.setCursor(64+17, 110);
          if (selected_radio->getLongtermChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getLongtermChannelUtil()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getLongtermChannelUtil()*100.0);
          }
          disp_area.drawBitmap(64, 98, bm_hg_high, 10, 18, DISPLAY_BLACK, DISPLAY_WHITE);
        #else
          selected_radio = interface_obj[online_interface_list[interface_page]];
          disp_area.fillRect(0,8,disp_area.width(),37, DISPLAY_BLACK); disp_area.fillRect(0,37,disp_area.width(),27, DISPLAY_WHITE);
          disp_area.setFont(SMALL_FONT); disp_area.setTextWrap(false); disp_area.setTextColor(DISPLAY_WHITE);

          disp_area.setCursor(2, 13);
          disp_area.print("On");
          disp_area.setCursor(14, 13);
          disp_area.print("@");
          disp_area.setCursor(21, 13);
          disp_area.printf("%.1fKbps", (float)(selected_radio->getBitrate())/1000.0);

          disp_area.setCursor(2, 23-1);
          disp_area.print("Airtime:");

          disp_area.setCursor(11, 33-1);
          if (selected_radio->getTotalChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getAirtime()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getAirtime()*100.0);
          }

          disp_area.drawBitmap(2, 26-1, bm_hg_low, 5, 9, DISPLAY_WHITE, DISPLAY_BLACK);

          disp_area.setCursor(32+11, 33-1);

          if (selected_radio->getLongtermChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getLongtermAirtime()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getLongtermAirtime()*100.0);
          }
          disp_area.drawBitmap(32+2, 26-1, bm_hg_high, 5, 9, DISPLAY_WHITE, DISPLAY_BLACK);

          disp_area.setTextColor(DISPLAY_BLACK);

          disp_area.setCursor(2, 46);
          disp_area.print("Channel");
          disp_area.setCursor(38, 46);
          disp_area.print("Load:");

          disp_area.setCursor(11, 57);
          if (selected_radio->getTotalChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getTotalChannelUtil()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getTotalChannelUtil()*100.0);
          }
          disp_area.drawBitmap(2, 50, bm_hg_low, 5, 9, DISPLAY_BLACK, DISPLAY_WHITE);
        
          disp_area.setCursor(32+11, 57);
          if (selected_radio->getLongtermChannelUtil() < 0.099) {
            disp_area.printf("%.1f%%", selected_radio->getLongtermChannelUtil()*100.0);
          } else {
            disp_area.printf("%.0f%%", selected_radio->getLongtermChannelUtil()*100.0);
          }
          disp_area.drawBitmap(32+2, 50, bm_hg_high, 5, 9, DISPLAY_BLACK, DISPLAY_WHITE);
        #endif

      } else {
        if (device_signatures_ok()) {
          #if DISP_H == 122
            disp_area.drawBitmap(0, 0, bm_def_lc, disp_area.width(), 71, DISPLAY_WHITE, DISPLAY_BLACK);
          #else
            disp_area.drawBitmap(0, 0, bm_def_lc, disp_area.width(), 37, DISPLAY_WHITE, DISPLAY_BLACK);
          #endif
        } else {
          #if DISP_H == 122
            disp_area.drawBitmap(0, 0, bm_def, disp_area.width(), 71, DISPLAY_WHITE, DISPLAY_BLACK);
          #else
            disp_area.drawBitmap(0, 0, bm_def, disp_area.width(), 37, DISPLAY_WHITE, DISPLAY_BLACK);
          #endif
        }
      }

      if (!hw_ready || !device_firmware_ok()) {
        if (!device_firmware_ok()) {
          #if DISP_H == 122
            disp_area.drawBitmap(0, 71, bm_fw_corrupt, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
          #else
            disp_area.drawBitmap(0, 37, bm_fw_corrupt, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
          #endif
        } else {
          if (!modems_installed) {
            #if DISP_H == 122
              disp_area.drawBitmap(0, 71, bm_no_radio, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
            #else
              disp_area.drawBitmap(0, 37, bm_no_radio, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
            #endif
          } else {
            #if DISP_H == 122
              disp_area.drawBitmap(0, 71, bm_hwfail, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
            #else
              disp_area.drawBitmap(0, 37, bm_hwfail, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
            #endif
          }
        }
      } else if (bt_state == BT_STATE_PAIRING and bt_ssp_pin != 0) {
        char *pin_str = (char*)malloc(DISP_PIN_SIZE+1);
        sprintf(pin_str, "%06d", bt_ssp_pin);

        #if DISP_H == 122
          disp_area.drawBitmap(0, 71, bm_pairing, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
        #else
          disp_area.drawBitmap(0, 37, bm_pairing, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
        #endif
        for (int i = 0; i < DISP_PIN_SIZE; i++) {
          uint8_t numeric = pin_str[i]-48;
          #if DISP_H == 122
            uint8_t offset = numeric*20;
            disp_area.drawBitmap(14+17*i, 71+32, bm_n_uh+offset, 10, 10, DISPLAY_WHITE, DISPLAY_BLACK);
          #else
            uint8_t offset = numeric*5;
            disp_area.drawBitmap(7+9*i, 37+16, bm_n_uh+offset, 8, 5, DISPLAY_WHITE, DISPLAY_BLACK);
          #endif
        }
        free(pin_str);
      } else {
        if (millis()-last_page_flip >= page_interval) {
          disp_page = (++disp_page%pages);
          last_page_flip = millis();
          if (not community_fw and disp_page == 0) disp_page = 1;
        }

        if (radio_online) {
          if (!display_diagnostics) {
            #if DISP_H == 122
              disp_area.drawBitmap(0, 37, bm_online, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
            #else
              disp_area.drawBitmap(0, 71, bm_online, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
            #endif
          }
        } else {
          if (disp_page == 0) {
            if (true || device_signatures_ok()) {
              #if DISP_H == 122
                disp_area.drawBitmap(0, 71, bm_checks, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
              #else
                disp_area.drawBitmap(0, 37, bm_checks, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
              #endif
            } else {
              #if DISP_H == 122
                disp_area.drawBitmap(0, 71, bm_nfr, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
              #else
                disp_area.drawBitmap(0, 37, bm_nfr, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
              #endif
            }
          } else if (disp_page == 1) {
            if (!console_active) {
              #if DISP_H == 122
                disp_area.drawBitmap(0, 71, bm_hwok, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
              #else
                disp_area.drawBitmap(0, 37, bm_hwok, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
              #endif
            } else {
              #if DISP_H == 122
                disp_area.drawBitmap(0, 71, bm_console_active, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
              #else
                disp_area.drawBitmap(0, 37, bm_console_active, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
              #endif
            }
          } else if (disp_page == 2) {
            #if DISP_H == 122
              disp_area.drawBitmap(0, 71, bm_version, disp_area.width(), 54, DISPLAY_WHITE, DISPLAY_BLACK);
            #else
              disp_area.drawBitmap(0, 37, bm_version, disp_area.width(), 27, DISPLAY_WHITE, DISPLAY_BLACK);
            #endif
            char *v_str = (char*)malloc(3+1);
            sprintf(v_str, "%01d%02d", MAJ_VERS, MIN_VERS);
            for (int i = 0; i < 3; i++) {
              #if DISP_H == 122
              uint8_t dxp = 43;
              uint8_t numeric = v_str[i]-48; uint8_t bm_offset = numeric*20;
              if (i == 2) dxp += 9*2+6;
              #else
              uint8_t dxp = DISP_H * 0.32;
              uint8_t numeric = v_str[i]-48; uint8_t bm_offset = numeric*20;
              if (i == 2) dxp += 9*2+6;
              #endif
              if (i == 1) dxp += 9*1+4;
              #if DISP_H == 122
              // add gap manually rather than oversizing bitmap, as the gfx lib fills in the extra space with black
              disp_area.drawBitmap(dxp, 71+32, bm_n_uh+bm_offset, 10, 10, DISPLAY_WHITE, DISPLAY_BLACK);
              #else
              disp_area.drawBitmap(dxp, 37+16, bm_n_uh+bm_offset, 8, 5, DISPLAY_WHITE, DISPLAY_BLACK);
              #endif
            }
            free(v_str);
            #if DISP_H == 122
            disp_area.drawLine(27, 37+20, 28, 37+20, DISPLAY_BLACK);
            disp_area.drawLine(27, 37+20, 28, 37+20, DISPLAY_BLACK);
            #else
            disp_area.drawLine(27, 37+19, 28, 37+19, DISPLAY_BLACK);
            disp_area.drawLine(27, 37+19, 28, 37+19, DISPLAY_BLACK);
            #endif
          }
        }
      }
    } else {
        disp_area.drawBitmap(0, 0, fb, disp_area.width(), disp_area.height(), DISPLAY_WHITE, DISPLAY_BLACK);
    }
  }
}

void update_disp_area() {
  draw_disp_area();
  display.drawBitmap(p_ad_x, p_ad_y, disp_area.getBuffer(), disp_area.width(), disp_area.height(), DISPLAY_WHITE, DISPLAY_BLACK);
  if (disp_mode == DISP_MODE_LANDSCAPE) {
    if (device_init_done && !firmware_update_mode && !disp_ext_fb) {
      #if DISP_H == 122
      display.drawLine(0, 0, 0, 63, DISPLAY_WHITE);
      #else
      display.drawLine(0, 0, 0, DISP_H / 2, DISPLAY_WHITE);
      #endif
    }
  }
}

void do_screensaver(uint32_t current){
  #if DISPLAY == OLED
    // Invert display to protect against OLED screen burn in
    //TODO: Make this a option configurable through rnodeconf
    //TODO: Implement other ways to do the screensaver, such as scrolling the screen.
    if (screensaver_enabled) {
      if (current-last_screensaver >= SCREENSAVER_INTERVAL+SCREENSAVER_TIME) {
          display.invertDisplay(0);
          last_screensaver = current;
          screensaver_enabled = false;
      }
    }
    else if (current-last_screensaver >= SCREENSAVER_INTERVAL) {
      display.invertDisplay(1);
      screensaver_enabled = true;
    }
  #endif
}

void update_display(bool blank = false) {
  #if DISPLAY == OLED
    if (display_contrast != display_intensity) {
      display_contrast = display_intensity;
      set_contrast(&display, display_contrast);
    }
    display.clearDisplay();
  #endif
  if (blank) {
    display.display();    
  } else {
    if (millis()-last_disp_update >= disp_update_interval) {
      uint32_t current = millis();
      #if screensaver_enabled
        do_screensaver(current);
      #endif
      #if DISPLAY == EINK_BW || DISPLAY == EINK_3C
      display.setFullWindow();
      display.fillScreen(DISPLAY_WHITE);
      #endif
      update_stat_area();
      update_disp_area();
      #if DISPLAY == OLED
      display.display();
      #else
      if (current-last_epd_refresh >= REFRESH_PERIOD) {
        // Perform a full refresh after the correct time has elapsed
        display.display(false);
        last_epd_refresh = current;
      } else {
        // Only perform a partial refresh
        display.display(true);
      }
      #endif
      last_disp_update = current;
    }
  }
}

void ext_fb_enable() {
  disp_ext_fb = true;
}

void ext_fb_disable() {
  disp_ext_fb = false;
}
