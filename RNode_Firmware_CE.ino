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

#include <Arduino.h>
#include <SPI.h>
#include "Utilities.h"

#if MCU_VARIANT == MCU_NRF52
  #if BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_OPENCOM_XL
      #define INTERFACE_SPI
        // Required because on RAK4631, non-default SPI pins must be initialised when class is declared.
      SPIClass interface_spi[1] = {
            // SX1262
            SPIClass(
                NRF_SPIM2, 
                interface_pins[0][3], 
                interface_pins[0][1], 
                interface_pins[0][2]
               )
      };
  #elif BOARD_MODEL == BOARD_TECHO
    #define INTERFACE_SPI
    SPIClass interface_spi[1] = {
            // SX1262
            SPIClass(
                NRF_SPIM3, 
                interface_pins[0][3], 
                interface_pins[0][1], 
                interface_pins[0][2]
               )
      };
  #elif BOARD_MODEL == BOARD_HELTEC_T114
    #define INTERFACE_SPI
    SPIClass interface_spi[1] = {
            // SX1262
            SPIClass(
                NRF_SPIM1, 
                interface_pins[0][3], 
                interface_pins[0][1], 
                interface_pins[0][2]
               )
      };
  #endif
#endif

#ifndef INTERFACE_SPI
// INTERFACE_SPI is only required on NRF52 platforms, as the SPI pins are set in the class constructor and not by a setter method.
// Even if custom SPI interfaces are not needed, the array must exist to prevent compilation errors.
#define INTERFACE_SPI
SPIClass interface_spi[1];
#endif

FIFOBuffer serialFIFO;
uint8_t serialBuffer[CONFIG_UART_BUFFER_SIZE+1];

uint16_t packet_starts_buf[(CONFIG_QUEUE_MAX_LENGTH)+1];

uint16_t packet_lengths_buf[(CONFIG_QUEUE_MAX_LENGTH)+1];

FIFOBuffer16 packet_starts[INTERFACE_COUNT];
FIFOBuffer16 packet_lengths[INTERFACE_COUNT];

volatile uint8_t queue_height[INTERFACE_COUNT] = {0};
volatile uint16_t queued_bytes[INTERFACE_COUNT] = {0};

volatile uint16_t queue_cursor[INTERFACE_COUNT] = {0};
volatile uint16_t current_packet_start[INTERFACE_COUNT] = {0};
volatile bool serial_buffering = false;

extern void setup_interfaces(); // from /src/misc/ModemISR.h

#if HAS_BLUETOOTH || HAS_BLE == true
  bool bt_init_ran = false;
#endif

#if HAS_CONSOLE
  #include "Console.h"
#endif

#define MODEM_QUEUE_SIZE 4*INTERFACE_COUNT
typedef struct {
      size_t len;
      int rssi;
      int snr_raw;
      uint8_t interface;
      uint8_t data[];
} modem_packet_t;
static xQueueHandle modem_packet_queue = NULL;

char sbuf[128];

uint8_t *packet_queue[INTERFACE_COUNT];

void setup() {
  #if MCU_VARIANT == MCU_ESP32
    boot_seq();
    EEPROM.begin(EEPROM_SIZE);
    Serial.setRxBufferSize(CONFIG_UART_BUFFER_SIZE);

    #if BOARD_MODEL == BOARD_TDECK
      pinMode(pin_poweron, OUTPUT);
      digitalWrite(pin_poweron, HIGH);

      pinMode(SD_CS, OUTPUT);
      pinMode(DISPLAY_CS, OUTPUT);
      digitalWrite(SD_CS, HIGH);
      digitalWrite(DISPLAY_CS, HIGH);

      pinMode(DISPLAY_BL_PIN, OUTPUT);
    #endif

  #elif MCU_VARIANT == MCU_NRF52
    #if BOARD_MODEL == BOARD_TECHO
      delay(200);
      pinMode(PIN_VEXT_EN, OUTPUT);
      digitalWrite(PIN_VEXT_EN, HIGH);
      pinMode(pin_btn_usr1, INPUT_PULLUP);
      pinMode(pin_btn_touch, INPUT_PULLUP);
      pinMode(PIN_LED_RED, OUTPUT);
      pinMode(PIN_LED_GREEN, OUTPUT);
      pinMode(PIN_LED_BLUE, OUTPUT);
      delay(200);
    #elif BOARD_MODEL == BOARD_HELTEC_T114
      delay(200);
      pinMode(PIN_VEXT_EN, OUTPUT);
      digitalWrite(PIN_VEXT_EN, HIGH);
      delay(100);
    #endif


    if (!eeprom_begin()) { Serial.write("EEPROM initialisation failed.\r\n"); }
  #endif

  // Seed the PRNG for CSMA R-value selection
  #if MCU_VARIANT == MCU_ESP32
    // On ESP32, get the seed value from the
    // hardware RNG
    unsigned long seed_val = (unsigned long)esp_random();
  #elif MCU_VARIANT == MCU_NRF52
    // On nRF, get the seed value from the
    // hardware RNG
    unsigned long seed_val = get_rng_seed();
  #else
    // Otherwise, get a pseudo-random seed
    // value from an unconnected analog pin
    //
    // CAUTION! If you are implementing the
    // firmware on a platform that does not
    // have a hardware RNG, you MUST take
    // care to get a seed value with enough
    // entropy at each device reset!
    unsigned long seed_val = analogRead(0);
  #endif
  randomSeed(seed_val);

  // Initialise serial communication
  memset(serialBuffer, 0, sizeof(serialBuffer));
  fifo_init(&serialFIFO, serialBuffer, CONFIG_UART_BUFFER_SIZE);

  Serial.begin(serial_baudrate);

  #if HAS_NP
    led_init();
  #endif

  #if MCU_VARIANT == MCU_NRF52 && HAS_NP == true
    boot_seq();
  #endif

  #if BOARD_MODEL != BOARD_RAK4631 && BOARD_MODEL != BOARD_HELTEC_T114 && BOARD_MODEL != BOARD_TECHO && BOARD_MODEL != BOARD_T3S3 && BOARD_MODEL != BOARD_TBEAM_S_V1 && BOARD_MODEL != BOARD_OPENCOM_XL
  // Some boards need to wait until the hardware UART is set up before booting
  // the full firmware. In the case of the RAK4631/TECHO, the line below will wait
  // until a serial connection is actually established with a master. Thus, it
  // is disabled on this platform.
    while (!Serial);
  #endif

  // Configure input and output pins
  #if HAS_INPUT
    input_init();
  #endif

  #if HAS_NP == false
    pinMode(pin_led_rx, OUTPUT);
    pinMode(pin_led_tx, OUTPUT);
  #endif

  for (int i = 0; i < INTERFACE_COUNT; i++) {
    if (interface_pins[i][9] != -1) {
        pinMode(interface_pins[i][9], OUTPUT);
        digitalWrite(interface_pins[i][9], HIGH);
    }
  }

  // Initialise buffers
  memset(pbuf, 0, sizeof(pbuf));
  memset(cmdbuf, 0, sizeof(cmdbuf));
  
  memset(packet_starts_buf, 0, sizeof(packet_starts_buf));
  memset(packet_lengths_buf, 0, sizeof(packet_starts_buf));

  memset(seq, 0xFF, sizeof(seq));
  memset(read_len, 0, sizeof(read_len));

  setup_interfaces();

  modem_packet_queue = xQueueCreate(MODEM_QUEUE_SIZE, sizeof(modem_packet_t*));

  for (int i = 0; i < INTERFACE_COUNT; i++) {
      fifo16_init(&packet_starts[i], packet_starts_buf, CONFIG_QUEUE_MAX_LENGTH);
      fifo16_init(&packet_lengths[i], packet_lengths_buf, CONFIG_QUEUE_MAX_LENGTH);
      packet_queue[i] = (uint8_t*)malloc(getQueueSize(i)+1);
  }

  memset(packet_rdy_interfaces_buf, 0, sizeof(packet_rdy_interfaces_buf));

  fifo_init(&packet_rdy_interfaces, packet_rdy_interfaces_buf, MAX_INTERFACES);

  #if HAS_GPS
  // init GPS
  gps_s.begin(GPS_BAUD_RATE);
  #endif

  // add call to init_channel_stats here? \todo

  // Create and configure interface objects
  for (uint8_t i = 0; i < INTERFACE_COUNT; i++) {
      switch (interfaces[i]) {
          case SX1262:
          {
              sx126x* obj;
              // if default spi enabled
              if (interface_cfg[i][0]) {
                obj = new sx126x(i, &SPI, interface_cfg[i][1],
                interface_cfg[i][2], interface_pins[i][0], interface_pins[i][1],
                interface_pins[i][2], interface_pins[i][3], interface_pins[i][6],
                interface_pins[i][5], interface_pins[i][4], interface_pins[i][8]);
              }
              else {
            obj = new sx126x(i, &interface_spi[i], interface_cfg[i][1],
            interface_cfg[i][2], interface_pins[i][0], interface_pins[i][1],
            interface_pins[i][2], interface_pins[i][3], interface_pins[i][6],
            interface_pins[i][5], interface_pins[i][4], interface_pins[i][8]);
              }
            interface_obj[i] = obj;
            interface_obj_sorted[i] = obj;
            break;
          }

          case SX1276:
          case SX1278:
          {
              sx127x* obj;
              // if default spi enabled
              if (interface_cfg[i][0]) {
            obj = new sx127x(i, &SPI, interface_pins[i][0],
            interface_pins[i][1], interface_pins[i][2], interface_pins[i][3],
            interface_pins[i][6], interface_pins[i][5], interface_pins[i][4]);
              }
              else {
            obj = new sx127x(i, &interface_spi[i], interface_pins[i][0],
            interface_pins[i][1], interface_pins[i][2], interface_pins[i][3],
            interface_pins[i][6], interface_pins[i][5], interface_pins[i][4]);
              }
            interface_obj[i] = obj;
            interface_obj_sorted[i] = obj;
            break;
          }

          case SX1280:
          {
              sx128x* obj;
              // if default spi enabled
            if (interface_cfg[i][0]) {
                obj = new sx128x(i, &SPI, interface_cfg[i][1],
                interface_pins[i][0], interface_pins[i][1], interface_pins[i][2],
                interface_pins[i][3], interface_pins[i][6], interface_pins[i][5],
                interface_pins[i][4], interface_pins[i][8], interface_pins[i][7]);
            }
            else {
                obj = new sx128x(i, &interface_spi[i], interface_cfg[i][1],
                interface_pins[i][0], interface_pins[i][1], interface_pins[i][2],
                interface_pins[i][3], interface_pins[i][6], interface_pins[i][5],
                interface_pins[i][4], interface_pins[i][8], interface_pins[i][7]);
            }
            interface_obj[i] = obj;
            interface_obj_sorted[i] = obj;
            break;
          }
          
          default:
            break;
      }
  }

  #if BOARD_MODEL == BOARD_T3S3 && BOARD_VARIANT == MODEL_AC
    // Fix weird radio not found bug on T3S3 SX1280
    delay(300);
    interface_obj[0]->reset();
    delay(100);
  #endif

    // Check installed transceiver chip(s) and probe boot parameters. If any of
    // the configured modems cannot be initialised, do not boot
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        switch (interfaces[i]) {
            case SX1262:
            case SX1276:
            case SX1278:
            case SX1280:
                selected_radio = interface_obj[i];
                break;

            default:
                modems_installed = false;
                break;
        }
        if (selected_radio->preInit()) {
          modems_installed = true;
          #if HAS_INPUT
            // Skip quick-reset console activation
          #else
              uint32_t lfr = selected_radio->getFrequency();
              if (lfr == 0) {
                // Normal boot
              } else if (lfr == M_FRQ_R) {
                // Quick reboot
                #if HAS_CONSOLE
                  if (rtc_get_reset_reason(0) == POWERON_RESET) {
                    console_active = true;
                  }
                #endif
              } else {
                // Unknown boot
              }
              selected_radio->setFrequency(M_FRQ_S);
          #endif
        } else {
          modems_installed = false;
        }
        if (!modems_installed) {
            break;
        }
    }

  #if HAS_DISPLAY
    #if HAS_EEPROM
    if (EEPROM.read(eeprom_addr(ADDR_CONF_DSET)) != CONF_OK_BYTE) {
    #elif MCU_VARIANT == MCU_NRF52
    if (eeprom_read(eeprom_addr(ADDR_CONF_DSET)) != CONF_OK_BYTE) {
    #endif
      eeprom_update(eeprom_addr(ADDR_CONF_DSET), CONF_OK_BYTE);
      #if BOARD_MODEL == BOARD_TECHO
        eeprom_update(eeprom_addr(ADDR_CONF_DINT), 0x03);
      #else
        eeprom_update(eeprom_addr(ADDR_CONF_DINT), 0xFF);
      #endif
    }
    #if BOARD_MODEL == BOARD_OPENCOM_XL && (DISPLAY == EINK_BW || DISPLAY == EINK_3C)
      // On this board it isn't possible to run the main loop whilst the
      // display is updating as the SPI pins are shared between the display and
      // secondary modem. Because running the main loop causes a lockup, we
      // just run the serial poll loop instead.
      display_add_callback(process_serial);
    #elif DISPLAY == EINK_BW || DISPLAY == EINK_3C
      display_add_callback(process_serial); // todo: get this working with work_while_waiting again
    #endif

    display_unblank();
    disp_ready = display_init();
    if (disp_ready) update_display();
  #endif

    #if HAS_PMU == true
      pmu_ready = init_pmu();
    #endif

    #if HAS_BLUETOOTH || HAS_BLE == true
      bt_init();
      bt_init_ran = true;
    #endif

    if (console_active) {
      #if HAS_CONSOLE
        console_start();
      #else
        kiss_indicate_reset();
      #endif
    } else {
      kiss_indicate_reset();
    }

    for (int i = 0; i < INTERFACE_COUNT; i++) {
        selected_radio = interface_obj[i];
        if (interfaces[i] == SX1280) {
            selected_radio->setAvdInterference(false);
        }
        if (selected_radio->getAvdInterference()) {
          #if HAS_EEPROM
            uint8_t ia_conf = EEPROM.read(eeprom_addr(ADDR_CONF_DIA));
            if (ia_conf == 0x00) { selected_radio->setAvdInterference(true); }
            else                 { selected_radio->setAvdInterference(false); }
          #elif MCU_VARIANT == MCU_NRF52
            uint8_t ia_conf = eeprom_read(eeprom_addr(ADDR_CONF_DIA));
            if (ia_conf == 0x00) { selected_radio->setAvdInterference(true); }
            else                 { selected_radio->setAvdInterference(false); }
          #endif
        }
    }


  // Validate board health, EEPROM and config
  validate_status();
}

void lora_receive(RadioInterface* radio) {
  if (!implicit) {
    radio->receive();
  } else {
    radio->receive(implicit_l);
  }
}

inline void kiss_write_packet(int index) {
  // Print index of interface the packet came from
  serial_write(FEND);
  serial_write(CMD_SEL_INT);
  serial_write(index);
  serial_write(FEND);

  serial_write(FEND);
  serial_write(CMD_DATA);

  for (uint16_t i = 0; i < read_len[index]; i++) {
    #if MCU_VARIANT == MCU_NRF52
      portENTER_CRITICAL();
      uint8_t byte = pbuf[i];
      portEXIT_CRITICAL();
    #else
      uint8_t byte = pbuf[i];
    #endif

    if (byte == FEND) { serial_write(FESC); byte = TFEND; }
    if (byte == FESC) { serial_write(FESC); byte = TFESC; }
    serial_write(byte);
  }

  serial_write(FEND);
  read_len[index] = 0;

  #if MCU_VARIANT == MCU_ESP32 && HAS_BLE
      bt_flush();
  #endif
}

inline void getPacketData(RadioInterface* radio, uint16_t len) {
    uint8_t index = radio->getIndex();
  #if MCU_VARIANT != MCU_NRF52
    while (len-- && read_len[index] < MTU) {
      pbuf[read_len[index]++] = radio->read();
    }  
  #else
    BaseType_t int_mask = taskENTER_CRITICAL_FROM_ISR();
    while (len-- && read_len[index] < MTU) {
      pbuf[read_len[index]++] = radio->read();
    }
    taskEXIT_CRITICAL_FROM_ISR(int_mask);
  #endif
}

inline bool queue_packet(RadioInterface* radio, uint8_t index) {
    // Allocate packet struct, but abort if there
    // is not enough memory available.
    modem_packet_t *modem_packet = (modem_packet_t*)malloc(sizeof(modem_packet_t) + read_len[index]);
    if(!modem_packet) { memory_low = true; return false; }

    // Get packet RSSI and SNR
    modem_packet->snr_raw = radio->packetSnrRaw();

    // Pass raw SNR to get RSSI as SX127X driver requires it for calculations
    modem_packet->rssi = radio->packetRssi(modem_packet->snr_raw);

    modem_packet->interface = index;

    // Send packet to event queue, but free the
    // allocated memory again if the queue is
    // unable to receive the packet.
    modem_packet->len = read_len[index];
    memcpy(modem_packet->data, pbuf, read_len[index]);
    if (!modem_packet_queue || xQueueSendFromISR(modem_packet_queue, &modem_packet, NULL) != pdPASS) {
        free(modem_packet);
        return false;
    }
    return true;
}

void ISR_VECT receive_callback(uint8_t index, int packet_size) {
    selected_radio = interface_obj[index];
    bool    ready    = false;

  BaseType_t int_mask;
  if (!promisc) {
    // The standard operating mode allows large
    // packets with a payload up to 500 bytes,
    // by combining two raw LoRa packets.
    // We read the 1-byte header and extract
    // packet sequence number and split flags
    
    uint8_t header   = selected_radio->read(); packet_size--;
    uint8_t sequence = packetSequence(header);

    if (isSplitPacket(header) && seq[index] == SEQ_UNSET) {
      // This is the first part of a split
      // packet, so we set the seq variable
      // and add the data to the buffer
      #if MCU_VARIANT == MCU_NRF52
        int_mask = taskENTER_CRITICAL_FROM_ISR(); read_len[index] = 0; taskEXIT_CRITICAL_FROM_ISR(int_mask);
      #else
        read_len[index] = 0;
      #endif
      
      seq[index] = sequence;

      getPacketData(selected_radio, packet_size);

    } else if (isSplitPacket(header) && seq[index] == sequence) {
      // This is the second part of a split
      // packet, so we add it to the buffer
      // and set the ready flag.
      
      getPacketData(selected_radio, packet_size);

      seq[index] = SEQ_UNSET;
      ready = true;

    } else if (isSplitPacket(header) && seq[index] != sequence) {
      // This split packet does not carry the
      // same sequence id, so we must assume
      // that we are seeing the first part of
      // a new split packet.
      #if MCU_VARIANT == MCU_NRF52
        int_mask = taskENTER_CRITICAL_FROM_ISR(); read_len[index] = 0; taskEXIT_CRITICAL_FROM_ISR(int_mask);
      #else
        read_len[index] = 0;
      #endif
      seq[index] = sequence;

      getPacketData(selected_radio, packet_size);

    } else if (!isSplitPacket(header)) {
      // This is not a split packet, so we
      // just read it and set the ready
      // flag to true.

      if (seq[index] != SEQ_UNSET) {
        // If we already had part of a split
        // packet in the buffer, we clear it.
        #if MCU_VARIANT == MCU_NRF52
          int_mask = taskENTER_CRITICAL_FROM_ISR(); read_len[index] = 0; taskEXIT_CRITICAL_FROM_ISR(int_mask);
        #else
          read_len[index] = 0;
        #endif
        seq[index] = SEQ_UNSET;
      }

      getPacketData(selected_radio, packet_size);

      ready = true;
    }
  } else {
    // In promiscuous mode, raw packets are
    // output directly to the host
    read_len[index] = 0;

    getPacketData(selected_radio, packet_size);

    ready = true;
  }

  #if MCU_VARIANT == MCU_NRF52
  // Temporary fix for permanent preamble detected bug
  // TODO: figure out how to prevent the timing issue which causes the IRQ to never be cleared
  interface_obj[0]->getPacketValidity();
  #endif

  if (ready) {
      queue_packet(selected_radio, index);
  }

  last_rx = millis();
}

bool startRadio(RadioInterface* radio) {
  update_radio_lock(radio);
  
  if (modems_installed && !console_active) {
    if (!radio->getRadioOnline()) {
        if (!radio->getRadioLock() && hw_ready) {
          if (!radio->begin()) {
            // The radio could not be started.
            // Indicate this failure over both the
            // serial port and with the onboard LEDs
            kiss_indicate_error(ERROR_INITRADIO);
            led_indicate_error(0);
            return false;
          } else {
            radio->enableCrc();

            radio->onReceive(receive_callback);

            radio->updateBitrate();
            sort_interfaces();
            kiss_indicate_phy_stats(radio);

            lora_receive(radio);

            // Flash an info pattern to indicate
            // that the radio is now on
            kiss_indicate_radiostate(radio);
            led_indicate_info(3);
            return true;
          }

        } else {
          // Flash a warning pattern to indicate
          // that the radio was locked, and thus
          // not started
          kiss_indicate_radiostate(radio);
          led_indicate_warning(3);
          return false;
        }
      } else {
        // If radio is already on, we silently
        // ignore the request.
        kiss_indicate_radiostate(radio);
        return true;
      }
  }
  return false;
}

void stopRadio(RadioInterface* radio) {
  if (radio->getRadioOnline()) {
      radio->end();
      sort_interfaces();
      kiss_indicate_radiostate(radio);
  }
}

void update_radio_lock(RadioInterface* radio) {
  if (radio->getFrequency() != 0 && radio->getSignalBandwidth() != 0 && radio->getTxPower() != 0xFF && radio->getSpreadingFactor() != 0) {
    radio->setRadioLock(false);
  } else {
    radio->setRadioLock(true);
  }
}

// Check if the queue is full for the selected radio.
// Returns true if full, false if not
bool queue_full(RadioInterface* radio) {
  return (queue_height[radio->getIndex()] >= (CONFIG_QUEUE_MAX_LENGTH) || queued_bytes[radio->getIndex()] >= (getQueueSize(radio->getIndex())));
}

volatile bool queue_flushing = false;

// Flushes all packets for the interface
void flush_queue(RadioInterface* radio) {
    uint8_t index = radio->getIndex();
  if (!queue_flushing) {
    queue_flushing = true;

    led_tx_on();
    uint16_t processed = 0;
    uint8_t data_byte;

    while (!fifo16_isempty(&packet_starts[index])) {
      uint16_t start = fifo16_pop(&packet_starts[index]);
      uint16_t length = fifo16_pop(&packet_lengths[index]);

      if (length >= MIN_L && length <= MTU) {
        for (uint16_t i = 0; i < length; i++) {
          uint16_t pos = (start+i)%(getQueueSize(index));
          tbuf[i] = packet_queue[index][pos];
        }
        transmit(radio, length);
        processed++;
      }
    }

    lora_receive(radio);
    led_tx_off();

  }

  queue_flushing = false;
  queue_height[index] = 0;
  queued_bytes[index] = 0;
  radio->updateAirtime();

  #if HAS_DISPLAY
    display_tx[radio->getIndex()] = true;
  #endif
}

void pop_queue(RadioInterface* radio) {
  uint8_t index = radio->getIndex();
  if (!queue_flushing) {
    queue_flushing = true;
    led_tx_on(); uint16_t processed = 0;

    if (!fifo16_isempty(&packet_starts[index])) {

      uint16_t start = fifo16_pop(&packet_starts[index]);
      uint16_t length = fifo16_pop(&packet_lengths[index]);
      if (length >= MIN_L && length <= MTU) {
        for (uint16_t i = 0; i < length; i++) {
          uint16_t pos = (start+i)%getQueueSize(index);
          tbuf[i] = packet_queue[index][pos];
        }

        transmit(radio, length); processed++;
      }

      queue_height[index] -= processed;
      queued_bytes[index] -= length;
    }

    lora_receive(radio); led_tx_off();
  }

  radio->updateAirtime();

  queue_flushing = false;
  
  #if HAS_DISPLAY
    display_tx[radio->getIndex()] = true;
  #endif

}

void transmit(RadioInterface* radio, uint16_t size) {
  if (radio->getRadioOnline()) { 
    if (!promisc) {
      uint16_t  written = 0;
      uint8_t header  = random(256) & 0xF0;

      if (size > SINGLE_MTU - HEADER_L) {
        header = header | FLAG_SPLIT;
      }


      radio->beginPacket();
      radio->write(header); written++;

      for (uint16_t i=0; i < size; i++) {
        radio->write(tbuf[i]);

        written++;

        // Only start a new packet if this is a split packet and it has
        // exceeded the length of a single packet
        if (written == 255 && isSplitPacket(header)) {
          if (!radio->endPacket()) {
              kiss_indicate_error(ERROR_MODEM_TIMEOUT);
              kiss_indicate_error(ERROR_TXFAILED);
              led_indicate_error(5);
              hard_reset();
          }

          radio->addAirtime();
          radio->beginPacket();
          radio->write(header);
          written = 1;
        }
      }

      if (!radio->endPacket()) {
        kiss_indicate_error(ERROR_MODEM_TIMEOUT);
        kiss_indicate_error(ERROR_TXFAILED);
        led_indicate_error(5);
        hard_reset();
      }
      radio->addAirtime();

    } else {
      // In promiscuous mode, we only send out
      // plain raw LoRa packets with a maximum
      // payload of 255 bytes
      led_tx_on();
      uint16_t  written = 0;
      
      // Cap packets at 255 bytes
      if (size > SINGLE_MTU) {
        size = SINGLE_MTU;
      }

      // If implicit header mode has been set,
      // set packet length to payload data length
      if (!implicit) {
        radio->beginPacket();
      } else {
        radio->beginPacket(size);
      }

      for (uint16_t i=0; i < size; i++) {
        radio->write(tbuf[i]);

        written++;
      }
      radio->endPacket(); radio->addAirtime();
    }
    last_tx = millis();
  } else {
    kiss_indicate_error(ERROR_TXFAILED);
    led_indicate_error(5);
  }
}

void serial_callback(uint8_t sbyte) {
  if (IN_FRAME && sbyte == FEND && 
            command == CMD_DATA) {
    IN_FRAME = false;

    if (interface < INTERFACE_COUNT) {
        if (!fifo16_isfull(&packet_starts[interface]) && (queued_bytes[interface] < (getQueueSize(interface)))) {
            uint16_t s = current_packet_start[interface];
            int32_t e = queue_cursor[interface]-1; if (e == -1) e = (getQueueSize(interface))-1;
            uint16_t l;

            if (s != e) {
                l = (s < e) ? e - s + 1: (getQueueSize(interface)) - s + e + 1;
            } else {
                l = 1;
            }

            if (l >= MIN_L) {
                queue_height[interface]++;

                fifo16_push(&packet_starts[interface], s);
                fifo16_push(&packet_lengths[interface], l);
                current_packet_start[interface] = queue_cursor[interface];
            }

        }
    }

  } else if (sbyte == FEND) {
    IN_FRAME = true;
    command = CMD_UNKNOWN;
    frame_len = 0;
  } else if (IN_FRAME && frame_len < MTU) {
    // Have a look at the command byte first
    if (frame_len == 0 && command == CMD_UNKNOWN) {
        command = sbyte;

    } else if  (command == CMD_SEL_INT) {
            interface = sbyte;
    } else if  (command == CMD_DATA) {
        if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }

            if (interface < INTERFACE_COUNT) {
                if (queue_height[interface] < CONFIG_QUEUE_MAX_LENGTH && queued_bytes[interface] < (getQueueSize(interface))) {
                  queued_bytes[interface]++;
                  packet_queue[interface][queue_cursor[interface]++] = sbyte;
                  if (queue_cursor[interface] == (getQueueSize(interface))) queue_cursor[interface] = 0;
                }
            }
        }
    } else if (command == CMD_INTERFACES) {
        for (int i = 0; i < INTERFACE_COUNT; i++) {
            kiss_indicate_interface(i);
        }
    } else if (command == CMD_FREQUENCY) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 4) {
          selected_radio = interface_obj[interface];
          uint32_t freq = (uint32_t)cmdbuf[0] << 24 | (uint32_t)cmdbuf[1] << 16 | (uint32_t)cmdbuf[2] << 8 | (uint32_t)cmdbuf[3];

          if (freq == 0) {
            kiss_indicate_frequency(selected_radio);
          } else {
            if (op_mode == MODE_HOST) selected_radio->setFrequency(freq);
            kiss_indicate_frequency(selected_radio);
          }
          interface = 0;
        }
    } else if (command == CMD_BANDWIDTH) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }
        

        if (frame_len == 4) {
          selected_radio = interface_obj[interface];
          uint32_t bw = (uint32_t)cmdbuf[0] << 24 | (uint32_t)cmdbuf[1] << 16 | (uint32_t)cmdbuf[2] << 8 | (uint32_t)cmdbuf[3];

          if (bw == 0) {
            kiss_indicate_bandwidth(selected_radio);
          } else {
            if (op_mode == MODE_HOST) selected_radio->setSignalBandwidth(bw);
            selected_radio->updateBitrate();
            sort_interfaces();
            kiss_indicate_bandwidth(selected_radio);
            kiss_indicate_phy_stats(selected_radio);
          }
          interface = 0;
        }
    } else if (command == CMD_TXPOWER) {
      selected_radio = interface_obj[interface];

      if (sbyte == 0xFF) {
        kiss_indicate_txpower(selected_radio);
      } else {
        int8_t txp = (int8_t)sbyte;

        if (op_mode == MODE_HOST) setTXPower(selected_radio, txp);
        kiss_indicate_txpower(selected_radio);
      }
      interface = 0;
    } else if (command == CMD_SF) {
      selected_radio = interface_obj[interface];

      if (sbyte == 0xFF) {
        kiss_indicate_spreadingfactor(selected_radio);
      } else {
        int sf = sbyte;
        if (sf < 5) sf = 5;
        if (sf > 12) sf = 12;

        if (op_mode == MODE_HOST) selected_radio->setSpreadingFactor(sf);
        selected_radio->updateBitrate();
        sort_interfaces();
        kiss_indicate_spreadingfactor(selected_radio);
        kiss_indicate_phy_stats(selected_radio);
      }
      interface = 0;
    } else if (command == CMD_CR) {
      selected_radio = interface_obj[interface];
      if (sbyte == 0xFF) {
        kiss_indicate_codingrate(selected_radio);
      } else {
        int cr = sbyte;
        if (cr < 5) cr = 5;
        if (cr > 8) cr = 8;

        if (op_mode == MODE_HOST) selected_radio->setCodingRate4(cr);
        selected_radio->updateBitrate();
        sort_interfaces();
        kiss_indicate_codingrate(selected_radio);
        kiss_indicate_phy_stats(selected_radio);
      }
      interface = 0;
    } else if (command == CMD_IMPLICIT) {
      set_implicit_length(sbyte);
      kiss_indicate_implicit_length();
    } else if (command == CMD_LEAVE) {
      if (sbyte == 0xFF) {
        display_unblank();
        cable_state   = CABLE_STATE_DISCONNECTED;
        //current_rssi  = -292;
        last_rssi     = -292;
        last_rssi_raw = 0x00;
        last_snr_raw  = 0x80;
      }
    } else if (command == CMD_RADIO_STATE) {
      selected_radio = interface_obj[interface];
      if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
      if (sbyte == 0xFF) {
        kiss_indicate_radiostate(selected_radio);
      } else if (sbyte == 0x00) {
        stopRadio(selected_radio);
      } else if (sbyte == 0x01) {
        startRadio(selected_radio);
      }
      interface = 0;
    } else if (command == CMD_ST_ALOCK) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          selected_radio = interface_obj[interface];
          uint16_t at = (uint16_t)cmdbuf[0] << 8 | (uint16_t)cmdbuf[1];

          if (at == 0) {
            selected_radio->setSTALock(0.0);
          } else {
            int st_airtime_limit = (float)at/(100.0*100.0);
            if (st_airtime_limit >= 1.0) { st_airtime_limit = 0.0; }
            selected_radio->setSTALock(st_airtime_limit);
          }
          kiss_indicate_st_alock(selected_radio);
          interface = 0;
        }
    } else if (command == CMD_LT_ALOCK) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          selected_radio = interface_obj[interface];
          uint16_t at = (uint16_t)cmdbuf[0] << 8 | (uint16_t)cmdbuf[1];

          if (at == 0) {
            selected_radio->setLTALock(0.0);
          } else {
            int lt_airtime_limit = (float)at/(100.0*100.0);
            if (lt_airtime_limit >= 1.0) { lt_airtime_limit = 0.0; }
            selected_radio->setLTALock(lt_airtime_limit);
          }
          kiss_indicate_lt_alock(selected_radio);
          interface = 0;
        }
    } else if (command == CMD_STAT_RX) {
      kiss_indicate_stat_rx();
    } else if (command == CMD_STAT_TX) {
      kiss_indicate_stat_tx();
    } else if (command == CMD_STAT_RSSI) {
      kiss_indicate_stat_rssi(interface_obj[interface]);
    } else if (command == CMD_RADIO_LOCK) {
      selected_radio = interface_obj[interface];
      update_radio_lock(selected_radio);
      kiss_indicate_radio_lock(selected_radio);
      interface = 0;
    } else if (command == CMD_BLINK) {
      led_indicate_info(sbyte);
    } else if (command == CMD_RANDOM) {
      // pick an interface at random to get data from
      int int_index = random(INTERFACE_COUNT);
      selected_radio = interface_obj[int_index];
      kiss_indicate_random(getRandom(selected_radio));
      interface = 0;
    } else if (command == CMD_DETECT) {
      if (sbyte == DETECT_REQ) {
        if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
        kiss_indicate_detect();
      }
    } else if (command == CMD_PROMISC) {
      if (sbyte == 0x01) {
        promisc_enable();
      } else if (sbyte == 0x00) {
        promisc_disable();
      }
      kiss_indicate_promisc();
    } else if (command == CMD_READY) {
      selected_radio = interface_obj[interface];
      if (!queue_full(selected_radio)) {
        kiss_indicate_ready();
      } else {
        kiss_indicate_not_ready();
      }
    } else if (command == CMD_UNLOCK_ROM) {
      if (sbyte == ROM_UNLOCK_BYTE) {
        unlock_rom();
      }
    } else if (command == CMD_RESET) {
      if (sbyte == CMD_RESET_BYTE) {
        hard_reset();
      }
    } else if (command == CMD_ROM_READ) {
      kiss_dump_eeprom();
    } else if (command == CMD_ROM_WRITE) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }

        if (frame_len == 2) {
          eeprom_write(cmdbuf[0], cmdbuf[1]);
        }
    } else if (command == CMD_FW_VERSION) {
      kiss_indicate_version();
    } else if (command == CMD_PLATFORM) {
      kiss_indicate_platform();
    } else if (command == CMD_MCU) {
      kiss_indicate_mcu();
    } else if (command == CMD_BOARD) {
      kiss_indicate_board();
    } else if (command == CMD_CONF_SAVE) {
        // todo: add extra space in EEPROM so this isn't hardcoded
      eeprom_conf_save(interface_obj[0]);
    } else if (command == CMD_CONF_DELETE) {
      eeprom_conf_delete();
    } else if (command == CMD_FB_EXT) {
      #if HAS_DISPLAY == true
        if (sbyte == 0xFF) {
          kiss_indicate_fbstate();
        } else if (sbyte == 0x00) {
          ext_fb_disable();
          kiss_indicate_fbstate();
        } else if (sbyte == 0x01) {
          ext_fb_enable();
          kiss_indicate_fbstate();
        }
      #endif
    } else if (command == CMD_FB_WRITE) {
      if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
        }
        #if HAS_DISPLAY
          if (frame_len == 9) {
            uint8_t line = cmdbuf[0];
            if (line > 63) line = 63;
            int fb_o = line*8; 
            memcpy(fb+fb_o, cmdbuf+1, 8);
          }
        #endif
    } else if (command == CMD_FB_READ) {
      if (sbyte != 0x00) {
        kiss_indicate_fb();
      }
    } else if (command == CMD_DISP_READ) {
      if (sbyte != 0x00) { kiss_indicate_disp(); }
    } else if (command == CMD_DEV_HASH) {
        if (sbyte != 0x00) {
          kiss_indicate_device_hash();
        }
    } else if (command == CMD_DEV_SIG) {
        if (sbyte == FESC) {
              ESCAPE = true;
          } else {
              if (ESCAPE) {
                  if (sbyte == TFEND) sbyte = FEND;
                  if (sbyte == TFESC) sbyte = FESC;
                  ESCAPE = false;
              }
              if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
          }

          if (frame_len == DEV_SIG_LEN) {
            memcpy(dev_sig, cmdbuf, DEV_SIG_LEN);
            device_save_signature();
          }
    } else if (command == CMD_FW_UPD) {
      if (sbyte == 0x01) {
        firmware_update_mode = true;
      } else {
        firmware_update_mode = false;
      }
    } else if (command == CMD_HASHES) {
        if (sbyte == 0x01) {
          kiss_indicate_target_fw_hash();
        } else if (sbyte == 0x02) {
          kiss_indicate_fw_hash();
        } else if (sbyte == 0x03) {
          kiss_indicate_bootloader_hash();
        } else if (sbyte == 0x04) {
          kiss_indicate_partition_table_hash();
        }
    } else if (command == CMD_FW_HASH) {
        if (sbyte == FESC) {
              ESCAPE = true;
          } else {
              if (ESCAPE) {
                  if (sbyte == TFEND) sbyte = FEND;
                  if (sbyte == TFESC) sbyte = FESC;
                  ESCAPE = false;
              }
              if (frame_len < CMD_L) cmdbuf[frame_len++] = sbyte;
          }

          if (frame_len == DEV_HASH_LEN) {
            memcpy(dev_firmware_hash_target, cmdbuf, DEV_HASH_LEN);
            device_save_firmware_hash();
          }
    } else if (command == CMD_BT_CTRL) {
      #if HAS_BLUETOOTH || HAS_BLE
        if (sbyte == 0x00) {
          bt_stop();
          bt_conf_save(false);
        } else if (sbyte == 0x01) {
          bt_start();
          bt_conf_save(true);
        } else if (sbyte == 0x02) {
          if (bt_state == BT_STATE_OFF) {
            bt_start();
            bt_conf_save(true);
          }
          if (bt_state != BT_STATE_CONNECTED) {
            bt_enable_pairing();
          }
        }
      #endif
    } else if (command == CMD_BT_UNPAIR) {
      #if HAS_BLE
        if (sbyte == 0x01) { bt_debond_all(); }
      #endif
    } else if (command == CMD_DISP_INT) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            display_intensity = sbyte;
            di_conf_save(display_intensity);
            display_unblank();
        }

      #endif
    } else if (command == CMD_DISP_ADDR) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            display_addr = sbyte;
            da_conf_save(display_addr);
        }

      #endif
    } else if (command == CMD_DISP_BLNK) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            db_conf_save(sbyte);
            display_unblank();
        }

      #endif
    } else if (command == CMD_DISP_ROT) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            drot_conf_save(sbyte);
            display_unblank();
        }
      #endif
    } else if (command == CMD_DIS_IA) {
      if (sbyte == FESC) {
          ESCAPE = true;
      } else {
          if (ESCAPE) {
              if (sbyte == TFEND) sbyte = FEND;
              if (sbyte == TFESC) sbyte = FESC;
              ESCAPE = false;
          }
          dia_conf_save(sbyte);
      }
    } else if (command == CMD_DISP_RCND) {
      #if HAS_DISPLAY
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            if (sbyte > 0x00) recondition_display = true;
        }
      #endif
    } else if (command == CMD_NP_INT) {
      #if HAS_NP
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }
            sbyte;
            led_set_intensity(sbyte);
            np_int_conf_save(sbyte);
        }

      #endif
    }
  }
}

#if MCU_VARIANT == MCU_ESP32
  portMUX_TYPE update_lock = portMUX_INITIALIZER_UNLOCKED;
#endif

bool medium_free(RadioInterface* radio) {
  radio->updateModemStatus();
  if (radio->getAvdInterference() && radio->getInterference()) { return false; }
  return !radio->getDCD();
}

void validate_status() {
  #if MCU_VARIANT == MCU_ESP32
      // TODO: Get ESP32 boot flags
      uint8_t boot_flags = 0x02;
      uint8_t F_POR = 0x00;
      uint8_t F_BOR = 0x00;
      uint8_t F_WDR = 0x01;
  #elif MCU_VARIANT == MCU_NRF52
      // TODO: Get NRF52 boot flags
      uint8_t boot_flags = 0x02;
      uint8_t F_POR = 0x00;
      uint8_t F_BOR = 0x00;
      uint8_t F_WDR = 0x01;
  #endif

  if (hw_ready || device_init_done) {
    hw_ready = false;
    Serial.write("Error, invalid hardware check state\r\n");
    #if HAS_DISPLAY
      if (disp_ready) {
        device_init_done = true;
        update_display();
      }
    #endif
    led_indicate_boot_error();
  }

  if (boot_flags & (1<<F_POR)) {
    boot_vector = START_FROM_POWERON;
  } else if (boot_flags & (1<<F_BOR)) {
    boot_vector = START_FROM_BROWNOUT;
  } else if (boot_flags & (1<<F_WDR)) {
    boot_vector = START_FROM_BOOTLOADER;
  } else {
      Serial.write("Error, indeterminate boot vector\r\n");
      #if HAS_DISPLAY
        if (disp_ready) {
          device_init_done = true;
          update_display();
        }
      #endif
      led_indicate_boot_error();
  }

  if (boot_vector == START_FROM_BOOTLOADER || boot_vector == START_FROM_POWERON) {
    if (eeprom_lock_set()) {
      if (eeprom_product_valid() && eeprom_model_valid() && eeprom_hwrev_valid()) {
        if (eeprom_checksum_valid()) {
          eeprom_ok = true;
          if (modems_installed) {
            if (device_init()) {
              hw_ready = true;
            } else {
              hw_ready = false;
            }
          } else {
            hw_ready = false;
            Serial.write("No radio module found\r\n");
            #if HAS_DISPLAY
              if (disp_ready) {
                device_init_done = true;
                update_display();
              }
            #endif
          }
        } else {
          hw_ready = false;
          Serial.write("Invalid EEPROM checksum\r\n");
          #if HAS_DISPLAY
            if (disp_ready) {
              device_init_done = true;
              update_display();
            }
          #endif
        }
      } else {
        hw_ready = false;
        Serial.write("Invalid EEPROM configuration\r\n");
        #if HAS_DISPLAY
          if (disp_ready) {
            device_init_done = true;
            update_display();
          }
        #endif
      }
    } else {
      hw_ready = false;
      Serial.write("Device unprovisioned, no device configuration found in EEPROM\r\n");
      #if HAS_DISPLAY
        if (disp_ready) {
          device_init_done = true;
          update_display();
        }
      #endif
    }
  } else {
    hw_ready = false;
    Serial.write("Error, incorrect boot vector\r\n");
    #if HAS_DISPLAY
      if (disp_ready) {
        device_init_done = true;
        update_display();
      }
    #endif
    led_indicate_boot_error();
  }
}

void tx_queue_handler(RadioInterface* radio) {
  if (queue_height[radio->getIndex()] > 0) {
    if (radio->getCW() == -1) {
      radio->setCW(random(radio->getCWMin(), radio->getCWMax()));
      radio->setCWWaitTarget(radio->getCW() * radio->getCSMASlotMS());
    }

    if (radio->getDifsWaitStart() == 0) {                                                  // DIFS wait not yet started
      if (medium_free(radio)) { radio->setDifsWaitStart(millis()); return; }                  // Set DIFS wait start time
      else               { return; } }                                            // Medium not yet free, continue waiting
    
    else {                                                                        // We are waiting for DIFS or CW to pass
      if (!medium_free(radio)) { radio->setDifsWaitStart(0); radio->setCWWaitStart(0); return; }   // Medium became occupied while in DIFS wait, restart waiting when free again
      else {                                                                      // Medium is free, so continue waiting
        if (millis() < radio->getDifsWaitStart()+radio->getDifsMS()) { return; }                       // DIFS has not yet passed, continue waiting
        else {                                                                    // DIFS has passed, and we are now in CW wait
          if (radio->getCWWaitStart() == 0) { radio->setCWWaitStart(millis()); return; }          // If we haven't started counting CW wait time, do it from now
          else {                                                                  // If we are already counting CW wait time, add it to the counter
            radio->addCWWaitPassed(millis()-radio->getCWWaitStart()); radio->setCWWaitStart(millis());
            if (radio->getCWWaitStatus()) { return; }                      // Contention window wait time has not yet passed, continue waiting
            else {                                                                // Wait time has passed, flush the queue
              if (!radio->getLimitRate()) { flush_queue(radio); } else { pop_queue(radio); }
              radio->resetCWWaitPassed(); radio->setCW(-1); radio->setDifsWaitStart(0); }
          }
        }
      }
    }
  }
}

void work_while_waiting() { loop(); }

void loop() {
    #if MCU_VARIANT == MCU_ESP32
      modem_packet_t *modem_packet = NULL;
      if(modem_packet_queue && xQueueReceive(modem_packet_queue, &modem_packet, 0) == pdTRUE && modem_packet) {
        uint8_t packet_interface = modem_packet->interface;
        read_len[packet_interface] = modem_packet->len;
        last_rssi = modem_packet->rssi;
        last_snr_raw = modem_packet->snr_raw;
        memcpy(&pbuf, modem_packet->data, modem_packet->len);
        free(modem_packet);
        modem_packet = NULL;

        kiss_indicate_stat_rssi(interface_obj[packet_interface]);
        kiss_indicate_stat_snr(interface_obj[packet_interface]);
        kiss_write_packet(packet_interface);
      }

    #elif MCU_VARIANT == MCU_NRF52
      modem_packet_t *modem_packet = NULL;
      if(modem_packet_queue && xQueueReceive(modem_packet_queue, &modem_packet, 0) == pdTRUE && modem_packet) {
        uint8_t packet_interface = modem_packet->interface;
        read_len[packet_interface] = modem_packet->len;
        last_rssi = modem_packet->rssi;
        last_snr_raw = modem_packet->snr_raw;
        memcpy(&pbuf, modem_packet->data, modem_packet->len);
        free(modem_packet);
        modem_packet = NULL;

        kiss_indicate_stat_rssi(interface_obj[packet_interface]);
        kiss_indicate_stat_snr(interface_obj[packet_interface]);
        kiss_write_packet(packet_interface);
      }
    #endif

    bool ready = false;
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        selected_radio = interface_obj[i];
        if (selected_radio->getRadioOnline()) {
            ready = true;
        }
    }


  // If at least one radio is online then we can continue
  if (ready) {
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        selected_radio = interface_obj_sorted[i];

        if (selected_radio->calculateALock() || !selected_radio->getRadioOnline()) {
            // skip this interface
            continue;
        }
        tx_queue_handler(selected_radio);
        selected_radio->checkModemStatus();
    }
  
  } else {
    if (hw_ready) {
      if (console_active) {
        #if HAS_CONSOLE
          console_loop();
        #endif
      } else {
        led_indicate_standby();
      }
    } else {
      led_indicate_not_ready();
      // shut down all radio interfaces
      for (int i = 0; i < INTERFACE_COUNT; i++) {
          stopRadio(interface_obj[i]);
      }
    }
  }

  process_serial();

  #if HAS_DISPLAY
    #if DISPLAY == OLED || DISPLAY == TFT || DISPLAY == ADAFRUIT_TFT
    if (disp_ready) update_display();
    #elif DISPLAY == EINK_BW || DISPLAY == EINK_3C
    // Display refreshes take so long on e-paper displays that they can disrupt
    // the regular operation of the device. To combat this the time it is
    // chosen to do so must be strategically chosen. Particularly on the
    // RAK4631, the display and the potentially installed SX1280 modem share
    // the same SPI bus. Thus it is not possible to solve this by utilising the
    // callback functionality to poll the modem in this case. todo, this may be
    // able to be improved in the future.
    if (disp_ready) {
        if (millis() - last_tx >= 4000) {
            if (millis() - last_rx >= 1000) {
                update_display();
            }
        }
    }
    #endif
  #endif

  #if HAS_PMU
    if (pmu_ready) update_pmu();
  #endif

  #if HAS_BLUETOOTH || HAS_BLE == true
    if (!console_active && bt_ready) update_bt();
  #endif

  #if HAS_INPUT
    input_read();
  #endif

  #if HAS_GPS
    while (gps_s.available() > 0) {
      if (gps.encode(gps_s.read()) && millis() - last_gps >= GPS_INTERVAL) {
          kiss_indicate_location();
          last_gps = millis();
      }
    }
    if (millis() > 5000 && gps.charsProcessed() < 10) {
        while (true) {
            Serial.println(F("No GPS detected: check wiring."));
        }
    }
  #endif

  if (memory_low) {
    #if PLATFORM == PLATFORM_ESP32
      if (esp_get_free_heap_size() < 8192) {
        kiss_indicate_error(ERROR_MEMORY_LOW); memory_low = false;
      } else {
        memory_low = false;
      }
    #else
      kiss_indicate_error(ERROR_MEMORY_LOW); memory_low = false;
    #endif
  }
}

void process_serial() {
      buffer_serial();
      if (!fifo_isempty(&serialFIFO)) serial_poll();
}

void sleep_now() {
  #if HAS_SLEEP == true
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        stopRadio(interface_obj[i]); // TODO: Check this on all platforms
    }
    #if PLATFORM == PLATFORM_ESP32
      #if BOARD_MODEL == BOARD_T3S3 || BOARD_MODEL == BOARD_XIAO_S3
        display_intensity = 0;
        update_display(true);
      #endif
      #if PIN_DISP_SLEEP >= 0
        pinMode(PIN_DISP_SLEEP, OUTPUT);
        digitalWrite(PIN_DISP_SLEEP, DISP_SLEEP_LEVEL);
      #endif
      #if HAS_BLUETOOTH
        if (bt_state == BT_STATE_CONNECTED) {
          bt_stop();
          delay(100);
        }
      #endif
      esp_sleep_enable_ext0_wakeup(PIN_WAKEUP, WAKEUP_LEVEL);
      esp_deep_sleep_start();
    #elif PLATFORM == PLATFORM_NRF52
      #if BOARD_MODEL == BOARD_HELTEC_T114
        npset(0,0,0);
        digitalWrite(PIN_VEXT_EN, LOW);
        digitalWrite(PIN_T114_TFT_BLGT, HIGH);
        digitalWrite(PIN_T114_TFT_EN, HIGH);
      #elif BOARD_MODEL == BOARD_TECHO
        for (uint8_t i = display_intensity; i > 0; i--) { analogWrite(pin_backlight, i-1); delay(1); }
        epd_black(true); delay(300); epd_black(true); delay(300); epd_black(false);
        delay(2000);
        analogWrite(PIN_VEXT_EN, 0);
        delay(100);
      #endif
      sd_power_gpregret_set(0, 0x6d);
      nrf_gpio_cfg_sense_input(pin_btn_usr1, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
      NRF_POWER->SYSTEMOFF = 1;
    #endif
  #endif
}

void button_event(uint8_t event, unsigned long duration) {
    if (display_blanked) {
      display_unblank();
    } else {
      if (duration > 10000) {
        #if HAS_CONSOLE
          #if HAS_BLUETOOTH || HAS_BLE
            bt_stop();
          #endif
          console_active = true;
          console_start();
        #endif
      } else if (duration > 5000) {
        #if HAS_BLUETOOTH || HAS_BLE
          if (bt_state != BT_STATE_CONNECTED) { bt_enable_pairing(); }
        #endif
      } else if (duration > 700) {
        #if HAS_SLEEP
          sleep_now();
        #endif
      } else {
        #if HAS_BLUETOOTH || HAS_BLE
        if (bt_state != BT_STATE_CONNECTED) {
          if (bt_state == BT_STATE_OFF) {
            bt_start();
            bt_conf_save(true);
          } else {
            bt_stop();
            bt_conf_save(false);
          }
        }
        #endif
      }
    }
}

void poll_buffers() {
    process_serial();
}

volatile bool serial_polling = false;
void serial_poll() {
  serial_polling = true;

  while (!fifo_isempty(&serialFIFO)) {
    char sbyte = fifo_pop(&serialFIFO);
    serial_callback(sbyte);
  }

  serial_polling = false;
}

#define MAX_CYCLES 20

void buffer_serial() {
  if (!serial_buffering) {
    serial_buffering = true;

    uint8_t c = 0;

    #if HAS_BLUETOOTH || HAS_BLE == true
    while (
      c < MAX_CYCLES &&
      ( (bt_state != BT_STATE_CONNECTED && Serial.available()) || (bt_state == BT_STATE_CONNECTED && SerialBT.available()) )
      )
    #else
    while (c < MAX_CYCLES && Serial.available())
    #endif
    {
      c++;

      #if HAS_BLUETOOTH || HAS_BLE == true
        if (bt_state == BT_STATE_CONNECTED) {
          if (!fifo_isfull(&serialFIFO)) {
            fifo_push(&serialFIFO, SerialBT.read());
          }
        } else {
          if (!fifo_isfull(&serialFIFO)) {
            fifo_push(&serialFIFO, Serial.read());
          }
        }
      #else
        if (!fifo_isfull(&serialFIFO)) {
          fifo_push(&serialFIFO, Serial.read());
        }
      #endif
    }

    serial_buffering = false;
  }
}
