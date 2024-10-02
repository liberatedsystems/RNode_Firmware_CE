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

#include <Arduino.h>
#include <SPI.h>
#include "Utilities.h"

#if MCU_VARIANT == MCU_NRF52
  #define INTERFACE_SPI
  #if BOARD_MODEL == BOARD_RAK4631 
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
    SPIClass interface_spi[1] = {
            // SX1262
            SPIClass(
                NRF_SPIM3, 
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
#if HAS_BLUETOOTH || HAS_BLE == true
  bool bt_init_ran = false;
#endif

#if HAS_CONSOLE
  #include "Console.h"
#endif

char sbuf[128];

uint8_t *packet_queue[INTERFACE_COUNT];

void setup() {
  #if MCU_VARIANT == MCU_ESP32
    boot_seq();
    EEPROM.begin(EEPROM_SIZE);
    Serial.setRxBufferSize(CONFIG_UART_BUFFER_SIZE);
  #endif

  #if MCU_VARIANT == MCU_NRF52
    if (!eeprom_begin()) {
        Serial.write("EEPROM initialisation failed.\r\n");
    }
  #endif

  // Seed the PRNG for CSMA R-value selection
  # if MCU_VARIANT == MCU_ESP32
    // On ESP32, get the seed value from the
    // hardware RNG
    int seed_val = (int)esp_random();
  #else
    // Otherwise, get a pseudo-random seed
    // value from an unconnected analog pin
    int seed_val = analogRead(0);
  #endif
  randomSeed(seed_val);

  // Initialise serial communication
  memset(serialBuffer, 0, sizeof(serialBuffer));
  fifo_init(&serialFIFO, serialBuffer, CONFIG_UART_BUFFER_SIZE);

  Serial.begin(serial_baudrate);

  #if BOARD_MODEL != BOARD_RAK4631 && BOARD_MODEL != BOARD_T3S3 && BOARD_MODEL != BOARD_TECHO
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

  for (int i = 0; i < INTERFACE_COUNT; i++) {
      fifo16_init(&packet_starts[i], packet_starts_buf, CONFIG_QUEUE_MAX_LENGTH+1);
      fifo16_init(&packet_lengths[i], packet_lengths_buf, CONFIG_QUEUE_MAX_LENGTH+1);
      packet_queue[i] = (uint8_t*)malloc(getQueueSize(i)+1);
  }

  memset(packet_rdy_interfaces_buf, 0, sizeof(packet_rdy_interfaces_buf));

  fifo_init(&packet_rdy_interfaces, packet_rdy_interfaces_buf, MAX_INTERFACES);

  // Create and configure interface objects
  for (uint8_t i = 0; i < INTERFACE_COUNT; i++) {
      switch (interfaces[i]) {
          case SX126X:
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

          case SX127X:
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

          case SX128X:
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

    // Check installed transceiver chip(s) and probe boot parameters. If any of
    // the configured modems cannot be initialised, do not boot
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        switch (interfaces[i]) {
            case SX126X:
            case SX1262:
            case SX127X:
            case SX1276:
            case SX1278:
            case SX128X:
            case SX1280:
                selected_radio = interface_obj[i];
                break;

            default:
                modems_installed = false;
                break;
        }
        if (selected_radio->preInit()) {
          modems_installed = true;
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
      eeprom_update(eeprom_addr(ADDR_CONF_DINT), 0xFF);
    }
    #if DISPLAY == EINK_BW || DISPLAY == EINK_3C
    // Poll and process incoming serial commands whilst e-ink display is
    // refreshing to make device still seem responsive
    display_add_callback(process_serial);
    #endif
    disp_ready = display_init();
    update_display();
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
  // We need to convert the interface index to the command byte representation
  uint8_t cmd_byte = getInterfaceCommandByte(index);

  serial_write(FEND);

  // Add index of interface the packet came from
  serial_write(cmd_byte);

  for (uint16_t i = 0; i < read_len; i++) {
    uint8_t byte = pbuf[i];
    if (byte == FEND) { serial_write(FESC); byte = TFEND; }
    if (byte == FESC) { serial_write(FESC); byte = TFESC; }
    serial_write(byte);
  }
  serial_write(FEND);
  read_len = 0;
  packet_ready = false;
}

inline void getPacketData(RadioInterface* radio, uint16_t len) {
  while (len-- && read_len < MTU) {
    pbuf[read_len++] = radio->read();
  }
}

void receive_callback(uint8_t index, int packet_size) {
        selected_radio = interface_obj[index];
    bool    ready    = false;
  if (!promisc) {
    // The standard operating mode allows large
    // packets with a payload up to 500 bytes,
    // by combining two raw LoRa packets.
    // We read the 1-byte header and extract
    // packet sequence number and split flags
    
    uint8_t header   = selected_radio->read(); packet_size--;
    uint8_t sequence = packetSequence(header);

    if (isSplitPacket(header) && seq == SEQ_UNSET) {
      // This is the first part of a split
      // packet, so we set the seq variable
      // and add the data to the buffer
      read_len = 0;
      seq = sequence;

      getPacketData(selected_radio, packet_size);

    } else if (isSplitPacket(header) && seq == sequence) {
      // This is the second part of a split
      // packet, so we add it to the buffer
      // and set the ready flag.
      
      getPacketData(selected_radio, packet_size);

      seq = SEQ_UNSET;
      packet_interface = index;
      packet_ready = true;

    } else if (isSplitPacket(header) && seq != sequence) {
      // This split packet does not carry the
      // same sequence id, so we must assume
      // that we are seeing the first part of
      // a new split packet.
      read_len = 0;
      seq = sequence;

      getPacketData(selected_radio, packet_size);

    } else if (!isSplitPacket(header)) {
      // This is not a split packet, so we
      // just read it and set the ready
      // flag to true.

      if (seq != SEQ_UNSET) {
        // If we already had part of a split
        // packet in the buffer, we clear it.
        read_len = 0;
        seq = SEQ_UNSET;
      }

      getPacketData(selected_radio, packet_size);

      packet_interface = index;
      packet_ready = true;
    }
  } else {
    // In promiscuous mode, raw packets are
    // output directly to the host
    read_len = 0;

    getPacketData(selected_radio, packet_size);

    packet_interface = index;
    packet_ready = true;
  }

  last_rx = millis();
}

bool startRadio(RadioInterface* radio) {
  update_radio_lock(radio);
  
  if (modems_installed && !console_active) {
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

void stopRadio(RadioInterface* radio) {
  radio->end();
  sort_interfaces();
  kiss_indicate_radiostate(radio);
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
bool queueFull(RadioInterface* radio) {
  return (queue_height[radio->getIndex()] >= (CONFIG_QUEUE_MAX_LENGTH) || queued_bytes[radio->getIndex()] >= (getQueueSize(radio->getIndex())));
}

volatile bool queue_flushing = false;

// Flushes all packets for the interface
void flushQueue(RadioInterface* radio) {
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

    radio->setPostTxYieldTimeout(millis()+(lora_post_tx_yield_slots*selected_radio->getCSMASlotMS()));
  }

  queue_height[index] = 0;
  queued_bytes[index] = 0;
  selected_radio->updateAirtime();
  queue_flushing = false;
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

        if (written == 255) {
          radio->endPacket(); radio->addAirtime(written);
          radio->beginPacket();
          radio->write(header);
          written = 1;
        }
      }

      radio->endPacket(); radio->addAirtime(written);
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
      radio->endPacket(); radio->addAirtime(written);
    }
    last_tx = millis();
  } else {
    kiss_indicate_error(ERROR_TXFAILED);
    led_indicate_error(5);
  }
}

void serialCallback(uint8_t sbyte) {
  if (IN_FRAME && sbyte == FEND && 
            (command == CMD_INT0_DATA
          || command == CMD_INT1_DATA
          || command == CMD_INT2_DATA
          || command == CMD_INT3_DATA
          || command == CMD_INT4_DATA
          || command == CMD_INT5_DATA
          || command == CMD_INT6_DATA
          || command == CMD_INT7_DATA
          || command == CMD_INT8_DATA
          || command == CMD_INT9_DATA
          || command == CMD_INT10_DATA 
          || command == CMD_INT11_DATA)) {
    IN_FRAME = false;

    if (getInterfaceIndex(command) < INTERFACE_COUNT) {
            uint8_t index = getInterfaceIndex(command);
        if (!fifo16_isfull(&packet_starts[index]) && (queued_bytes[index] < (getQueueSize(index)))) {
            uint16_t s = current_packet_start[index];
            int32_t e = queue_cursor[index]-1; if (e == -1) e = (getQueueSize(index))-1;
            uint16_t l;

            if (s != e) {
                l = (s < e) ? e - s + 1: (getQueueSize(index)) - s + e + 1;
            } else {
                l = 1;
            }

            if (l >= MIN_L) {
                queue_height[index]++;

                fifo16_push(&packet_starts[index], s);
                fifo16_push(&packet_lengths[index], l);
                current_packet_start[index] = queue_cursor[index];
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
        if  (command == CMD_SEL_INT0 
                 || command == CMD_SEL_INT1 
                 || command == CMD_SEL_INT2 
                 || command == CMD_SEL_INT3 
                 || command == CMD_SEL_INT4 
                 || command == CMD_SEL_INT5 
                 || command == CMD_SEL_INT6 
                 || command == CMD_SEL_INT7 
                 || command == CMD_SEL_INT8 
                 || command == CMD_SEL_INT9 
                 || command == CMD_SEL_INT10 
                 || command == CMD_SEL_INT11) {
            interface = getInterfaceIndex(command);
        }

    } else if  (command == CMD_INT0_DATA 
             || command == CMD_INT1_DATA 
             || command == CMD_INT2_DATA 
             || command == CMD_INT3_DATA 
             || command == CMD_INT4_DATA 
             || command == CMD_INT5_DATA 
             || command == CMD_INT6_DATA 
             || command == CMD_INT7_DATA 
             || command == CMD_INT8_DATA 
             || command == CMD_INT9_DATA 
             || command == CMD_INT10_DATA 
             || command == CMD_INT11_DATA) {
        if (bt_state != BT_STATE_CONNECTED) cable_state = CABLE_STATE_CONNECTED;
        if (sbyte == FESC) {
            ESCAPE = true;
        } else {
            if (ESCAPE) {
                if (sbyte == TFEND) sbyte = FEND;
                if (sbyte == TFESC) sbyte = FESC;
                ESCAPE = false;
            }

            if (getInterfaceIndex(command) < INTERFACE_COUNT) {
                    uint8_t index = getInterfaceIndex(command);
                if (queue_height[index] < CONFIG_QUEUE_MAX_LENGTH && queued_bytes[index] < (getQueueSize(index))) {
                  queued_bytes[index]++;
                  packet_queue[index][queue_cursor[index]++] = sbyte;
                  if (queue_cursor[index] == (getQueueSize(index))) queue_cursor[index] = 0;
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
          uint32_t freq = (uint32_t)cmdbuf[0] << 24 | (uint32_t)cmdbuf[1] << 16 | (uint32_t)cmdbuf[2] << 8 | (uint32_t)cmdbuf[3];

          selected_radio = interface_obj[interface];
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
          uint32_t bw = (uint32_t)cmdbuf[0] << 24 | (uint32_t)cmdbuf[1] << 16 | (uint32_t)cmdbuf[2] << 8 | (uint32_t)cmdbuf[3];

          selected_radio = interface_obj[interface];

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
      selected_radio = interface_obj[interface];
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
          uint16_t at = (uint16_t)cmdbuf[0] << 8 | (uint16_t)cmdbuf[1];

          if (at == 0) {
            selected_radio->setSTALock(0.0);
          } else {
            int st_airtime_limit = (float)at/(100.0*100.0);
            if (st_airtime_limit >= 1.0) { st_airtime_limit = 0.0; }
            selected_radio->setSTALock(st_airtime_limit);
          }
          kiss_indicate_st_alock(selected_radio);
        }
        interface = 0;
    } else if (command == CMD_LT_ALOCK) {
      selected_radio = interface_obj[interface];
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
          uint16_t at = (uint16_t)cmdbuf[0] << 8 | (uint16_t)cmdbuf[1];

          if (at == 0) {
            selected_radio->setLTALock(0.0);
          } else {
            int lt_airtime_limit = (float)at/(100.0*100.0);
            if (lt_airtime_limit >= 1.0) { lt_airtime_limit = 0.0; }
            selected_radio->setLTALock(lt_airtime_limit);
          }
          kiss_indicate_lt_alock(selected_radio);
        }
        interface = 0;
    } else if (command == CMD_STAT_RX) {
      kiss_indicate_stat_rx();
    } else if (command == CMD_STAT_TX) {
      kiss_indicate_stat_tx();
    } else if (command == CMD_STAT_RSSI) {
      kiss_indicate_stat_rssi();
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
      if (!queueFull(selected_radio)) {
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
          bt_enable_pairing();
        }
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
    }
  }
}

#if MCU_VARIANT == MCU_ESP32
  portMUX_TYPE update_lock = portMUX_INITIALIZER_UNLOCKED;
#endif

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
            Serial.write("No valid radio module found\r\n");
            #if HAS_DISPLAY
              if (disp_ready) {
                device_init_done = true;
                update_display();
              }
            #endif
          }
        } else {
          hw_ready = false;
          #if HAS_DISPLAY
            if (disp_ready) {
              device_init_done = true;
              update_display();
            }
          #endif
        }
      } else {
        hw_ready = false;
        #if HAS_DISPLAY
          if (disp_ready) {
            device_init_done = true;
            update_display();
          }
        #endif
      }
    } else {
      hw_ready = false;
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

void loop() {
  if (packet_ready) {
        #if MCU_VARIANT == MCU_ESP32
        portENTER_CRITICAL(&update_lock);
        #elif MCU_VARIANT == MCU_NRF52
        portENTER_CRITICAL();
        #endif
        last_rssi = selected_radio->packetRssi();
        last_snr_raw = selected_radio->packetSnrRaw();
        #if MCU_VARIANT == MCU_ESP32
        portEXIT_CRITICAL(&update_lock);
        #elif MCU_VARIANT == MCU_NRF52
        portEXIT_CRITICAL();
        #endif
        kiss_indicate_stat_rssi();
        kiss_indicate_stat_snr();
        kiss_write_packet(packet_interface);
  }

    bool ready = false;
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        selected_radio = interface_obj[i];
        if (selected_radio->getRadioOnline()) {
            selected_radio->checkModemStatus();
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

        // If a higher data rate interface has received a packet after its
        // loop, it still needs to be the first to transmit, so check if this
        // is the case.
        for (int j = 0; j < INTERFACE_COUNT; j++) {
            if (!interface_obj_sorted[j]->calculateALock() && interface_obj_sorted[j]->getRadioOnline()) {
                if (interface_obj_sorted[j]->getBitrate() > selected_radio->getBitrate()) {
                    if (queue_height[interface_obj_sorted[j]->getIndex()] > 0) {
                        selected_radio = interface_obj_sorted[j];
                    }
                }
            }
        }

        if (queue_height[selected_radio->getIndex()] > 0) {
            uint32_t check_time = millis();
            if (check_time > selected_radio->getPostTxYieldTimeout()) {
                if (selected_radio->getDCDWaiting() && (check_time >= selected_radio->getDCDWaitUntil())) { selected_radio->setDCDWaiting(false); }
                if (!selected_radio->getDCDWaiting()) {
                    // todo, will the delay here slow down transmission with
                    // multiple interfaces? needs investigation
                    for (uint8_t dcd_i = 0; dcd_i < DCD_THRESHOLD*2; dcd_i++) {
                        delay(STATUS_INTERVAL_MS); selected_radio->updateModemStatus();
                    }

                    if (!selected_radio->getDCD()) {
                        uint8_t csma_r = (uint8_t)random(256);
                        if (selected_radio->getCSMAp() >= csma_r) {
                            flushQueue(selected_radio);
                        } else {
                            selected_radio->setDCDWaiting(true);
                            selected_radio->setDCDWaitUntil(millis()+selected_radio->getCSMASlotMS());
                        }
                    }
                }
            }
        }
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

  buffer_serial();
  if (!fifo_isempty(&serialFIFO)) serial_poll();

  #if HAS_DISPLAY
    #if DISPLAY == OLED
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
    #if HAS_BACKLIGHT
    if (button_pressed()) {
      digitalWrite(pin_backlight, !digitalRead(pin_backlight));
    }
    #endif
  #endif
}

void process_serial() {
      buffer_serial();
      if (!fifo_isempty(&serialFIFO)) serial_poll();
}

void sleep_now() {
  #if HAS_SLEEP == true
    #if BOARD_MODEL == BOARD_T3S3
      display_intensity = 0;
      update_display(true);
    #endif
    #if PIN_DISP_SLEEP >= 0
      pinMode(PIN_DISP_SLEEP, OUTPUT);
      digitalWrite(PIN_DISP_SLEEP, DISP_SLEEP_LEVEL);
    #endif
    esp_sleep_enable_ext0_wakeup(PIN_WAKEUP, WAKEUP_LEVEL);
    esp_deep_sleep_start();
  #endif
}

void button_event(uint8_t event, unsigned long duration) {
  if (duration > 2000) {
    sleep_now();
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
    serialCallback(sbyte);
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
