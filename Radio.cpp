// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license.

// Modifications and additions copyright 2024 by Mark Qvist & Jacob Eva
// Obviously still under the MIT license.

#include "Radio.hpp"

#if PLATFORM == PLATFORM_ESP32 
  #if defined(ESP32) and !defined(CONFIG_IDF_TARGET_ESP32S3)
    #include "soc/rtc_wdt.h"
  #endif
  #define ISR_VECT IRAM_ATTR
#else
  #define ISR_VECT
#endif

// SX126x registers
#define OP_RF_FREQ_6X               0x86
#define OP_SLEEP_6X                 0x84
#define OP_STANDBY_6X               0x80
#define OP_TX_6X                    0x83
#define OP_RX_6X                    0x82
#define OP_PA_CONFIG_6X             0x95
#define OP_SET_IRQ_FLAGS_6X         0x08 // also provides info such as
                                      // preamble detection, etc for
                                      // knowing when it's safe to switch
                                      // antenna modes
#define OP_CLEAR_IRQ_STATUS_6X      0x02
#define OP_GET_IRQ_STATUS_6X        0x12
#define OP_RX_BUFFER_STATUS_6X      0x13
#define OP_PACKET_STATUS_6X         0x14 // get snr & rssi of last packet
#define OP_CURRENT_RSSI_6X          0x15
#define OP_MODULATION_PARAMS_6X     0x8B // bw, sf, cr, etc.
#define OP_PACKET_PARAMS_6X         0x8C // crc, preamble, payload length, etc.
#define OP_STATUS_6X                0xC0
#define OP_TX_PARAMS_6X             0x8E // set dbm, etc
#define OP_PACKET_TYPE_6X           0x8A
#define OP_BUFFER_BASE_ADDR_6X      0x8F
#define OP_READ_REGISTER_6X         0x1D
#define OP_WRITE_REGISTER_6X        0x0D
#define OP_DIO3_TCXO_CTRL_6X        0x97
#define OP_DIO2_RF_CTRL_6X          0x9D
#define OP_CAD_PARAMS               0x88
#define OP_CALIBRATE_6X             0x89
#define OP_RX_TX_FALLBACK_MODE_6X   0x93
#define OP_REGULATOR_MODE_6X        0x96
#define OP_CALIBRATE_IMAGE_6X       0x98

#define MASK_CALIBRATE_ALL          0x7f

#define IRQ_TX_DONE_MASK_6X         0x01
#define IRQ_RX_DONE_MASK_6X         0x02
#define IRQ_HEADER_DET_MASK_6X      0x10
#define IRQ_PREAMBLE_DET_MASK_6X    0x04
#define IRQ_PAYLOAD_CRC_ERROR_MASK_6X 0x40
#define IRQ_ALL_MASK_6X             0b0100001111111111

#define MODE_LONG_RANGE_MODE_6X     0x01

#define OP_FIFO_WRITE_6X            0x0E
#define OP_FIFO_READ_6X             0x1E
#define REG_OCP_6X                0x08E7
#define REG_LNA_6X                0x08AC // no agc in sx1262
#define REG_SYNC_WORD_MSB_6X      0x0740
#define REG_SYNC_WORD_LSB_6X      0x0741
#define REG_PAYLOAD_LENGTH_6X     0x0702 // https://github.com/beegee-tokyo/SX126x-Arduino/blob/master/src/radio/sx126x/sx126x.h#L98
#define REG_RANDOM_GEN_6X         0x0819

#define MODE_TCXO_3_3V_6X           0x07
#define MODE_TCXO_3_0V_6X           0x06
#define MODE_TCXO_2_7V_6X           0x06
#define MODE_TCXO_2_4V_6X           0x06
#define MODE_TCXO_2_2V_6X           0x03
#define MODE_TCXO_1_8V_6X           0x02
#define MODE_TCXO_1_7V_6X           0x01
#define MODE_TCXO_1_6V_6X           0x00

#define MODE_STDBY_RC_6X            0x00
#define MODE_STDBY_XOSC_6X          0x01
#define MODE_FALLBACK_STDBY_RC_6X   0x20
#define MODE_IMPLICIT_HEADER        0x01
#define MODE_EXPLICIT_HEADER        0x00

#define SYNC_WORD_6X              0x1424

#define XTAL_FREQ_6X (double)32000000
#define FREQ_DIV_6X (double)pow(2.0, 25.0)
#define FREQ_STEP_6X (double)(XTAL_FREQ_6X / FREQ_DIV_6X)

extern FIFOBuffer packet_rdy_interfaces;
extern RadioInterface* interface_obj[];

// ISRs cannot provide parameters to the functions they call. Since we have
// multiple interfaces, we have to read each dio0 pin for each one and see
// which one is high. We can then use the index of this pin in the 2D array to
// signal the correct interface to the main loop 
void ISR_VECT onDio0Rise() {
    for (int i = 0; i < INTERFACE_COUNT; i++) {
        if (digitalRead(interface_pins[i][5]) == HIGH) {
            fifo_push(&packet_rdy_interfaces, i);
        }
    }
}

sx126x::sx126x(uint8_t index, SPIClass* spi, bool tcxo, bool dio2_as_rf_switch, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy, int rxen) :
  RadioInterface(index),
    _spiSettings(8E6, MSBFIRST, SPI_MODE0), _spiModem(spi), _ss(ss),
    _sclk(sclk), _mosi(mosi), _miso(miso), _reset(reset), _dio0(dio0),
    _busy(busy), _rxen(rxen), _frequency(0), _txp(0), _sf(0x07), _bw(0x04),
    _cr(0x01), _ldro(0x00), _packetIndex(0), _implicitHeaderMode(0),
    _payloadLength(255), _crcMode(1), _fifo_tx_addr_ptr(0),
    _fifo_rx_addr_ptr(0), _preinit_done(false), _tcxo(tcxo),
    _dio2_as_rf_switch(dio2_as_rf_switch)
{
  // overide Stream timeout value
  setTimeout(0);
  // TODO, figure out why this has to be done. Using the index to reference the
  // interface_obj list causes a crash otherwise
  _index = getIndex();
}

bool sx126x::preInit() {
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);
  
  // todo: check if this change causes issues on any platforms
  #if MCU_VARIANT == MCU_ESP32
  if (_sclk != -1 && _miso != -1 && _mosi != -1 && _ss != -1) {
    _spiModem->begin(_sclk, _miso, _mosi, _ss);
  } else {
    _spiModem->begin();
  }
  #else
    _spiModem->begin();
  #endif

  // check version (retry for up to 2 seconds)
  // TODO: Actually read version registers, not syncwords
  long start = millis();
  uint8_t syncmsb;
  uint8_t synclsb;
  while (((millis() - start) < 2000) && (millis() >= start)) {
      syncmsb = readRegister(REG_SYNC_WORD_MSB_6X);
      synclsb = readRegister(REG_SYNC_WORD_LSB_6X);
      if ( uint16_t(syncmsb << 8 | synclsb) == 0x1424 || uint16_t(syncmsb << 8 | synclsb) == 0x4434) {
          break;
      }
      delay(100);
  }
  if ( uint16_t(syncmsb << 8 | synclsb) != 0x1424 && uint16_t(syncmsb << 8 | synclsb) != 0x4434) {
      return false;
  }

  _preinit_done = true;
  return true;
}

uint8_t ISR_VECT sx126x::readRegister(uint16_t address)
{
  return singleTransfer(OP_READ_REGISTER_6X, address, 0x00);
}

void sx126x::writeRegister(uint16_t address, uint8_t value)
{
    singleTransfer(OP_WRITE_REGISTER_6X, address, value);
}

uint8_t ISR_VECT sx126x::singleTransfer(uint8_t opcode, uint16_t address, uint8_t value)
{
    waitOnBusy();

    uint8_t response;

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(opcode);
    _spiModem->transfer((address & 0xFF00) >> 8);
    _spiModem->transfer(address & 0x00FF);
    if (opcode == OP_READ_REGISTER_6X) {
        _spiModem->transfer(0x00);
    }
    response = _spiModem->transfer(value);
    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);

    return response;
}

void sx126x::rxAntEnable()
{
  if (_rxen != -1) {
    digitalWrite(_rxen, HIGH);
  }
}

void sx126x::loraMode() {
    // enable lora mode on the SX1262 chip
    uint8_t mode = MODE_LONG_RANGE_MODE_6X;
    executeOpcode(OP_PACKET_TYPE_6X, &mode, 1);
}

void sx126x::waitOnBusy() {
    unsigned long time = millis();
    while (digitalRead(_busy) == HIGH)
    {
        if (millis() >= (time + 100)) {
            break;
        }
        // do nothing
    }
}

void sx126x::executeOpcode(uint8_t opcode, uint8_t *buffer, uint8_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(opcode);

    for (int i = 0; i < size; i++)
    {
        _spiModem->transfer(buffer[i]);
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx126x::executeOpcodeRead(uint8_t opcode, uint8_t *buffer, uint8_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(opcode);
    _spiModem->transfer(0x00);

    for (int i = 0; i < size; i++)
    {
        buffer[i] = _spiModem->transfer(0x00);
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx126x::writeBuffer(const uint8_t* buffer, size_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(OP_FIFO_WRITE_6X);
    _spiModem->transfer(_fifo_tx_addr_ptr);

    for (int i = 0; i < size; i++)
    {
        _spiModem->transfer(buffer[i]);
        _fifo_tx_addr_ptr++;
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx126x::readBuffer(uint8_t* buffer, size_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(OP_FIFO_READ_6X);
    _spiModem->transfer(_fifo_rx_addr_ptr);
    _spiModem->transfer(0x00);

    for (int i = 0; i < size; i++)
    {
        buffer[i] = _spiModem->transfer(0x00);
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx126x::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro) {
  // because there is no access to these registers on the sx1262, we have
  // to set all these parameters at once or not at all.
  uint8_t buf[8];

  buf[0] = sf;
  buf[1] = bw;
  buf[2] = cr; 
  // low data rate toggle
  buf[3] = ldro;
  // unused params in LoRa mode
  buf[4] = 0x00; 
  buf[5] = 0x00;
  buf[6] = 0x00;
  buf[7] = 0x00;

  executeOpcode(OP_MODULATION_PARAMS_6X, buf, 8);
}

void sx126x::setPacketParams(uint32_t preamble, uint8_t headermode, uint8_t length, uint8_t crc) {
  // because there is no access to these registers on the sx1262, we have
  // to set all these parameters at once or not at all.
  uint8_t buf[9];

  buf[0] = uint8_t((preamble & 0xFF00) >> 8);
  buf[1] = uint8_t((preamble & 0x00FF));
  buf[2] = headermode;
  buf[3] = length;
  buf[4] = crc;
  // standard IQ setting (no inversion)
  buf[5] = 0x00; 
  // unused params
  buf[6] = 0x00; 
  buf[7] = 0x00; 
  buf[8] = 0x00; 

  executeOpcode(OP_PACKET_PARAMS_6X, buf, 9);
}

void sx126x::reset(void) {
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }
}

void sx126x::calibrate(void) {
  // Put in STDBY_RC mode before calibration
  uint8_t mode_byte = MODE_STDBY_RC_6X;
  executeOpcode(OP_STANDBY_6X, &mode_byte, 1);

  // calibrate RC64k, RC13M, PLL, ADC and image
  uint8_t calibrate = MASK_CALIBRATE_ALL;
  executeOpcode(OP_CALIBRATE_6X, &calibrate, 1);

  delay(5);
  waitOnBusy();
}

void sx126x::calibrate_image(uint32_t frequency) {
  uint8_t image_freq[2] = {0};

  if (frequency >= 430E6 && frequency <= 440E6) {
    image_freq[0] = 0x6B;
    image_freq[1] = 0x6F;
  }
  else if (frequency >= 470E6 && frequency <= 510E6) {
    image_freq[0] = 0x75;
    image_freq[1] = 0x81;
  }
  else if (frequency >= 779E6 && frequency <= 787E6) {
    image_freq[0] = 0xC1;
    image_freq[1] = 0xC5;
  }
  else if (frequency >= 863E6 && frequency <= 870E6) {
    image_freq[0] = 0xD7;
    image_freq[1] = 0xDB;
  }
  else if (frequency >= 902E6 && frequency <= 928E6) {
    image_freq[0] = 0xE1;
    image_freq[1] = 0xE9;
  }

  executeOpcode(OP_CALIBRATE_IMAGE_6X, image_freq, 2);
  waitOnBusy();
}

int sx126x::begin()
{
  reset();

  if (_busy != -1) {
      pinMode(_busy, INPUT);
  }

  if (!_preinit_done) {
    if (!preInit()) {
      return false;
    }
  }

  if (_rxen != -1) {
      pinMode(_rxen, OUTPUT);
  }

  calibrate();
  calibrate_image(_frequency);

  enableTCXO();

  loraMode();
  standby();

  // Set sync word
  setSyncWord(SYNC_WORD_6X);

  if (_dio2_as_rf_switch) {
    // enable dio2 rf switch
    uint8_t byte = 0x01;
    executeOpcode(OP_DIO2_RF_CTRL_6X, &byte, 1);
  }

  rxAntEnable();

  setFrequency(_frequency);

  setTxPower(_txp);
  enableCrc();

  // set LNA boost
  writeRegister(REG_LNA_6X, 0x96);

  // set base addresses
  uint8_t basebuf[2] = {0};
  executeOpcode(OP_BUFFER_BASE_ADDR_6X, basebuf, 2);

  setModulationParams(_sf, _bw, _cr, _ldro);
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  _radio_online = true;
  return 1;
}

void sx126x::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  _spiModem->end();

  _bitrate = 0;

  _radio_online = false;

  _preinit_done = false;
}

int sx126x::beginPacket(int implicitHeader)
{
  standby();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  _payloadLength = 0;
  _fifo_tx_addr_ptr = 0;
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  return 1;
}

int sx126x::endPacket()
{
      setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

      // put in single TX mode
      uint8_t timeout[3] = {0};
      executeOpcode(OP_TX_6X, timeout, 3);

      uint8_t buf[2];

      buf[0] = 0x00;
      buf[1] = 0x00;

      executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);

      // wait for TX done
      while ((buf[1] & IRQ_TX_DONE_MASK_6X) == 0) {
        buf[0] = 0x00;
        buf[1] = 0x00;
        executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);
        yield();
      }

      // clear IRQ's

      uint8_t mask[2];
      mask[0] = 0x00;
      mask[1] = IRQ_TX_DONE_MASK_6X;
      executeOpcode(OP_CLEAR_IRQ_STATUS_6X, mask, 2);
  return 1;
}

uint8_t sx126x::modemStatus() {
    // imitate the register status from the sx1276 / 78
    uint8_t buf[2] = {0};

    executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);
    uint8_t clearbuf[2] = {0};
    uint8_t byte = 0x00;

    if ((buf[1] & IRQ_PREAMBLE_DET_MASK_6X) != 0) {
      byte = byte | 0x01 | 0x04;
      // clear register after reading
      clearbuf[1] = IRQ_PREAMBLE_DET_MASK_6X;
    }

    if ((buf[1] & IRQ_HEADER_DET_MASK_6X) != 0) {
      byte = byte | 0x02 | 0x04;
    }

    executeOpcode(OP_CLEAR_IRQ_STATUS_6X, clearbuf, 2);

    return byte; 
}


uint8_t sx126x::currentRssiRaw() {
    uint8_t byte = 0;
    executeOpcodeRead(OP_CURRENT_RSSI_6X, &byte, 1);
    return byte;
}

int ISR_VECT sx126x::currentRssi() {
    uint8_t byte = 0;
    executeOpcodeRead(OP_CURRENT_RSSI_6X, &byte, 1);
    int rssi = -(int(byte)) / 2;
    return rssi;
}

uint8_t sx126x::packetRssiRaw() {
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    return buf[2];
}

int ISR_VECT sx126x::packetRssi() {
    // may need more calculations here
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    int pkt_rssi = -buf[0] / 2;
    return pkt_rssi;
}

uint8_t ISR_VECT sx126x::packetSnrRaw() {
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    return buf[1];
}

float ISR_VECT sx126x::packetSnr() {
    uint8_t buf[3] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_6X, buf, 3);
    return float(buf[1]) * 0.25;
}

long sx126x::packetFrequencyError()
{
    // todo: implement this, no idea how to check it on the sx1262
    const float fError = 0.0;
    return static_cast<long>(fError);
}

size_t sx126x::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t sx126x::write(const uint8_t *buffer, size_t size)
{
    if ((_payloadLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - _payloadLength;
    }

    // write data
    writeBuffer(buffer, size);
    _payloadLength = _payloadLength + size;
    return size;
}

int ISR_VECT sx126x::available()
{
    uint8_t buf[2] = {0};
    executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, buf, 2);
    return buf[0] - _packetIndex;
}

int ISR_VECT sx126x::read()
{
  if (!available()) {
    return -1;
  }

  // if received new packet
  if (_packetIndex == 0) {
      uint8_t rxbuf[2] = {0};
      executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, rxbuf, 2);
      int size = rxbuf[0];
      _fifo_rx_addr_ptr = rxbuf[1];

      readBuffer(_packet, size);
  }

  uint8_t byte = _packet[_packetIndex];
  _packetIndex++;
  return byte;
}

int sx126x::peek()
{
  if (!available()) {
    return -1;
  }

  // if received new packet
  if (_packetIndex == 0) {
      uint8_t rxbuf[2] = {0};
      executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, rxbuf, 2);
      int size = rxbuf[0];
      _fifo_rx_addr_ptr = rxbuf[1];

      readBuffer(_packet, size);
  }

  uint8_t b = _packet[_packetIndex];
  return b;
}

void sx126x::flush()
{
}

void sx126x::onReceive(void(*callback)(uint8_t, int))
{
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);

    // set preamble and header detection irqs, plus dio0 mask
    uint8_t buf[8];

    // set irq masks, enable all
    buf[0] = 0xFF; 
    buf[1] = 0xFF;

    // set dio0 masks
    buf[2] = 0x00;
    buf[3] = IRQ_RX_DONE_MASK_6X; 

    // set dio1 masks
    buf[4] = 0x00; 
    buf[5] = 0x00;

    // set dio2 masks
    buf[6] = 0x00; 
    buf[7] = 0x00;

    executeOpcode(OP_SET_IRQ_FLAGS_6X, buf, 8);
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    _spiModem->usingInterrupt(digitalPinToInterrupt(_dio0));
#endif
    // make function available
    extern void onDio0Rise();

    attachInterrupt(digitalPinToInterrupt(_dio0), onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
#ifdef SPI_HAS_NOTUSINGINTERRUPT
    _spiModem->notUsingInterrupt(digitalPinToInterrupt(_dio0));
#endif
  }
}

void sx126x::receive(int size)
{
    if (size > 0) {
        implicitHeaderMode();

        // tell radio payload length
        _payloadLength = size;
        setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
    } else {
        explicitHeaderMode();
    }

    if (_rxen != -1) {
        rxAntEnable();
    }

    uint8_t mode[3] = {0xFF, 0xFF, 0xFF}; // continuous mode
    executeOpcode(OP_RX_6X, mode, 3);
}

void sx126x::standby()
{
  uint8_t byte;
  if (_tcxo) {
  // STDBY_XOSC
  byte = MODE_STDBY_XOSC_6X;
  } else {
  // STDBY_RC
  byte = MODE_STDBY_RC_6X;
  }
  executeOpcode(OP_STANDBY_6X, &byte, 1); 
}

void sx126x::sleep()
{
    uint8_t byte = 0x00;
    executeOpcode(OP_SLEEP_6X, &byte, 1);
}

void sx126x::enableTCXO() {
  if (_tcxo) {
    #if BOARD_MODEL == BOARD_RAK4631 || BOARD_MODEL == BOARD_HELTEC32_V3
      uint8_t buf[4] = {MODE_TCXO_3_3V_6X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_TBEAM
      uint8_t buf[4] = {MODE_TCXO_1_8V_6X, 0x00, 0x00, 0xFF};
    #elif BOARD_MODEL == BOARD_T3S3
      uint8_t buf[4] = {MODE_TCXO_1_8V_6X, 0x00, 0x00, 0xFF};
    #else
      uint8_t buf[4] = {0};
    #endif
    executeOpcode(OP_DIO3_TCXO_CTRL_6X, buf, 4);
  }
}

// TODO: Once enabled, SX1262 needs a complete reset to disable TCXO
void sx126x::disableTCXO() { }

void sx126x::setTxPower(int level, int outputPin) {
    // currently no low power mode for SX1262 implemented, assuming PA boost
    
    // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
    // RegTxClampConfig = @address 0x08D8
    writeRegister(0x08D8, readRegister(0x08D8) | (0x0F << 1));

    uint8_t pa_buf[4];

    pa_buf[0] = 0x04; // PADutyCycle needs to be 0x04 to achieve 22dBm output, but can be lowered for better efficiency at lower outputs
    pa_buf[1] = 0x07; // HPMax at 0x07 is maximum supported for SX1262
    pa_buf[2] = 0x00; // DeviceSel 0x00 for SX1262 (0x01 for SX1261)
    pa_buf[3] = 0x01; // PALut always 0x01 (reserved according to datasheet)

    executeOpcode(OP_PA_CONFIG_6X, pa_buf, 4); // set pa_config for high power

    if (level > 22) { level = 22; }
    else if (level < -9) { level = -9; }

    _txp = level;

    writeRegister(REG_OCP_6X, 0x38); // 160mA limit, overcurrent protection

    uint8_t tx_buf[2];

    tx_buf[0] = level;
    tx_buf[1] = 0x02; // PA ramping time - 40 microseconds
    
    executeOpcode(OP_TX_PARAMS_6X, tx_buf, 2);
}

uint8_t sx126x::getTxPower() {
    return _txp;
}

void sx126x::setFrequency(uint32_t frequency) {
  _frequency = frequency;

  uint8_t buf[4];

  uint32_t freq = (uint32_t)((double)frequency / (double)FREQ_STEP_6X);

  buf[0] = ((freq >> 24) & 0xFF);
  buf[1] = ((freq >> 16) & 0xFF);
  buf[2] = ((freq >> 8) & 0xFF);
  buf[3] = (freq & 0xFF);

  executeOpcode(OP_RF_FREQ_6X, buf, 4);
}

uint32_t sx126x::getFrequency() {
    // we can't read the frequency on the sx1262 / 80
    uint32_t frequency = _frequency;

    return frequency;
}

void sx126x::setSpreadingFactor(int sf)
{
  if (sf < 5) {
      sf = 5;
  } else if (sf > 12) {
    sf = 12;
  }

  _sf = sf;

  handleLowDataRate();
  setModulationParams(sf, _bw, _cr, _ldro);
}

uint8_t sx126x::getSpreadingFactor()
{
    return _sf;
}

uint32_t sx126x::getSignalBandwidth()
{
    int bw = _bw;
    switch (bw) {
        case 0x00: return 7.8E3;
        case 0x01: return 15.6E3;
        case 0x02: return 31.25E3;
        case 0x03: return 62.5E3;
        case 0x04: return 125E3;
        case 0x05: return 250E3;
        case 0x06: return 500E3;
        case 0x08: return 10.4E3;
        case 0x09: return 20.8E3;
        case 0x0A: return 41.7E3;
    }
  return 0;
}

void sx126x::handleLowDataRate(){
  if ( long( (1<<_sf) / (getSignalBandwidth()/1000)) > 16) {
    _ldro = 0x01;
  } else {
    _ldro = 0x00;
  }
}

void sx126x::optimizeModemSensitivity(){
    // todo: check if there's anything the sx1262 can do here
}

void sx126x::setSignalBandwidth(uint32_t sbw)
{
  if (sbw <= 7.8E3) {
      _bw = 0x00;
  } else if (sbw <= 10.4E3) {
      _bw = 0x08;
  } else if (sbw <= 15.6E3) {
      _bw = 0x01;
  } else if (sbw <= 20.8E3) {
      _bw = 0x09;
  } else if (sbw <= 31.25E3) {
      _bw = 0x02;
  } else if (sbw <= 41.7E3) {
      _bw = 0x0A;
  } else if (sbw <= 62.5E3) {
      _bw = 0x03;
  } else if (sbw <= 125E3) {
      _bw = 0x04;
  } else if (sbw <= 250E3) {
      _bw = 0x05;
  } else /*if (sbw <= 250E3)*/ {
      _bw = 0x06;
  }

  handleLowDataRate();
  setModulationParams(_sf, _bw, _cr, _ldro);

  optimizeModemSensitivity();
}

void sx126x::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  int cr = denominator - 4;

  _cr = cr;

  setModulationParams(_sf, _bw, cr, _ldro);
}

uint8_t sx126x::getCodingRate4()
{
    return _cr + 4;
}

void sx126x::setPreambleLength(long length)
{
  _preambleLength = length;
  setPacketParams(length, _implicitHeaderMode, _payloadLength, _crcMode);
}

void sx126x::setSyncWord(uint16_t sw)
{
  // TODO: Fix
    // writeRegister(REG_SYNC_WORD_MSB_6X, (sw & 0xFF00) >> 8);
    // writeRegister(REG_SYNC_WORD_LSB_6X, sw & 0x00FF);
    writeRegister(REG_SYNC_WORD_MSB_6X, 0x14);
    writeRegister(REG_SYNC_WORD_LSB_6X, 0x24);
}

void sx126x::enableCrc()
{
    _crcMode = 1;
    setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void sx126x::disableCrc()
{
    _crcMode = 0;
    setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

byte sx126x::random()
{
    return readRegister(REG_RANDOM_GEN_6X);
}

void sx126x::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void sx126x::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void sx126x::explicitHeaderMode()
{
  _implicitHeaderMode = 0;
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void sx126x::implicitHeaderMode()
{
  _implicitHeaderMode = 1;
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}


void sx126x::handleDio0Rise()
{
    uint8_t buf[2];

    buf[0] = 0x00;
    buf[1] = 0x00;

    executeOpcodeRead(OP_GET_IRQ_STATUS_6X, buf, 2);

    executeOpcode(OP_CLEAR_IRQ_STATUS_6X, buf, 2);

    if ((buf[1] & IRQ_PAYLOAD_CRC_ERROR_MASK_6X) == 0) {
        // received a packet
        _packetIndex = 0;

        // read packet length
        uint8_t rxbuf[2] = {0};
        executeOpcodeRead(OP_RX_BUFFER_STATUS_6X, rxbuf, 2);
        int packetLength = rxbuf[0];

        if (_onReceive) {
            _onReceive(_index, packetLength);
        }
    }
    // else {
    //   Serial.println("CRCE");
    //   Serial.println(buf[0]);
    //   Serial.println(buf[1]);
    // }
}

void sx126x::updateBitrate() {
    if (_radio_online) {
        _lora_symbol_rate = (float)getSignalBandwidth()/(float)(pow(2, _sf));
        _lora_symbol_time_ms = (1.0/_lora_symbol_rate)*1000.0;
        _bitrate = (uint32_t)(_sf * ( (4.0/(float)(_cr+4)) / ((float)(pow(2, _sf))/((float)getSignalBandwidth()/1000.0)) ) * 1000.0);
        _lora_us_per_byte = 1000000.0/((float)_bitrate/8.0);
        //_csma_slot_ms = _lora_symbol_time_ms*10;
        float target_preamble_symbols = (LORA_PREAMBLE_TARGET_MS/_lora_symbol_time_ms)-LORA_PREAMBLE_SYMBOLS_HW;
        if (target_preamble_symbols < LORA_PREAMBLE_SYMBOLS_MIN) {
            target_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN;
        } else {
            target_preamble_symbols = ceil(target_preamble_symbols);
        }
        _preambleLength = (long)target_preamble_symbols;
        setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
    } else {
        _bitrate = 0;
    }
}

// SX127x registers
#define REG_FIFO_7X                   0x00
#define REG_OP_MODE_7X                0x01
#define REG_FRF_MSB_7X                0x06
#define REG_FRF_MID_7X                0x07
#define REG_FRF_LSB_7X                0x08
#define REG_PA_CONFIG_7X              0x09
#define REG_OCP_7X                    0x0b
#define REG_LNA_7X                    0x0c
#define REG_FIFO_ADDR_PTR_7X          0x0d
#define REG_FIFO_TX_BASE_ADDR_7X      0x0e
#define REG_FIFO_RX_BASE_ADDR_7X      0x0f
#define REG_FIFO_RX_CURRENT_ADDR_7X   0x10
#define REG_IRQ_FLAGS_7X              0x12
#define REG_RX_NB_BYTES_7X            0x13
#define REG_MODEM_STAT_7X             0x18
#define REG_PKT_SNR_VALUE_7X          0x19
#define REG_PKT_RSSI_VALUE_7X         0x1a
#define REG_RSSI_VALUE_7X             0x1b
#define REG_MODEM_CONFIG_1_7X         0x1d
#define REG_MODEM_CONFIG_2_7X         0x1e
#define REG_PREAMBLE_MSB_7X           0x20
#define REG_PREAMBLE_LSB_7X           0x21
#define REG_PAYLOAD_LENGTH_7X         0x22
#define REG_MODEM_CONFIG_3_7X         0x26
#define REG_FREQ_ERROR_MSB_7X         0x28
#define REG_FREQ_ERROR_MID_7X         0x29
#define REG_FREQ_ERROR_LSB_7X         0x2a
#define REG_RSSI_WIDEBAND_7X          0x2c
#define REG_DETECTION_OPTIMIZE_7X     0x31
#define REG_HIGH_BW_OPTIMIZE_1_7X     0x36
#define REG_DETECTION_THRESHOLD_7X    0x37
#define REG_SYNC_WORD_7X              0x39
#define REG_HIGH_BW_OPTIMIZE_2_7X     0x3a
#define REG_DIO_MAPPING_1_7X          0x40
#define REG_VERSION_7X                0x42
#define REG_TCXO_7X                   0x4b
#define REG_PA_DAC_7X                 0x4d

// Modes
#define MODE_LONG_RANGE_MODE_7X       0x80
#define MODE_SLEEP_7X                 0x00
#define MODE_STDBY_7X                 0x01
#define MODE_TX_7X                    0x03
#define MODE_RX_CONTINUOUS_7X         0x05
#define MODE_RX_SINGLE_7X             0x06

// PA config
#define PA_BOOST_7X                   0x80

// IRQ masks
#define IRQ_TX_DONE_MASK_7X           0x08
#define IRQ_RX_DONE_MASK_7X           0x40
#define IRQ_PAYLOAD_CRC_ERROR_MASK_7X 0x20

#define SYNC_WORD_7X                  0x12

sx127x::sx127x(uint8_t index, SPIClass* spi, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy) :
  RadioInterface(index),
    _spiSettings(8E6, MSBFIRST, SPI_MODE0),
    _spiModem(spi),
  _ss(ss), _sclk(sclk), _mosi(mosi), _miso(miso),  _reset(reset), _dio0(dio0),
  _busy(busy), _frequency(0), _packetIndex(0), _preinit_done(false)
{ 
    setTimeout(0); 
    // TODO, figure out why this has to be done. Using the index to reference the
    // interface_obj list causes a crash otherwise
    _index = getIndex();
}

void sx127x::setSPIFrequency(uint32_t frequency) { _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0); }
uint8_t ISR_VECT sx127x::readRegister(uint8_t address) { return singleTransfer(address & 0x7f, 0x00); }
void sx127x::writeRegister(uint8_t address, uint8_t value) { singleTransfer(address | 0x80, value); }
void sx127x::standby() { writeRegister(REG_OP_MODE_7X, MODE_LONG_RANGE_MODE_7X | MODE_STDBY_7X); }
void sx127x::sleep() { writeRegister(REG_OP_MODE_7X, MODE_LONG_RANGE_MODE_7X | MODE_SLEEP_7X); }
uint8_t sx127x::modemStatus() { return readRegister(REG_MODEM_STAT_7X); }
void sx127x::setSyncWord(uint8_t sw) { writeRegister(REG_SYNC_WORD_7X, sw); }
void sx127x::enableCrc() { writeRegister(REG_MODEM_CONFIG_2_7X, readRegister(REG_MODEM_CONFIG_2_7X) | 0x04); }
void sx127x::disableCrc() { writeRegister(REG_MODEM_CONFIG_2_7X, readRegister(REG_MODEM_CONFIG_2_7X) & 0xfb); }
void sx127x::enableTCXO() { uint8_t tcxo_reg = readRegister(REG_TCXO_7X); writeRegister(REG_TCXO_7X, tcxo_reg | 0x10); }
void sx127x::disableTCXO() { uint8_t tcxo_reg = readRegister(REG_TCXO_7X); writeRegister(REG_TCXO_7X, tcxo_reg & 0xEF); }
void sx127x::explicitHeaderMode() { _implicitHeaderMode = 0; writeRegister(REG_MODEM_CONFIG_1_7X, readRegister(REG_MODEM_CONFIG_1_7X) & 0xfe); }
void sx127x::implicitHeaderMode() { _implicitHeaderMode = 1; writeRegister(REG_MODEM_CONFIG_1_7X, readRegister(REG_MODEM_CONFIG_1_7X) | 0x01); }
byte sx127x::random() { return readRegister(REG_RSSI_WIDEBAND_7X); }
void sx127x::flush() { }

bool sx127x::preInit() {
  pinMode(_ss, OUTPUT);
  digitalWrite(_ss, HIGH);
  // todo: check if this change causes issues on any platforms
  #if MCU_VARIANT == MCU_ESP32
  if (_sclk != -1 && _miso != -1 && _mosi != -1 && _ss != -1) {
    _spiModem->begin(_sclk, _miso, _mosi, _ss);
  } else {
    _spiModem->begin();
  }
  #else
    _spiModem->begin();
  #endif

  // Check modem version
  uint8_t version;
  long start = millis();
  while (((millis() - start) < 500) && (millis() >= start)) {
      version = readRegister(REG_VERSION_7X);
      if (version == 0x12) { break; }
      delay(100);
  }

  if (version != 0x12) { return false; }

  _preinit_done = true;
  return true;
}

uint8_t ISR_VECT sx127x::singleTransfer(uint8_t address, uint8_t value) {
  uint8_t response;

  digitalWrite(_ss, LOW);
  _spiModem->beginTransaction(_spiSettings);
  _spiModem->transfer(address);
  response = _spiModem->transfer(value);
  _spiModem->endTransaction();
  digitalWrite(_ss, HIGH);

  return response;
}

int sx127x::begin() {
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // Perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  if (_busy != -1) { pinMode(_busy, INPUT); }

  if (!_preinit_done) {
    if (!preInit()) { return false; }
  }

  sleep();
  setFrequency(_frequency);

  // set base addresses
  writeRegister(REG_FIFO_TX_BASE_ADDR_7X, 0);
  writeRegister(REG_FIFO_RX_BASE_ADDR_7X, 0);

  // set LNA boost and auto AGC
  writeRegister(REG_LNA_7X, readRegister(REG_LNA_7X) | 0x03);
  writeRegister(REG_MODEM_CONFIG_3_7X, 0x04);

  setSyncWord(SYNC_WORD_7X);
  enableCrc();
  setTxPower(2);

  standby();

  _radio_online = true;

  return 1;
}

void sx127x::end() {
  sleep();
  _spiModem->end();
  _bitrate = 0;
  _radio_online = false;
  _preinit_done = false;
}

int sx127x::beginPacket(int implicitHeader) {
  standby();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  // Reset FIFO address and payload length
  writeRegister(REG_FIFO_ADDR_PTR_7X, 0);
  writeRegister(REG_PAYLOAD_LENGTH_7X, 0);

  return 1;
}

int sx127x::endPacket() {
  // Enter TX mode
  writeRegister(REG_OP_MODE_7X, MODE_LONG_RANGE_MODE_7X | MODE_TX_7X);

  // Wait for TX completion
  while ((readRegister(REG_IRQ_FLAGS_7X) & IRQ_TX_DONE_MASK_7X) == 0) {
    yield();
  }

  // Clear TX complete IRQ
  writeRegister(REG_IRQ_FLAGS_7X, IRQ_TX_DONE_MASK_7X);
  return 1;
}

uint8_t sx127x::currentRssiRaw() {
    uint8_t rssi = readRegister(REG_RSSI_VALUE_7X);
    return rssi;
}

int ISR_VECT sx127x::currentRssi() {
    int rssi = (int)readRegister(REG_RSSI_VALUE_7X) - RSSI_OFFSET;
    if (_frequency < 820E6) rssi -= 7;
    return rssi;
}

uint8_t sx127x::packetRssiRaw() {
    uint8_t pkt_rssi_value = readRegister(REG_PKT_RSSI_VALUE_7X);
    return pkt_rssi_value;
}

int ISR_VECT sx127x::packetRssi() {
    int pkt_rssi = (int)readRegister(REG_PKT_RSSI_VALUE_7X) - RSSI_OFFSET;
    int pkt_snr = packetSnr();

    if (_frequency < 820E6) pkt_rssi -= 7;

    if (pkt_snr < 0) {
        pkt_rssi += pkt_snr;
    } else {
        // Slope correction is (16/15)*pkt_rssi,
        // this estimation looses one floating point
        // operation, and should be precise enough.
        pkt_rssi = (int)(1.066 * pkt_rssi);
    }
    return pkt_rssi;
}

uint8_t ISR_VECT sx127x::packetSnrRaw() {
    return readRegister(REG_PKT_SNR_VALUE_7X);
}

float ISR_VECT sx127x::packetSnr() {
    return ((int8_t)readRegister(REG_PKT_SNR_VALUE_7X)) * 0.25;
}

long sx127x::packetFrequencyError() {
  int32_t freqError = 0;
  freqError = static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MSB_7X) & B111);
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_MID_7X));
  freqError <<= 8L;
  freqError += static_cast<int32_t>(readRegister(REG_FREQ_ERROR_LSB_7X));

  if (readRegister(REG_FREQ_ERROR_MSB_7X) & B1000) { // Sign bit is on
      freqError -= 524288; // B1000'0000'0000'0000'0000
  }

  const float fXtal = 32E6; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
  const float fError = ((static_cast<float>(freqError) * (1L << 24)) / fXtal) * (getSignalBandwidth() / 500000.0f);

  return static_cast<long>(fError);
}

size_t sx127x::write(uint8_t byte) { return write(&byte, sizeof(byte)); }

size_t sx127x::write(const uint8_t *buffer, size_t size) {
    int currentLength = readRegister(REG_PAYLOAD_LENGTH_7X);
    if ((currentLength + size) > MAX_PKT_LENGTH) {
        size = MAX_PKT_LENGTH - currentLength;
    }

    for (size_t i = 0; i < size; i++) {
        writeRegister(REG_FIFO_7X, buffer[i]);
    }

    writeRegister(REG_PAYLOAD_LENGTH_7X, currentLength + size);
    return size;
}

int ISR_VECT sx127x::available() { return (readRegister(REG_RX_NB_BYTES_7X) - _packetIndex); }

int ISR_VECT sx127x::read() {
  if (!available()) { return -1; }
  _packetIndex++;
  return readRegister(REG_FIFO_7X);
}

int sx127x::peek() {
  if (!available()) { return -1; }

  // Remember current FIFO address, read, and then reset address
  int currentAddress = readRegister(REG_FIFO_ADDR_PTR_7X);
  uint8_t b = readRegister(REG_FIFO_7X);
  writeRegister(REG_FIFO_ADDR_PTR_7X, currentAddress);

  return b;
}

void sx127x::onReceive(void(*callback)(uint8_t, int)) {
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);
    writeRegister(REG_DIO_MAPPING_1_7X, 0x00);
    
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
      _spiModem->usingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
    
    // make function available
    extern void onDio0Rise();

    attachInterrupt(digitalPinToInterrupt(_dio0), onDio0Rise, RISING);
  
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
    
    #ifdef SPI_HAS_NOTUSINGINTERRUPT
      _spiModem->notUsingInterrupt(digitalPinToInterrupt(_dio0));
    #endif
  }
}

void sx127x::receive(int size) {
  if (size > 0) {
    implicitHeaderMode();
    writeRegister(REG_PAYLOAD_LENGTH_7X, size & 0xff);
  } else { explicitHeaderMode(); }

  writeRegister(REG_OP_MODE_7X, MODE_LONG_RANGE_MODE_7X | MODE_RX_CONTINUOUS_7X);
}

void sx127x::setTxPower(int level, int outputPin) {
  // Setup according to RFO or PA_BOOST output pin
  if (PA_OUTPUT_RFO_PIN == outputPin) {
    if (level < 0) { level = 0; }
    else if (level > 14) { level = 14; }

    writeRegister(REG_PA_DAC_7X, 0x84);
    writeRegister(REG_PA_CONFIG_7X, 0x70 | level);

  } else {
    if (level < 2) { level = 2; }
    else if (level > 17) { level = 17; }

    writeRegister(REG_PA_DAC_7X, 0x84);
    writeRegister(REG_PA_CONFIG_7X, PA_BOOST_7X | (level - 2));
  }
}

uint8_t sx127x::getTxPower() { byte txp = readRegister(REG_PA_CONFIG_7X); return txp; }

void sx127x::setFrequency(uint32_t frequency) {
  _frequency = frequency;
  uint32_t frf = ((uint64_t)frequency << 19) / 32000000;

  writeRegister(REG_FRF_MSB_7X, (uint8_t)(frf >> 16));
  writeRegister(REG_FRF_MID_7X, (uint8_t)(frf >> 8));
  writeRegister(REG_FRF_LSB_7X, (uint8_t)(frf >> 0));

  optimizeModemSensitivity();
}

uint32_t sx127x::getFrequency() {
  uint8_t msb = readRegister(REG_FRF_MSB_7X);
  uint8_t mid = readRegister(REG_FRF_MID_7X);
  uint8_t lsb = readRegister(REG_FRF_LSB_7X);

  uint32_t frf = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | (uint32_t)lsb;
  uint64_t frm = (uint64_t)frf*32000000;
  uint32_t frequency = (frm >> 19);

  return frequency;
}

void sx127x::setSpreadingFactor(int sf) {
  if (sf < 6) { sf = 6; }
  else if (sf > 12) { sf = 12; }

  if (sf == 6) {
    writeRegister(REG_DETECTION_OPTIMIZE_7X, 0xc5);
    writeRegister(REG_DETECTION_THRESHOLD_7X, 0x0c);
  } else {
    writeRegister(REG_DETECTION_OPTIMIZE_7X, 0xc3);
    writeRegister(REG_DETECTION_THRESHOLD_7X, 0x0a);
  }

  _sf = sf;

  writeRegister(REG_MODEM_CONFIG_2_7X, (readRegister(REG_MODEM_CONFIG_2_7X) & 0x0f) | ((sf << 4) & 0xf0));
  handleLowDataRate();
}

uint8_t sx127x::getSpreadingFactor()
{
    return _sf;
}

uint32_t sx127x::getSignalBandwidth() {
  byte bw = (readRegister(REG_MODEM_CONFIG_1_7X) >> 4);
  switch (bw) {
    case 0: return 7.8E3;
    case 1: return 10.4E3;
    case 2: return 15.6E3;
    case 3: return 20.8E3;
    case 4: return 31.25E3;
    case 5: return 41.7E3;
    case 6: return 62.5E3;
    case 7: return 125E3;
    case 8: return 250E3;
    case 9: return 500E3; }

  return 0;
}

void sx127x::setSignalBandwidth(uint32_t sbw) {
  int bw;
  if (sbw <= 7.8E3) {
      bw = 0;
  } else if (sbw <= 10.4E3) {
      bw = 1;
  } else if (sbw <= 15.6E3) {
      bw = 2;
  } else if (sbw <= 20.8E3) {
      bw = 3;
  } else if (sbw <= 31.25E3) {
      bw = 4;
  } else if (sbw <= 41.7E3) {
      bw = 5;
  } else if (sbw <= 62.5E3) {
      bw = 6;
  } else if (sbw <= 125E3) {
      bw = 7;
  } else if (sbw <= 250E3) {
      bw = 8;
  } else /*if (sbw <= 250E3)*/ {
      bw = 9;
  }

  writeRegister(REG_MODEM_CONFIG_1_7X, (readRegister(REG_MODEM_CONFIG_1_7X) & 0x0f) | (bw << 4));
  handleLowDataRate();
  optimizeModemSensitivity();
}

void sx127x::setCodingRate4(int denominator) {
  if (denominator < 5) { denominator = 5; }
  else if (denominator > 8) { denominator = 8; }
  int cr = denominator - 4;
  _cr = cr;
  writeRegister(REG_MODEM_CONFIG_1_7X, (readRegister(REG_MODEM_CONFIG_1_7X) & 0xf1) | (cr << 1));
}

uint8_t sx127x::getCodingRate4()
{
    return _cr + 4;
}

void sx127x::setPreambleLength(long length) { 
  _preambleLength = length;
  writeRegister(REG_PREAMBLE_MSB_7X, (uint8_t)(length >> 8));
  writeRegister(REG_PREAMBLE_LSB_7X, (uint8_t)(length >> 0));
}

void sx127x::handleLowDataRate() {
  int sf = (readRegister(REG_MODEM_CONFIG_2_7X) >> 4);
  if ( long( (1<<sf) / (getSignalBandwidth()/1000)) > 16) {
    // Set auto AGC and LowDataRateOptimize
    writeRegister(REG_MODEM_CONFIG_3_7X, (1<<3)|(1<<2));
  } else {
    // Only set auto AGC
    writeRegister(REG_MODEM_CONFIG_3_7X, (1<<2));
  }
}

void sx127x::optimizeModemSensitivity() {
  byte bw = (readRegister(REG_MODEM_CONFIG_1_7X) >> 4);
  uint32_t freq = getFrequency();

  if (bw == 9 && (410E6 <= freq) && (freq <= 525E6)) {
    writeRegister(REG_HIGH_BW_OPTIMIZE_1_7X, 0x02);
    writeRegister(REG_HIGH_BW_OPTIMIZE_2_7X, 0x7f);
  } else if (bw == 9 && (820E6 <= freq) && (freq <= 1020E6)) {
    writeRegister(REG_HIGH_BW_OPTIMIZE_1_7X, 0x02);
    writeRegister(REG_HIGH_BW_OPTIMIZE_2_7X, 0x64);
  } else {
    writeRegister(REG_HIGH_BW_OPTIMIZE_1_7X, 0x03);
  }
}

void sx127x::handleDio0Rise() {
  int irqFlags = readRegister(REG_IRQ_FLAGS_7X);

  // Clear IRQs
  writeRegister(REG_IRQ_FLAGS_7X, irqFlags);
  if ((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK_7X) == 0) {
    _packetIndex = 0;
    int packetLength = _implicitHeaderMode ? readRegister(REG_PAYLOAD_LENGTH_7X) : readRegister(REG_RX_NB_BYTES_7X);
    writeRegister(REG_FIFO_ADDR_PTR_7X, readRegister(REG_FIFO_RX_CURRENT_ADDR_7X));
    if (_onReceive) { 
        _onReceive(_index, packetLength); 
    }
    writeRegister(REG_FIFO_ADDR_PTR_7X, 0);
  }
}

void sx127x::updateBitrate() {
    if (_radio_online) {
        _lora_symbol_rate = (float)getSignalBandwidth()/(float)(pow(2, _sf));
        _lora_symbol_time_ms = (1.0/_lora_symbol_rate)*1000.0;
        _bitrate = (uint32_t)(_sf * ( (4.0/(float)(_cr+4)) / ((float)(pow(2, _sf))/((float)getSignalBandwidth()/1000.0)) ) * 1000.0);
        _lora_us_per_byte = 1000000.0/((float)_bitrate/8.0);
        //_csma_slot_ms = _lora_symbol_time_ms*10;
        float target_preamble_symbols = (LORA_PREAMBLE_TARGET_MS/_lora_symbol_time_ms)-LORA_PREAMBLE_SYMBOLS_HW;
        if (target_preamble_symbols < LORA_PREAMBLE_SYMBOLS_MIN) {
            target_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN;
        } else {
            target_preamble_symbols = ceil(target_preamble_symbols);
        }
        _preambleLength = (long)target_preamble_symbols;
    } else {
        _bitrate = 0;
    }
}

// SX128x registers
#define OP_RF_FREQ_8X               0x86
#define OP_SLEEP_8X                 0x84
#define OP_STANDBY_8X               0x80
#define OP_TX_8X                    0x83
#define OP_RX_8X                    0x82
#define OP_SET_IRQ_FLAGS_8X         0x8D // also provides info such as
                                         // preamble detection, etc for
                                         // knowing when it's safe to switch
                                         // antenna modes
#define OP_CLEAR_IRQ_STATUS_8X      0x97
#define OP_GET_IRQ_STATUS_8X        0x15
#define OP_RX_BUFFER_STATUS_8X      0x17
#define OP_PACKET_STATUS_8X         0x1D // get snr & rssi of last packet
#define OP_CURRENT_RSSI_8X          0x1F
#define OP_MODULATION_PARAMS_8X     0x8B // bw, sf, cr, etc.
#define OP_PACKET_PARAMS_8X         0x8C // crc, preamble, payload length, etc.
#define OP_STATUS_8X                0xC0
#define OP_TX_PARAMS_8X             0x8E // set dbm, etc
#define OP_PACKET_TYPE_8X           0x8A
#define OP_BUFFER_BASE_ADDR_8X      0x8F
#define OP_READ_REGISTER_8X         0x19
#define OP_WRITE_REGISTER_8X        0x18
#define IRQ_TX_DONE_MASK_8X         0x01
#define IRQ_RX_DONE_MASK_8X         0x02
#define IRQ_HEADER_DET_MASK_8X      0x10
#define IRQ_HEADER_ERROR_MASK_8X    0x20
#define IRQ_PAYLOAD_CRC_ERROR_MASK_8X 0x40

#define MODE_LONG_RANGE_MODE_8X     0x01

#define OP_FIFO_WRITE_8X            0x1A
#define OP_FIFO_READ_8X             0x1B
#define IRQ_PREAMBLE_DET_MASK_8X    0x80

#define REG_PACKET_SIZE            0x901
#define REG_FIRM_VER_MSB           0x154
#define REG_FIRM_VER_LSB           0x153

#define XTAL_FREQ_8X (double)52000000
#define FREQ_DIV_8X (double)pow(2.0, 18.0)
#define FREQ_STEP_8X (double)(XTAL_FREQ_8X / FREQ_DIV_8X)

sx128x::sx128x(uint8_t index, SPIClass* spi, bool tcxo, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy, int rxen, int txen) :
  RadioInterface(index),
    _spiSettings(8E6, MSBFIRST, SPI_MODE0),
    _spiModem(spi),
  _ss(ss), _sclk(sclk), _mosi(mosi), _miso(miso), _reset(reset), _dio0(dio0),
  _busy(busy), _rxen(rxen), _txen(txen), _frequency(0), _txp(0), _sf(0x05),
  _bw(0x34), _cr(0x01), _packetIndex(0), _implicitHeaderMode(0),
  _payloadLength(255), _crcMode(0), _fifo_tx_addr_ptr(0), _fifo_rx_addr_ptr(0),
  _rxPacketLength(0), _preinit_done(false),
  _tcxo(tcxo)
{
  // overide Stream timeout value
  setTimeout(0);
  // TODO, figure out why this has to be done. Using the index to reference the
  // interface_obj list causes a crash otherwise
  _index = getIndex();
}

bool sx128x::preInit() {
  // setup pins
  pinMode(_ss, OUTPUT);
  // set SS high
  digitalWrite(_ss, HIGH);
  
  // todo: check if this change causes issues on any platforms
  #if MCU_VARIANT == MCU_ESP32
  if (_sclk != -1 && _miso != -1 && _mosi != -1 && _ss != -1) {
    _spiModem->begin(_sclk, _miso, _mosi, _ss);
  } else {
    _spiModem->begin();
  }
  #else
    _spiModem->begin();
  #endif

  // check version (retry for up to 2 seconds)
  long start = millis();

  uint8_t version_msb;
  uint8_t version_lsb;

  while (((millis() - start) < 2000) && (millis() >= start)) {

      version_msb = readRegister(REG_FIRM_VER_MSB);
      version_lsb = readRegister(REG_FIRM_VER_LSB);

      if ((version_msb == 0xB7 && version_lsb == 0xA9) || (version_msb == 0xB5 && version_lsb == 0xA9)) {
          break;
      }
      delay(100);
  }
  if ((version_msb != 0xB7 || version_lsb != 0xA9) && (version_msb != 0xB5 || version_lsb != 0xA9)) {
      return false;
  }

  _preinit_done = true;
  return true;
}

uint8_t ISR_VECT sx128x::readRegister(uint16_t address)
{
  return singleTransfer(OP_READ_REGISTER_8X, address, 0x00);
}

void sx128x::writeRegister(uint16_t address, uint8_t value)
{
    singleTransfer(OP_WRITE_REGISTER_8X, address, value);
}

uint8_t ISR_VECT sx128x::singleTransfer(uint8_t opcode, uint16_t address, uint8_t value)
{
    waitOnBusy();

    uint8_t response;

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(opcode);
    _spiModem->transfer((address & 0xFF00) >> 8);
    _spiModem->transfer(address & 0x00FF);
    if (opcode == OP_READ_REGISTER_8X) {
        _spiModem->transfer(0x00);
    }
    response = _spiModem->transfer(value);
    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);

    return response;
}

void sx128x::rxAntEnable()
{
    if (_txen != -1) {
        digitalWrite(_txen, LOW);
    }
    if (_rxen != -1) {
        digitalWrite(_rxen, HIGH);
    }
}

void sx128x::txAntEnable()
{
    if (_txen != -1) {
        digitalWrite(_txen, HIGH);
    }
    if (_rxen != -1) {
        digitalWrite(_rxen, LOW);
    }
}

void sx128x::loraMode() {
    // enable lora mode on the SX1262 chip
    uint8_t mode = MODE_LONG_RANGE_MODE_8X;
    executeOpcode(OP_PACKET_TYPE_8X, &mode, 1);
}

void sx128x::waitOnBusy() {
    unsigned long time = millis();
    while (digitalRead(_busy) == HIGH)
    {
        if (millis() >= (time + 100)) {
            break;
        }
        // do nothing
    }
}

void sx128x::executeOpcode(uint8_t opcode, uint8_t *buffer, uint8_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(opcode);

    for (int i = 0; i < size; i++)
    {
        _spiModem->transfer(buffer[i]);
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx128x::executeOpcodeRead(uint8_t opcode, uint8_t *buffer, uint8_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(opcode);
    _spiModem->transfer(0x00);

    for (int i = 0; i < size; i++)
    {
        buffer[i] = _spiModem->transfer(0x00);
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx128x::writeBuffer(const uint8_t* buffer, size_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(OP_FIFO_WRITE_8X);
    _spiModem->transfer(_fifo_tx_addr_ptr);

    for (int i = 0; i < size; i++)
    {
        _spiModem->transfer(buffer[i]);
        _fifo_tx_addr_ptr++;
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx128x::readBuffer(uint8_t* buffer, size_t size)
{
    waitOnBusy();

    digitalWrite(_ss, LOW);

    _spiModem->beginTransaction(_spiSettings);
    _spiModem->transfer(OP_FIFO_READ_8X);
    _spiModem->transfer(_fifo_rx_addr_ptr);
    _spiModem->transfer(0x00);

    for (int i = 0; i < size; i++)
    {
        buffer[i] = _spiModem->transfer(0x00);
    }

    _spiModem->endTransaction();

    digitalWrite(_ss, HIGH);
}

void sx128x::setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr) {
  // because there is no access to these registers on the sx1280, we have
  // to set all these parameters at once or not at all.
  uint8_t buf[3];

  buf[0] = sf << 4;
  buf[1] = bw;
  buf[2] = cr; 
  executeOpcode(OP_MODULATION_PARAMS_8X, buf, 3);

  if (sf <= 6) {
      writeRegister(0x925, 0x1E);
  } else if (sf <= 8) {
      writeRegister(0x925, 0x37);
  } else if (sf >= 9) {
      writeRegister(0x925, 0x32);
  }
  writeRegister(0x093C, 0x1);
}

void sx128x::setPacketParams(uint32_t preamble, uint8_t headermode, uint8_t length, uint8_t crc) {
  // because there is no access to these registers on the sx1280, we have
  // to set all these parameters at once or not at all.
  uint8_t buf[7];
  // calculate exponent and mantissa values for modem
  uint8_t e = 1;
  uint8_t m = 1;
  uint32_t preamblelen;

  while (e <= 15) {
      while (m <= 15) {
          preamblelen = m * (pow(2,e));
          if (preamblelen >= preamble) break;
          m++;
      }
      if (preamblelen >= preamble) break;
      m = 0;
      e++;
  }

  buf[0] = (e << 4) | m;
  buf[1] = headermode;
  buf[2] = length;
  buf[3] = crc;
  // standard IQ setting (no inversion)
  buf[4] = 0x40; 
  // unused params
  buf[5] = 0x00; 
  buf[6] = 0x00; 

  executeOpcode(OP_PACKET_PARAMS_8X, buf, 7);
}

int sx128x::begin()
{
  if (_reset != -1) {
    pinMode(_reset, OUTPUT);

    // perform reset
    digitalWrite(_reset, LOW);
    delay(10);
    digitalWrite(_reset, HIGH);
    delay(10);
  }

  if (_rxen != -1) {
      pinMode(_rxen, OUTPUT);
  }

  if (_txen != -1) {
      pinMode(_txen, OUTPUT);
  }

  if (_busy != -1) {
      pinMode(_busy, INPUT);
  }

  if (!_preinit_done) {
    if (!preInit()) {
      return false;
    }
  }

  standby();
  loraMode();
  rxAntEnable();

  setFrequency(_frequency);

  // set LNA boost
  // todo: implement this
  //writeRegister(REG_LNA, 0x96);

  setModulationParams(_sf, _bw, _cr);
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  setTxPower(_txp);

  // set base addresses
  uint8_t basebuf[2] = {0};
  executeOpcode(OP_BUFFER_BASE_ADDR_8X, basebuf, 2);

  _radio_online = true;
  return 1;
}

void sx128x::end()
{
  // put in sleep mode
  sleep();

  // stop SPI
  _spiModem->end();

  _bitrate = 0;

  _radio_online = false;
  _preinit_done = false;
}

int sx128x::beginPacket(int implicitHeader)
{
  // put in standby mode
  standby();

  if (implicitHeader) {
    implicitHeaderMode();
  } else {
    explicitHeaderMode();
  }

  _payloadLength = 0;
  _fifo_tx_addr_ptr = 0;
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  return 1;
}

int sx128x::endPacket()
{
  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);

  txAntEnable();

  // put in single TX mode
  uint8_t timeout[3] = {0};
  executeOpcode(OP_TX_8X, timeout, 3);

  uint8_t buf[2];

  buf[0] = 0x00;
  buf[1] = 0x00;

  executeOpcodeRead(OP_GET_IRQ_STATUS_8X, buf, 2);

  // wait for TX done
  while ((buf[1] & IRQ_TX_DONE_MASK_8X) == 0) {
    buf[0] = 0x00;
    buf[1] = 0x00;
    executeOpcodeRead(OP_GET_IRQ_STATUS_8X, buf, 2);
    yield();
  }


  // clear IRQ's

  uint8_t mask[2];
  mask[0] = 0x00;
  mask[1] = IRQ_TX_DONE_MASK_8X;
  executeOpcode(OP_CLEAR_IRQ_STATUS_8X, mask, 2);
  return 1;
}

uint8_t sx128x::modemStatus() {
    // imitate the register status from the sx1276 / 78
    uint8_t buf[2] = {0};

    executeOpcodeRead(OP_GET_IRQ_STATUS_8X, buf, 2);

    uint8_t clearbuf[2] = {0};

    uint8_t byte = 0x00;

    if ((buf[0] & IRQ_PREAMBLE_DET_MASK_8X) != 0) {
        byte = byte | 0x01 | 0x04;
        // clear register after reading
        clearbuf[0] = IRQ_PREAMBLE_DET_MASK_8X;
    }

    if ((buf[1] & IRQ_HEADER_DET_MASK_8X) != 0) {
        byte = byte | 0x02 | 0x04;
    }

    executeOpcode(OP_CLEAR_IRQ_STATUS_8X, clearbuf, 2);

    return byte;
}


uint8_t sx128x::currentRssiRaw() {
    uint8_t byte = 0;
    executeOpcodeRead(OP_CURRENT_RSSI_8X, &byte, 1);
    return byte;
}

int ISR_VECT sx128x::currentRssi() {
    uint8_t byte = 0;
    executeOpcodeRead(OP_CURRENT_RSSI_8X, &byte, 1);
    int rssi = -byte / 2;
    return rssi;
}

uint8_t sx128x::packetRssiRaw() {
    uint8_t buf[5] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_8X, buf, 5);
    return buf[0];
}

int ISR_VECT sx128x::packetRssi() {
    // may need more calculations here
    uint8_t buf[5] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_8X, buf, 5);
    int pkt_rssi = -buf[0] / 2;
    return pkt_rssi;
}

uint8_t ISR_VECT sx128x::packetSnrRaw() {
    uint8_t buf[5] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_8X, buf, 5);
    return buf[1];
}

float ISR_VECT sx128x::packetSnr() {
    uint8_t buf[5] = {0};
    executeOpcodeRead(OP_PACKET_STATUS_8X, buf, 3);
    return float(buf[1]) * 0.25;
}

long sx128x::packetFrequencyError()
{
  int32_t freqError = 0;
  // todo: implement this, page 120 of sx1280 datasheet
  const float fError = 0.0;
  return static_cast<long>(fError);
}

size_t sx128x::write(uint8_t byte)
{
  return write(&byte, sizeof(byte));
}

size_t sx128x::write(const uint8_t *buffer, size_t size)
{
  if ((_payloadLength + size) > MAX_PKT_LENGTH) {
      size = MAX_PKT_LENGTH - _payloadLength;
  }

  // write data
  writeBuffer(buffer, size);
  _payloadLength = _payloadLength + size;
  return size;
}

int ISR_VECT sx128x::available()
{
    return _rxPacketLength - _packetIndex;
}

int ISR_VECT sx128x::read()
{
  if (!available()) {
    return -1;
  }

  uint8_t byte = _packet[_packetIndex];
  _packetIndex++;
  return byte;
}

int sx128x::peek()
{
  if (!available()) {
    return -1;
  }

  uint8_t b = _packet[_packetIndex];
  return b;
}

void sx128x::flush()
{
}

void sx128x::onReceive(void(*callback)(uint8_t, int))
{
  _onReceive = callback;

  if (callback) {
    pinMode(_dio0, INPUT);

      // set preamble and header detection irqs, plus dio0 mask
      uint8_t buf[8];

      // set irq masks, enable all
      buf[0] = 0xFF; 
      buf[1] = 0xFF;

      // set dio0 masks
      buf[2] = 0x00;
      buf[3] = IRQ_RX_DONE_MASK_8X; 

      // set dio1 masks
      buf[4] = 0x00; 
      buf[5] = 0x00;

      // set dio2 masks
      buf[6] = 0x00; 
      buf[7] = 0x00;

      executeOpcode(OP_SET_IRQ_FLAGS_8X, buf, 8);
//#ifdef SPI_HAS_NOTUSINGINTERRUPT
//    _spiModem->usingInterrupt(digitalPinToInterrupt(_dio0));
//#endif

    // make function available
    extern void onDio0Rise();

    attachInterrupt(digitalPinToInterrupt(_dio0), onDio0Rise, RISING);
  } else {
    detachInterrupt(digitalPinToInterrupt(_dio0));
//#ifdef SPI_HAS_NOTUSINGINTERRUPT
//    _spiModem->notUsingInterrupt(digitalPinToInterrupt(_dio0));
//#endif
  }
}

void sx128x::receive(int size)
{
  if (size > 0) {
    implicitHeaderMode();

    // tell radio payload length
    //_rxPacketLength = size;
    //_payloadLength = size;
    //setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
  } else {
    explicitHeaderMode();
  }

  rxAntEnable();

    uint8_t mode[3] = {0xFF, 0xFF, 0xFF}; // continuous mode
    executeOpcode(OP_RX_8X, mode, 3);
}

void sx128x::standby()
{
    uint8_t byte;
    if (_tcxo) {
          // STDBY_XOSC
          byte = 0x01;
    } else {
          // STDBY_RC
          byte = 0x00;
    }
      executeOpcode(OP_STANDBY_8X, &byte, 1); 
}

void sx128x::sleep()
{
    uint8_t byte = 0x00;
    executeOpcode(OP_SLEEP_8X, &byte, 1);
}

void sx128x::enableTCXO() {
    // todo: need to check how to implement on sx1280
}

void sx128x::disableTCXO() {
    // todo: need to check how to implement on sx1280
}

void sx128x::setTxPower(int level, int outputPin) {
    uint8_t tx_buf[2];
    #if BOARD_VARIANT == MODEL_13
    // RAK4631 with WisBlock SX1280 module (LIBSYS002)
    if (level > 27) {
        level = 27;
    } else if (level < 0) {
        level = 0;
    }

    _txp = level;

    int reg_value;

    switch (level) {
        case 0:
            reg_value = -18;
            break;
        case 1:
            reg_value = -17;
            break;
        case 2:
            reg_value = -16;
            break;
        case 3:
            reg_value = -15;
            break;
        case 4:
            reg_value = -14;
            break;
        case 5:
            reg_value = -13;
            break;
        case 6:
            reg_value = -12;
            break;
        case 7:
            reg_value = -10;
            break;
        case 8:
            reg_value = -9;
            break;
        case 9:
            reg_value = -8;
            break;
        case 10:
            reg_value = -7;
            break;
        case 11:
            reg_value = -6;
            break;
        case 12:
            reg_value = -5;
            break;
        case 13:
            reg_value = -4;
            break;
        case 14:
            reg_value = -3;
            break;
        case 15:
            reg_value = -2;
            break;
        case 16:
            reg_value = -1;
            break;
        case 17:
            reg_value = 0;
            break;
        case 18:
            reg_value = 1;
            break;
        case 19:
            reg_value = 2;
            break;
        case 20:
            reg_value = 3;
            break;
        case 21:
            reg_value = 4;
            break;
        case 22:
            reg_value = 5;
            break;
        case 23:
            reg_value = 6;
            break;
        case 24:
            reg_value = 8;
            break;
        case 25:
            reg_value = 9;
            break;
        case 26:
            reg_value = 12;
            break;
        case 27:
            reg_value = 13;
            break;
        default:
            reg_value = 0;
            break;
    }

    tx_buf[0] = reg_value + 18;
    tx_buf[1] = 0xE0; // ramping time - 20 microseconds

    executeOpcode(OP_TX_PARAMS_8X, tx_buf, 2);

    #elif BOARD_VARIANT == MODEL_A5
    // T3S3 SX1280 PA
        if (level > 20) {
            level = 20;
        } else if (level < 0) {
            level = 0;
        }

        _txp = level;

        int reg_value;

        switch (level) {
            /*case 0:
                reg_value = -18;
                break;
            case 1:
                reg_value = -17;
                break;
            case 2:
                reg_value = -16;
                break;
            case 3:
                reg_value = -15;
                break;
            case 4:
                reg_value = -14;
                break;
            case 5:
                reg_value = -13;
                break;
            case 6:
                reg_value = -12;
                break;
            case 7:
                reg_value = -10;
                break;
            case 8:
                reg_value = -9;
                break;
            case 9:
                reg_value = -8;
                break;
            case 10:
                reg_value = -7;
                break;
            case 11:
                reg_value = -6;
                break;
            case 12:
                reg_value = -5;
                break;
            case 13:
                reg_value = -4;
                break;
            case 14:
                reg_value = -3;
                break;
            case 15:
                reg_value = -2;
                break;
            case 16:
                reg_value = -1;
                break;
            case 17:
                reg_value = 0;
                break;
            case 18:
                reg_value = 1;
                break;
            case 19:
                reg_value = 2;
                break;*/
            case 20:
                reg_value = 3;
                break;
            default:
                reg_value = 0;
                break;
        }

        tx_buf[0] = reg_value + 18;
        tx_buf[1] = 0xE0; // ramping time - 20 microseconds
    #else
    // For SX1280 boards with no specific PA requirements
        if (level > 13) {
            level = 13;
        } else if (level < -18) {
            level = -18;
        }

        _txp = level;

        tx_buf[0] = level + 18;
        tx_buf[1] = 0xE0; // ramping time - 20 microseconds
    #endif
    executeOpcode(OP_TX_PARAMS_8X, tx_buf, 2);
}

uint8_t sx128x::getTxPower() {
      return _txp;
}

void sx128x::setFrequency(uint32_t frequency) {
  _frequency = frequency;

  uint8_t buf[3];

  uint32_t freq = (uint32_t)((double)frequency / (double)FREQ_STEP_8X);

  buf[0] = ((freq >> 16) & 0xFF);
  buf[1] = ((freq >> 8) & 0xFF);
  buf[2] = (freq & 0xFF);

  executeOpcode(OP_RF_FREQ_8X, buf, 3);
}

uint32_t sx128x::getFrequency() {
  // we can't read the frequency on the sx1280
  uint32_t frequency = _frequency;

  return frequency;
}

void sx128x::setSpreadingFactor(int sf)
{
  if (sf < 5) {
      sf = 5;
  } else if (sf > 12) {
    sf = 12;
  }

  _sf = sf;

  setModulationParams(sf, _bw, _cr);
  handleLowDataRate();
}

uint8_t sx128x::getSpreadingFactor()
{
    return _sf;
}

uint32_t sx128x::getSignalBandwidth()
{
  int bw = _bw;
  switch (bw) {
      case 0x34: return 203.125E3;
      case 0x26: return 406.25E3;
      case 0x18: return 812.5E3;
      case 0x0A: return 1625E3;
  }
  
  return 0;
}

void sx128x::handleLowDataRate(){
    // todo: do i need this??
}

void sx128x::optimizeModemSensitivity(){
    // todo: check if there's anything the sx1280 can do here
}

void sx128x::setSignalBandwidth(uint32_t sbw)
{
      if (sbw <= 203.125E3) {
          _bw = 0x34;
      } else if (sbw <= 406.25E3) {
          _bw = 0x26;
      } else if (sbw <= 812.5E3) {
          _bw = 0x18;
      } else {
          _bw = 0x0A;
      }

      setModulationParams(_sf, _bw, _cr);

  handleLowDataRate();
  optimizeModemSensitivity();
}

void sx128x::setCodingRate4(int denominator)
{
  if (denominator < 5) {
    denominator = 5;
  } else if (denominator > 8) {
    denominator = 8;
  }

  _cr = denominator - 4;

  // todo: add support for new interleaving scheme, see page 117 of sx1280
  // datasheet

  // update cr values for sx1280's use

  setModulationParams(_sf, _bw, _cr);
}

uint8_t sx128x::getCodingRate4()
{
    return _cr + 4;
}

void sx128x::setPreambleLength(long length)
{
  _preambleLength = length;
  setPacketParams(length, _implicitHeaderMode, _payloadLength, _crcMode);
}

void sx128x::setSyncWord(int sw)
{
    // not implemented
}

void sx128x::enableCrc()
{
      _crcMode = 0x20;
      setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void sx128x::disableCrc()
{
    _crcMode = 0;
    setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

byte sx128x::random()
{
    // todo: implement
}

void sx128x::setSPIFrequency(uint32_t frequency)
{
  _spiSettings = SPISettings(frequency, MSBFIRST, SPI_MODE0);
}

void sx128x::dumpRegisters(Stream& out)
{
  for (int i = 0; i < 128; i++) {
    out.print("0x");
    out.print(i, HEX);
    out.print(": 0x");
    out.println(readRegister(i), HEX);
  }
}

void sx128x::explicitHeaderMode()
{
  _implicitHeaderMode = 0;

  setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}

void sx128x::implicitHeaderMode()
{
    _implicitHeaderMode = 0x80;
    setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
}


void sx128x::handleDio0Rise()
{
    uint8_t buf[2];

    buf[0] = 0x00;
    buf[1] = 0x00;

    executeOpcodeRead(OP_GET_IRQ_STATUS_8X, buf, 2);

    executeOpcode(OP_CLEAR_IRQ_STATUS_8X, buf, 2);

    if ((buf[1] & IRQ_PAYLOAD_CRC_ERROR_MASK_8X) == 0) {
        // received a packet
        _packetIndex = 0;

        uint8_t rxbuf[2] = {0};
        executeOpcodeRead(OP_RX_BUFFER_STATUS_8X, rxbuf, 2);

        // If implicit header mode is enabled, read packet length as payload length instead.
        // See SX1280 datasheet v3.2, page 92
        if (_implicitHeaderMode == 0x80) {
            _rxPacketLength = _payloadLength;
        } else {
            _rxPacketLength = rxbuf[0];
        }

        _fifo_rx_addr_ptr = rxbuf[1];

        readBuffer(_packet, _rxPacketLength);

        if (_onReceive) {
            _onReceive(_index, _rxPacketLength);
        }

    }
}

void sx128x::updateBitrate() {
    if (_radio_online) {
        _lora_symbol_rate = (float)getSignalBandwidth()/(float)(pow(2, _sf));
        _lora_symbol_time_ms = (1.0/_lora_symbol_rate)*1000.0;
        _bitrate = (uint32_t)(_sf * ( (4.0/(float)(_cr+4)) / ((float)(pow(2, _sf))/((float)getSignalBandwidth()/1000.0)) ) * 1000.0);
        _lora_us_per_byte = 1000000.0/((float)_bitrate/8.0);
        _csma_slot_ms = 10;

        float target_preamble_symbols;
        if (_bitrate <= LORA_FAST_BITRATE_THRESHOLD) {
            target_preamble_symbols = (LORA_PREAMBLE_TARGET_MS/_lora_symbol_time_ms)-LORA_PREAMBLE_SYMBOLS_HW;
        } else {
            target_preamble_symbols = (LORA_PREAMBLE_FAST_TARGET_MS/_lora_symbol_time_ms)-LORA_PREAMBLE_SYMBOLS_HW;
        }
        if (target_preamble_symbols < LORA_PREAMBLE_SYMBOLS_MIN) {
            target_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN;
        } else {
            target_preamble_symbols = ceil(target_preamble_symbols);
        }
        _preambleLength = (long)target_preamble_symbols;
        setPacketParams(_preambleLength, _implicitHeaderMode, _payloadLength, _crcMode);
    } else {
        _bitrate = 0;
    }
}
