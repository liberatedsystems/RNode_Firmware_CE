// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license.

// Modifications and additions copyright 2023 by Mark Qvist & Jacob Eva
// Obviously still under the MIT license.

#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include "Interfaces.h"
#include "Boards.h"
#include "src/misc/FIFOBuffer.h"

#define MAX_PKT_LENGTH                255

// TX
#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

// DCD
#define STATUS_INTERVAL_MS 3
#define DCD_SAMPLES 2500
#define UTIL_UPDATE_INTERVAL_MS 1000
#define UTIL_UPDATE_INTERVAL (UTIL_UPDATE_INTERVAL_MS/STATUS_INTERVAL_MS)
#define AIRTIME_LONGTERM 3600
#define AIRTIME_LONGTERM_MS (AIRTIME_LONGTERM*1000)
#define AIRTIME_BINLEN_MS (STATUS_INTERVAL_MS*DCD_SAMPLES)
#define AIRTIME_BINS ((AIRTIME_LONGTERM*1000)/AIRTIME_BINLEN_MS)
#define current_airtime_bin(void) (millis()%AIRTIME_LONGTERM_MS)/AIRTIME_BINLEN_MS
#define DCD_THRESHOLD 2
#define DCD_LED_STEP_D 3
#define LORA_PREAMBLE_SYMBOLS_HW  4
#define LORA_PREAMBLE_SYMBOLS_MIN 18
#define LORA_PREAMBLE_TARGET_MS   15
#define LORA_PREAMBLE_FAST_TARGET_MS 1
#define LORA_FAST_BITRATE_THRESHOLD 40000

#define RSSI_OFFSET 157

#define PHY_HEADER_LORA_SYMBOLS 8

#define _e 2.71828183
#define _S 10.0

// Status flags
const uint8_t SIG_DETECT = 0x01;
const uint8_t SIG_SYNCED = 0x02;
const uint8_t RX_ONGOING = 0x04;

// forward declare Utilities.h LED functions
void led_rx_on();

void led_rx_off();
void led_indicate_airtime_lock();

#if PLATFORM == PLATFORM_ESP32
// get update_lock for ESP32
extern portMUX_TYPE update_lock;
#endif

class RadioInterface : public Stream {
public:
    // todo: in the future define _spiModem and _spiSettings from here for inheritence by child classes
    RadioInterface(uint8_t index) : _index(index), _radio_locked(false),
    _radio_online(false), _st_airtime_limit(0.0), _lt_airtime_limit(0.0),
    _airtime_lock(false), _airtime(0.0), _longterm_airtime(0.0),
    _local_channel_util(0.0), _total_channel_util(0.0),
    _longterm_channel_util(0.0), _last_status_update(0),
     _stat_signal_detected(false), _stat_signal_synced(false),_stat_rx_ongoing(false), _last_dcd(0), 
     _dcd_count(0), _dcd(false), _dcd_led(false),
    _dcd_waiting(false), _dcd_wait_until(0), _dcd_sample(0),
    _post_tx_yield_timeout(0), _csma_slot_ms(50), _csma_p_min(0.1),
    _csma_p_max(0.8), _preambleLength(6), _lora_symbol_time_ms(0.0),
    _lora_symbol_rate(0.0), _lora_us_per_byte(0.0), _bitrate(0),
     _packet{0}, _onReceive(NULL) {};
    virtual int begin() = 0;
    virtual void end() = 0;

    virtual int beginPacket(int implicitHeader = false) = 0;
    virtual int endPacket() = 0;

    virtual int packetRssi() = 0;
    virtual int currentRssi() = 0;
    virtual uint8_t packetRssiRaw() = 0;
    virtual uint8_t currentRssiRaw() = 0;
    virtual uint8_t packetSnrRaw() = 0;
    virtual float packetSnr() = 0;
    virtual long packetFrequencyError() = 0;

    // from Print
    virtual size_t write(uint8_t byte) = 0;
    virtual size_t write(const uint8_t *buffer, size_t size) = 0;

    // from Stream
    virtual int available() = 0;
    virtual int read() = 0;
    virtual int peek() = 0;
    virtual void flush() = 0;

    virtual void onReceive(void(*callback)(uint8_t, int)) = 0;

    virtual void receive(int size = 0) = 0;
    virtual void standby() = 0;
    virtual void sleep() = 0;

    virtual bool preInit() = 0;
    virtual uint8_t getTxPower() = 0;
    virtual void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN) = 0;
    virtual uint32_t getFrequency() = 0;
    virtual void setFrequency(uint32_t frequency) = 0;
    virtual void setSpreadingFactor(int sf) = 0;
    virtual uint8_t getSpreadingFactor() = 0;
    virtual uint32_t getSignalBandwidth() = 0;
    virtual void setSignalBandwidth(uint32_t sbw) = 0;
    virtual void setCodingRate4(int denominator) = 0;
    virtual uint8_t getCodingRate4() = 0;
    virtual void setPreambleLength(long length) = 0;
    virtual uint8_t modemStatus() = 0;
    virtual void enableCrc() = 0;
    virtual void disableCrc() = 0;
    virtual void enableTCXO() = 0;
    virtual void disableTCXO() = 0;

    virtual byte random() = 0;

    virtual void setSPIFrequency(uint32_t frequency) = 0;

    virtual void updateBitrate() = 0;
    virtual void handleDio0Rise() = 0;
    uint32_t getBitrate() { return _bitrate; };
    uint8_t getIndex() { return _index; };
    void setRadioLock(bool lock) { _radio_locked = lock; };
    bool getRadioLock() { return _radio_locked; };
    void setRadioOnline(bool online) { _radio_online = online; };
    bool getRadioOnline() { return _radio_online; };
    void setSTALock(float at) { _st_airtime_limit = at; };
    float getSTALock() { return _st_airtime_limit; };
    void setLTALock(float at) { _lt_airtime_limit = at; };
    float getLTALock() { return _lt_airtime_limit; };
    bool calculateALock() {
      _airtime_lock = false;
      if (_st_airtime_limit != 0.0 && _airtime >= _st_airtime_limit) {
          _airtime_lock = true;
      }
      if (_lt_airtime_limit != 0.0 && _longterm_airtime >= _lt_airtime_limit) {
          _airtime_lock = true;
      }
      return _airtime_lock;
    };
    void updateAirtime() {
        uint16_t cb = current_airtime_bin();
        uint16_t pb = cb-1; if (cb-1 < 0) { pb = AIRTIME_BINS-1; }
        uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
        _airtime_bins[nb] = 0;
        _airtime = (float)(_airtime_bins[cb]+_airtime_bins[pb])/(2.0*AIRTIME_BINLEN_MS);
    
        uint32_t longterm_airtime_sum = 0;
        for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
          longterm_airtime_sum += _airtime_bins[bin];
        }
        _longterm_airtime = (float)longterm_airtime_sum/(float)AIRTIME_LONGTERM_MS;
    
        float longterm_channel_util_sum = 0.0;
        for (uint16_t bin = 0; bin < AIRTIME_BINS; bin++) {
          longterm_channel_util_sum += _longterm_bins[bin];
        }
        _longterm_channel_util = (float)longterm_channel_util_sum/(float)AIRTIME_BINS;
    
          updateCSMAp();

          //kiss_indicate_channel_stats(); // todo: enable me!
    };
    void addAirtime(uint16_t written) {
        float packet_cost_ms = 0.0;
        float payload_cost_ms = ((float)written * _lora_us_per_byte)/1000.0;
        packet_cost_ms += payload_cost_ms;
        packet_cost_ms += (_preambleLength+4.25)*_lora_symbol_time_ms;
        packet_cost_ms += PHY_HEADER_LORA_SYMBOLS * _lora_symbol_time_ms;
        uint16_t cb = current_airtime_bin();
        uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
        _airtime_bins[cb] += packet_cost_ms;
        _airtime_bins[nb] = 0;
    };
    void checkModemStatus() {
      if (millis()-_last_status_update >= STATUS_INTERVAL_MS) {
        updateModemStatus();
    
          _util_samples[_dcd_sample] = _dcd;
          _dcd_sample = (_dcd_sample+1)%DCD_SAMPLES;
          if (_dcd_sample % UTIL_UPDATE_INTERVAL == 0) {
            int util_count = 0;
            for (int ui = 0; ui < DCD_SAMPLES; ui++) {
              if (_util_samples[ui]) util_count++;
            }
            _local_channel_util = (float)util_count / (float)DCD_SAMPLES;
            _total_channel_util = _local_channel_util + _airtime;
            if (_total_channel_util > 1.0) _total_channel_util = 1.0;
    
            int16_t cb = current_airtime_bin();
            uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
            if (_total_channel_util > _longterm_bins[cb]) _longterm_bins[cb] = _total_channel_util;
            _longterm_bins[nb] = 0.0;
    
            updateAirtime();
          }
      }
    };
      void updateModemStatus() {
          #if PLATFORM == PLATFORM_ESP32 
          portENTER_CRITICAL(&update_lock);
          #elif PLATFORM == PLATFORM_NRF52 
          portENTER_CRITICAL();
          #endif
    
          uint8_t status = modemStatus();
    
          _last_status_update = millis();
    
          #if PLATFORM == PLATFORM_ESP32 
          portEXIT_CRITICAL(&update_lock);
          #elif PLATFORM == PLATFORM_NRF52 
          portEXIT_CRITICAL();
          #endif
    
          if ((status & SIG_DETECT) == SIG_DETECT) { _stat_signal_detected = true; } else { _stat_signal_detected = false; }
          if ((status & SIG_SYNCED) == SIG_SYNCED) { _stat_signal_synced = true; } else { _stat_signal_synced = false; }
          if ((status & RX_ONGOING) == RX_ONGOING) { _stat_rx_ongoing = true; } else { _stat_rx_ongoing = false; }
    
          // if (stat_signal_detected || stat_signal_synced || stat_rx_ongoing) {
          if (_stat_signal_detected || _stat_signal_synced) {
              if (_stat_rx_ongoing) {
                  if (_dcd_count < DCD_THRESHOLD) {
                      _dcd_count++;
                  } else {
                      _last_dcd = _last_status_update;
                      _dcd_led = true;
                      _dcd = true;
                  }
              }
          } else {
              if (_dcd_count == 0) {
                  _dcd_led = false;
              } else if (_dcd_count > DCD_LED_STEP_D) {
                  _dcd_count -= DCD_LED_STEP_D;
              } else {
                  _dcd_count = 0;
              }
    
              if (_last_status_update > _last_dcd+_csma_slot_ms) {
                  _dcd = false;
                  _dcd_led = false;
                  _dcd_count = 0;
              }
          }

    
          if (_dcd_led) {
              led_rx_on();
          } else {
              if (_airtime_lock) {
                  led_indicate_airtime_lock();
              } else {
                  led_rx_off();
              }
          }
    };
    void setPostTxYieldTimeout(uint32_t timeout) { _post_tx_yield_timeout = timeout; };
    uint32_t getPostTxYieldTimeout() { return _post_tx_yield_timeout; };
    void setDCD(bool dcd) { _dcd = dcd; };
    bool getDCD() { return _dcd; };
    void setDCDWaiting(bool dcd_waiting) { _dcd_waiting = dcd_waiting; };
    bool getDCDWaiting() { return _dcd_waiting; };
    void setDCDWaitUntil(uint32_t dcd_wait_until) { _dcd_wait_until = dcd_wait_until; };
    bool getDCDWaitUntil() { return _dcd_wait_until; };
    float getAirtime() { return _airtime; };
    float getLongtermAirtime() { return _longterm_airtime; };
    float getTotalChannelUtil() { return _total_channel_util; };
    float getLongtermChannelUtil() { return _longterm_channel_util; };
    float CSMASlope(float u) { return (pow(_e,_S*u-_S/2.0))/(pow(_e,_S*u-_S/2.0)+1.0); };
    void updateCSMAp() {
      _csma_p = (uint8_t)((1.0-(_csma_p_min+(_csma_p_max-_csma_p_min)*CSMASlope(_airtime)))*255.0);
    };
    uint8_t getCSMAp() { return _csma_p; };
    void setCSMASlotMS(int slot_size) { _csma_slot_ms = slot_size; };
    int getCSMASlotMS() { return _csma_slot_ms; };
    float getSymbolTime() { return _lora_symbol_time_ms; };
    float getSymbolRate() { return _lora_symbol_rate; };
    long getPreambleLength() { return _preambleLength; };
protected:
    virtual void explicitHeaderMode() = 0;
    virtual void implicitHeaderMode() = 0;

    uint8_t _index;
    bool _radio_locked;
    bool _radio_online;
    float _st_airtime_limit;
    float _lt_airtime_limit;
    bool _airtime_lock;
    uint16_t _airtime_bins[AIRTIME_BINS] = {0};
    uint16_t _longterm_bins[AIRTIME_BINS] = {0};
    float _airtime;
    float _longterm_airtime;
    float _local_channel_util;
    float _total_channel_util;
    float _longterm_channel_util;
    uint32_t _last_status_update;
    bool _stat_signal_detected;
    bool _stat_signal_synced;
    bool _stat_rx_ongoing;
    uint32_t _last_dcd;
    uint16_t _dcd_count;
    bool _dcd;
    bool _dcd_led;
    bool _dcd_waiting;
    long _dcd_wait_until;
	bool _util_samples[DCD_SAMPLES] = {false};
	int _dcd_sample;
    uint32_t _post_tx_yield_timeout;
    uint8_t _csma_p;
    int _csma_slot_ms;
    float _csma_p_min;
    float _csma_p_max;
    long _preambleLength;
    float _lora_symbol_time_ms;
    float _lora_symbol_rate;
    float _lora_us_per_byte;
    uint32_t _bitrate;
  uint8_t _packet[255];
    void (*_onReceive)(uint8_t, int);
};

class sx126x : public RadioInterface {
public:
  sx126x(uint8_t index, SPIClass* spi, bool tcxo, bool dio2_as_rf_switch, int ss, int sclk, int mosi, int miso, int reset, int
          dio0, int busy, int rxen);

  int begin();
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int packetRssi();
  int currentRssi();
  uint8_t packetRssiRaw();
  uint8_t currentRssiRaw();
  uint8_t packetSnrRaw();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  int available();
  int read();
  int peek();
  void flush();

  void onReceive(void(*callback)(uint8_t, int));

  void receive(int size = 0);
  void standby();
  void sleep();

  bool preInit();
  uint8_t getTxPower();
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  uint32_t getFrequency();
  void setFrequency(uint32_t frequency);
  void setSpreadingFactor(int sf);
  uint8_t getSpreadingFactor();
  uint32_t getSignalBandwidth();
  void setSignalBandwidth(uint32_t sbw);
  void setCodingRate4(int denominator);
  uint8_t getCodingRate4();
  void setPreambleLength(long length);
  uint8_t modemStatus();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();


  byte random();

  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

  void updateBitrate();

  void handleDio0Rise();
private:
  void writeBuffer(const uint8_t* buffer, size_t size);
  void readBuffer(uint8_t* buffer, size_t size);
  void loraMode();
  void rxAntEnable();
  void setPacketParams(uint32_t preamble, uint8_t headermode, uint8_t length, uint8_t crc);
  void setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, int ldro);
  void setSyncWord(uint16_t sw);
  void waitOnBusy();
  void executeOpcode(uint8_t opcode, uint8_t *buffer, uint8_t size);
  void executeOpcodeRead(uint8_t opcode, uint8_t *buffer, uint8_t size);
  void explicitHeaderMode();
  void implicitHeaderMode();


  uint8_t readRegister(uint16_t address);
  void writeRegister(uint16_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t opcode, uint16_t address, uint8_t value);

  static void onDio0Rise();

  void handleLowDataRate();
  void optimizeModemSensitivity();

  void reset(void);
  void calibrate(void);
  void calibrate_image(uint32_t frequency);

private:
  SPISettings _spiSettings;
  SPIClass* _spiModem;
  int _ss;
  int _sclk;
  int _mosi;
  int _miso;
  int _reset;
  int _dio0;
  int _rxen;
  int _busy;
  uint32_t _frequency;
  int _txp;
  uint8_t _sf;
  uint8_t _bw;
  uint8_t _cr;
  uint8_t _ldro;
  int _packetIndex;
  int _implicitHeaderMode;
  int _payloadLength;
  int _crcMode;
  int _fifo_tx_addr_ptr;
  int _fifo_rx_addr_ptr;
  bool _preinit_done;
  uint8_t _index;
  bool _tcxo;
  bool _dio2_as_rf_switch;
};

class sx127x : public RadioInterface {
public:
  sx127x(uint8_t index, SPIClass* spi, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy);

  int begin();
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int packetRssi();
  int currentRssi();
  uint8_t packetRssiRaw();
  uint8_t currentRssiRaw();
  uint8_t packetSnrRaw();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  int available();
  int read();
  int peek();
  void flush();

  void onReceive(void(*callback)(uint8_t, int));

  void receive(int size = 0);
  void standby();
  void sleep();

  bool preInit();
  uint8_t getTxPower();
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  uint32_t getFrequency();
  void setFrequency(uint32_t frequency);
  void setSpreadingFactor(int sf);
  uint8_t getSpreadingFactor();
  uint32_t getSignalBandwidth();
  void setSignalBandwidth(uint32_t sbw);
  void setCodingRate4(int denominator);
  uint8_t getCodingRate4();
  void setPreambleLength(long length);
  uint8_t modemStatus();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();

  byte random();

  void setSPIFrequency(uint32_t frequency);

  void updateBitrate();

  void handleDio0Rise();
private:
  void setSyncWord(uint8_t sw);
  void explicitHeaderMode();
  void implicitHeaderMode();


  uint8_t readRegister(uint8_t address);
  void writeRegister(uint8_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t address, uint8_t value);

  static void onDio0Rise();

  void handleLowDataRate();
  void optimizeModemSensitivity();

private:
  SPISettings _spiSettings;
  SPIClass* _spiModem;
  int _ss;
  int _sclk;
  int _mosi;
  int _miso;
  int _reset;
  int _dio0;
  int _busy;
  uint32_t _frequency;
  int _packetIndex;
  int _implicitHeaderMode;
  bool _preinit_done;
  uint8_t _index;
  uint8_t _sf;
  uint8_t _cr;
};

class sx128x : public RadioInterface {
public:
  sx128x(uint8_t index, SPIClass* spi, bool tcxo, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy, int rxen, int txen);

  int begin();
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int packetRssi();
  int currentRssi();
  uint8_t packetRssiRaw();
  uint8_t currentRssiRaw();
  uint8_t packetSnrRaw();
  float packetSnr();
  long packetFrequencyError();

  // from Print
  size_t write(uint8_t byte);
  size_t write(const uint8_t *buffer, size_t size);

  // from Stream
  int available();
  int read();
  int peek();
  void flush();

  void onReceive(void(*callback)(uint8_t, int));

  void receive(int size = 0);
  void standby();
  void sleep();

  bool preInit();
  uint8_t getTxPower();
  void setTxPower(int level, int outputPin = PA_OUTPUT_PA_BOOST_PIN);
  uint32_t getFrequency();
  void setFrequency(uint32_t frequency);
  void setSpreadingFactor(int sf);
  uint8_t getSpreadingFactor();
  uint32_t getSignalBandwidth();
  void setSignalBandwidth(uint32_t sbw);
  void setCodingRate4(int denominator);
  uint8_t getCodingRate4();
  void setPreambleLength(long length);
  uint8_t modemStatus();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();

  byte random();

  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

  void updateBitrate();

  void handleDio0Rise();

private:
  void writeBuffer(const uint8_t* buffer, size_t size);
  void readBuffer(uint8_t* buffer, size_t size);
  void txAntEnable();
  void rxAntEnable();
  void loraMode();
  void waitOnBusy();
  void executeOpcode(uint8_t opcode, uint8_t *buffer, uint8_t size);
  void executeOpcodeRead(uint8_t opcode, uint8_t *buffer, uint8_t size);
  void setPacketParams(uint32_t preamble, uint8_t headermode, uint8_t length, uint8_t crc);
  void setModulationParams(uint8_t sf, uint8_t bw, uint8_t cr);
  void setSyncWord(int sw);
  void explicitHeaderMode();
  void implicitHeaderMode();


  uint8_t readRegister(uint16_t address);
  void writeRegister(uint16_t address, uint8_t value);
  uint8_t singleTransfer(uint8_t opcode, uint16_t address, uint8_t value);

  static void onDio0Rise();

  void handleLowDataRate();
  void optimizeModemSensitivity();

private:
  SPISettings _spiSettings;
  SPIClass* _spiModem;
  int _ss;
  int _sclk;
  int _mosi;
  int _miso;
  int _reset;
  int _dio0;
  int _rxen;
  int _txen;
  int _busy;
  int _modem;
  uint32_t _frequency;
  int _txp;
  uint8_t _sf;
  uint8_t _bw;
  uint8_t _cr;
  int _packetIndex;
  int _implicitHeaderMode;
  int _payloadLength;
  int _crcMode;
  int _fifo_tx_addr_ptr;
  int _fifo_rx_addr_ptr;
  bool _preinit_done;
  int _rxPacketLength;
  uint8_t _index;
  bool _tcxo;
};
#endif
