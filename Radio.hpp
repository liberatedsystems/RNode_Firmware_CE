// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license.

// Modifications and additions copyright 2024 by Mark Qvist & Jacob Eva
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

// Default LoRa settings
#define PHY_HEADER_LORA_SYMBOLS    20
#define PHY_CRC_LORA_BITS          16
#define LORA_PREAMBLE_SYMBOLS_MIN  18
#define LORA_PREAMBLE_TARGET_MS    24
#define LORA_PREAMBLE_FAST_DELTA   18
#define LORA_FAST_THRESHOLD_BPS    30E3
#define LORA_LIMIT_THRESHOLD_BPS   60E3

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


// CSMA Parameters
#define CSMA_SIFS_MS               0
#define CSMA_POST_TX_YIELD_SLOTS   3
#define CSMA_SLOT_MAX_MS           100
#define CSMA_SLOT_MIN_MS           24
#define CSMA_SLOT_MIN_FAST_DELTA   18
#define CSMA_SLOT_SYMBOLS          12
#define CSMA_CW_BANDS              4
#define CSMA_CW_MIN                0
#define CSMA_CW_PER_BAND_WINDOWS   15
#define CSMA_BAND_1_MAX_AIRTIME    7
#define CSMA_BAND_N_MIN_AIRTIME    85
#define CSMA_INFR_THRESHOLD_DB     12

#define LED_ID_TRIG 16

#define NOISE_FLOOR_SAMPLES 64

#define RSSI_OFFSET 157

#define PHY_HEADER_LORA_SYMBOLS 8

#define MODEM_TIMEOUT_MULT 1.5

// Status flags
const uint8_t SIG_DETECT = 0x01;
const uint8_t SIG_SYNCED = 0x02;
const uint8_t RX_ONGOING = 0x04;

// forward declare Utilities.h LED functions
void led_rx_on();
void led_rx_off();
void led_id_on();
void led_id_off();
void led_indicate_airtime_lock();

void kiss_indicate_channel_stats(uint8_t index);
void kiss_indicate_csma_stats(uint8_t index);

#if PLATFORM == PLATFORM_ESP32
// get update_lock for ESP32
extern portMUX_TYPE update_lock;
#endif

class RadioInterface : public Stream {
public:
    // todo: in the future define _spiModem and _spiSettings from here for inheritence by child classes
    RadioInterface(uint8_t index) : _index(index), _sf(0x07), _radio_locked(false),
    _radio_online(false), _st_airtime_limit(0.0), _lt_airtime_limit(0.0),
    _airtime_lock(false), _airtime(0.0), _longterm_airtime(0.0), _last_packet_cost(0.0),
    _local_channel_util(0.0), _total_channel_util(0.0),
    _longterm_channel_util(0.0), _last_status_update(0),
     _stat_signal_detected(false), _stat_signal_synced(false),_stat_rx_ongoing(false), _last_dcd(0), 
     _dcd_count(0), _dcd(false), _dcd_led(false),
    _dcd_waiting(false), _dcd_sample(0),
    _csma_slot_ms(CSMA_SLOT_MIN_MS),
    _preambleLength(LORA_PREAMBLE_SYMBOLS_MIN), _lora_symbol_time_ms(0.0),
    _lora_preamble_time_ms(0), _lora_header_time_ms(0), _lora_symbol_rate(0.0), _lora_us_per_byte(0.0), _bitrate(0),
     _packet{0}, _onReceive(NULL), _txp(0), _ldro(false), _limit_rate(false), _interference_detected(false), _avoid_interference(true), _difs_ms(CSMA_SIFS_MS + 2 * _csma_slot_ms), _difs_wait_start(0), _cw_wait_start(0), _cw_wait_target(0), _cw_wait_passed(0), _csma_cw(-1), _cw_band(1), _cw_min(0), _cw_max(CSMA_CW_PER_BAND_WINDOWS), _noise_floor_sampled(false), _noise_floor_sample(0), _noise_floor_buffer({0}), _noise_floor(-292), _led_id_filter(0), _preamble_detected_at(0) {};

    virtual void reset() = 0;

    virtual int begin() = 0;
    virtual void end() = 0;

    virtual int beginPacket(int implicitHeader = false) = 0;
    virtual int endPacket() = 0;

    virtual int packetRssi(uint8_t pkt_snr_raw = 0xFF) = 0;
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
    virtual int8_t getTxPower() = 0;
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
    virtual bool dcd() = 0;
    virtual void enableCrc() = 0;
    virtual void disableCrc() = 0;
    virtual void enableTCXO() = 0;
    virtual void disableTCXO() = 0;

    virtual uint8_t random() = 0;

    virtual void setSPIFrequency(uint32_t frequency) = 0;

    void updateBitrate() {
		if (!_radio_online) { _bitrate = 0; }
		else {
			_lora_symbol_rate = (float)getSignalBandwidth()/(float)(pow(2, _sf));
			_lora_symbol_time_ms = (1.0/_lora_symbol_rate)*1000.0;
			_bitrate = (uint32_t)(_sf * ( (4.0/(float)getCodingRate4()) / ((float)(pow(2, _sf))/((float)getSignalBandwidth()/1000.0)) ) * 1000.0);
			_lora_us_per_byte = 1000000.0/((float)_bitrate/8.0);
			
			bool fast_rate   = _bitrate > LORA_FAST_THRESHOLD_BPS;
			_limit_rate  = _bitrate > LORA_LIMIT_THRESHOLD_BPS;

			int csma_slot_min_ms = CSMA_SLOT_MIN_MS;
			float lora_preamble_target_ms = LORA_PREAMBLE_TARGET_MS;
			if (fast_rate) { csma_slot_min_ms        -= CSMA_SLOT_MIN_FAST_DELTA;
											 lora_preamble_target_ms -= LORA_PREAMBLE_FAST_DELTA; }
			
			_csma_slot_ms = _lora_symbol_time_ms*CSMA_SLOT_SYMBOLS;
			if (_csma_slot_ms > CSMA_SLOT_MAX_MS) { _csma_slot_ms = CSMA_SLOT_MAX_MS; }
			if (_csma_slot_ms < CSMA_SLOT_MIN_MS) { _csma_slot_ms = csma_slot_min_ms; }
			_difs_ms = CSMA_SIFS_MS + 2*_csma_slot_ms;
			
			float target_preamble_symbols = lora_preamble_target_ms/_lora_symbol_time_ms;
			if (target_preamble_symbols < LORA_PREAMBLE_SYMBOLS_MIN) { target_preamble_symbols = LORA_PREAMBLE_SYMBOLS_MIN; }
			else { target_preamble_symbols = (ceil)(target_preamble_symbols); }
			
            setPreambleLength(target_preamble_symbols);
			_lora_preamble_time_ms = (ceil)(_preambleLength * _lora_symbol_time_ms);
			_lora_header_time_ms   = (ceil)(PHY_HEADER_LORA_SYMBOLS * _lora_symbol_time_ms);

		}
    }
    virtual void handleDio0Rise() = 0;
    virtual bool getPacketValidity() = 0;
    uint32_t getBitrate() { return _bitrate; };
    uint8_t getIndex() { return _index; };
    void setRadioLock(bool lock) { _radio_locked = lock; };
    bool getRadioLock() { return _radio_locked; };
    void setRadioOnline(bool online) { _radio_online = online; };
    
    bool getRadioOnline() {return _radio_online; };
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
    
        updateCSMAParameters();
        kiss_indicate_channel_stats(_index);
    };
    float getAirtime(uint16_t written) { 
        float lora_symbols = 0;
        float packet_cost_ms = 0.0;

        if (interfaces[_index] == SX1276 || interfaces[_index] == SX1278) { 
            lora_symbols += (8*written + PHY_CRC_LORA_BITS - 4*_sf + 8 + PHY_HEADER_LORA_SYMBOLS);
            lora_symbols /=                          4*(_sf-2*_ldro);
            lora_symbols *= getCodingRate4();
            lora_symbols += _preambleLength + 0.25 + 8;
            packet_cost_ms += lora_symbols * _lora_symbol_time_ms;
        }
        else if (interfaces[_index] == SX1262 || interfaces[_index] == SX1280) {
            if (_sf < 7) {
                lora_symbols += (8*written + PHY_CRC_LORA_BITS - 4*_sf + PHY_HEADER_LORA_SYMBOLS);
                lora_symbols /=                              4*_sf;
                lora_symbols *= getCodingRate4();
                lora_symbols += _preambleLength + 2.25 + 8;
                packet_cost_ms += lora_symbols * _lora_symbol_time_ms;

            } else {
                lora_symbols += (8*written + PHY_CRC_LORA_BITS - 4*_sf + 8 + PHY_HEADER_LORA_SYMBOLS);
                lora_symbols /=                         4*(_sf-2*_ldro);
                lora_symbols *= getCodingRate4();
                lora_symbols += _preambleLength + 0.25 + 8;
                packet_cost_ms += lora_symbols * _lora_symbol_time_ms;
            }
        }
        _last_packet_cost = packet_cost_ms;
        return packet_cost_ms;
    }
    void addAirtime() {
        uint16_t cb = current_airtime_bin();
        uint16_t nb = cb+1; if (nb == AIRTIME_BINS) { nb = 0; }
        _airtime_bins[cb] += _last_packet_cost;
        _airtime_bins[nb] = 0;
    };
    void updateModemStatus() {
      #if MCU_VARIANT == MCU_ESP32
        portENTER_CRITICAL(&update_lock);
      #elif MCU_VARIANT == MCU_NRF52
        portENTER_CRITICAL();
      #endif

      bool carrier_detected = dcd();
      int current_rssi = currentRssi();
      _last_status_update = millis();

      #if MCU_VARIANT == MCU_ESP32
        portEXIT_CRITICAL(&update_lock);
      #elif MCU_VARIANT == MCU_NRF52
        portEXIT_CRITICAL();
      #endif

      _interference_detected = !carrier_detected && (current_rssi > (_noise_floor+CSMA_INFR_THRESHOLD_DB));
      if (_interference_detected) { if (_led_id_filter < LED_ID_TRIG) { _led_id_filter += 1; } }
      else                       { if (_led_id_filter > 0) {_led_id_filter -= 1; } }

      if (carrier_detected) { _dcd = true; } else { _dcd = false; }

      _dcd_led = _dcd;
      if (_dcd_led) { led_rx_on(); }
      else {
        if (_interference_detected) {
          if (_led_id_filter >= LED_ID_TRIG && _noise_floor_sampled) { led_id_on(); }
        } else {
          if (_airtime_lock) { led_indicate_airtime_lock(); }
          else              { led_rx_off(); led_id_off(); }
        }
      }
    }
    void updateNoiseFloor() {
        int current_rssi = currentRssi();
        if (!_dcd) {
            if (!_noise_floor_sampled || current_rssi < _noise_floor + CSMA_INFR_THRESHOLD_DB) {
                _noise_floor_buffer[_noise_floor_sample] = current_rssi;
                _noise_floor_sample = _noise_floor_sample+1;
                if (_noise_floor_sample >= NOISE_FLOOR_SAMPLES) {
                    _noise_floor_sample %= NOISE_FLOOR_SAMPLES;
                    _noise_floor_sampled = true;
                }

                if (_noise_floor_sampled) {
                    _noise_floor = 0;
                    for (int ni = 0; ni < NOISE_FLOOR_SAMPLES; ni++) { _noise_floor += _noise_floor_buffer[ni]; }
                    _noise_floor /= NOISE_FLOOR_SAMPLES;
                }
            }
        }
    }
    void checkModemStatus() {
      if (millis()-_last_status_update >= STATUS_INTERVAL_MS) {
        updateModemStatus();
        updateNoiseFloor();

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
    void updateCSMAParameters() {
        int airtime_pct = (int)(_airtime*100);
        int new_cw_band = _cw_band;

        if (airtime_pct <= CSMA_BAND_1_MAX_AIRTIME) { new_cw_band = 1; }
        else {
            int at = airtime_pct + CSMA_BAND_1_MAX_AIRTIME;
            new_cw_band = map(at, CSMA_BAND_1_MAX_AIRTIME, CSMA_BAND_N_MIN_AIRTIME, 2, CSMA_CW_BANDS);
        }

        if (new_cw_band > CSMA_CW_BANDS) { new_cw_band = CSMA_CW_BANDS; }
        if (new_cw_band != _cw_band) { 
            _cw_band = (uint8_t)(new_cw_band);
            _cw_min  = (_cw_band-1) * CSMA_CW_PER_BAND_WINDOWS;
            _cw_max  = (_cw_band) * CSMA_CW_PER_BAND_WINDOWS - 1;
            kiss_indicate_csma_stats(_index);
        }
    }
    void setDCD(bool dcd) { _dcd = dcd; };
    bool getDCD() { return _dcd; };
    void setDCDWaiting(bool dcd_waiting) { _dcd_waiting = dcd_waiting; };
    bool getDCDWaiting() { return _dcd_waiting; };
    float getAirtime() { return _airtime; };
    float getLongtermAirtime() { return _longterm_airtime; };
    float getTotalChannelUtil() { return _total_channel_util; };
    float getLongtermChannelUtil() { return _longterm_channel_util; };
    void setCSMASlotMS(int slot_size) { _csma_slot_ms = slot_size; };
    int getCSMASlotMS() { return _csma_slot_ms; };
    float getSymbolTime() { return _lora_symbol_time_ms; };
    float getSymbolRate() { return _lora_symbol_rate; };
    long getPreambleLength() { return _preambleLength; };
    void setAvdInterference(bool cfg) { _avoid_interference = cfg; };
    bool getAvdInterference() { return _avoid_interference; };
    bool getInterference() { return _interference_detected; };
    int getNoiseFloor() { return _noise_floor; };
    unsigned long getDifsMS() { return _difs_ms; };
    uint8_t getCWBand() { return _cw_band; };
    uint8_t getCWMin() { return _cw_min; };
    uint8_t getCWMax() { return _cw_max; };
    uint8_t getCW() { return _csma_cw; };
    void setCW(uint8_t cw) { _csma_cw = cw; };
    void setCWWaitTarget(unsigned long target) { _cw_wait_target = target; };
    unsigned long getCWWaitTarget() { return _cw_wait_target; };
    unsigned long getDifsWaitStart() { return _difs_wait_start; };
    void setDifsWaitStart(unsigned long start) { _difs_wait_start = start; };
    unsigned long getCWWaitStart() { return _cw_wait_start; };
    void setCWWaitStart(unsigned long start) { _cw_wait_start = start; };
    void addCWWaitPassed(unsigned long start) { _cw_wait_passed += start; };
    void resetCWWaitPassed() { _cw_wait_passed = 0; };
    bool getCWWaitStatus() { return _cw_wait_passed < _cw_wait_target; };
    bool getLimitRate() { return _limit_rate; };
protected:
    virtual void explicitHeaderMode() = 0;
    virtual void implicitHeaderMode() = 0;

    uint8_t _index;
    uint32_t _bitrate;
    int8_t _txp;
    uint8_t _sf;
    bool _radio_locked;
    bool _radio_online;
    float _st_airtime_limit;
    float _lt_airtime_limit;
    bool _airtime_lock;
    uint16_t _airtime_bins[AIRTIME_BINS] = {0};
    uint16_t _longterm_bins[AIRTIME_BINS] = {0};
    float _airtime;
    float _longterm_airtime;
    float _last_packet_cost;
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
	bool _util_samples[DCD_SAMPLES] = {false};
	int _dcd_sample;
    long _preambleLength;
    float _lora_symbol_time_ms;
    float _lora_symbol_rate;
    float _lora_us_per_byte;
    long _lora_preamble_time_ms;
    long _lora_header_time_ms;
    bool _ldro;
    bool _limit_rate;
    bool _interference_detected;
    bool _avoid_interference;
    int _csma_slot_ms;
    unsigned long _difs_ms;
    unsigned long _difs_wait_start;
    unsigned long _cw_wait_start;
    unsigned long _cw_wait_target;
    unsigned long _cw_wait_passed;
    int _csma_cw;
    uint8_t _cw_band;
    uint8_t _cw_min;
    uint8_t _cw_max;
    bool _noise_floor_sampled;
    int _noise_floor_sample;
    int _noise_floor_buffer[NOISE_FLOOR_SAMPLES];
    int _noise_floor;
    uint8_t _led_id_filter;
    unsigned long _preamble_detected_at;

    uint8_t _packet[255];
    void (*_onReceive)(uint8_t, int);
};

class sx126x : public RadioInterface {
public:
  sx126x(uint8_t index, SPIClass* spi, bool tcxo, bool dio2_as_rf_switch, int ss, int sclk, int mosi, int miso, int reset, int
          dio0, int busy, int rxen);

  void reset();

  int begin();
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int packetRssi(uint8_t pkt_snr_raw = 0xFF);
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
  int8_t getTxPower();
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
  bool dcd();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();


  byte random();

  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

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

  void calibrate(void);
  void calibrate_image(uint32_t frequency);
  bool getPacketValidity();

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
  uint8_t _bw;
  uint8_t _cr;
  int _packetIndex;
  int _implicitHeaderMode;
  int _payloadLength;
  int _crcMode;
  int _fifo_tx_addr_ptr;
  int _fifo_rx_addr_ptr;
  bool _preinit_done;
  bool _tcxo;
  bool _dio2_as_rf_switch;
};

class sx127x : public RadioInterface {
public:
  sx127x(uint8_t index, SPIClass* spi, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy);

  void reset();

  int begin();
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int packetRssi(uint8_t pkt_snr_raw = 0xFF);
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
  int8_t getTxPower();
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
  bool dcd();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();

  byte random();

  void setSPIFrequency(uint32_t frequency);

  void handleDio0Rise();
  bool getPacketValidity();
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
  uint8_t _cr;
  uint32_t _bw;
};

class sx128x : public RadioInterface {
public:
  sx128x(uint8_t index, SPIClass* spi, bool tcxo, int ss, int sclk, int mosi, int miso, int reset, int dio0, int busy, int rxen, int txen);

  void reset();

  int begin();
  void end();

  int beginPacket(int implicitHeader = false);
  int endPacket();

  int packetRssi(uint8_t pkt_snr_raw = 0xFF);
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
  int8_t getTxPower();
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
  bool dcd();
  void enableCrc();
  void disableCrc();
  void enableTCXO();
  void disableTCXO();

  byte random();

  void setSPIFrequency(uint32_t frequency);

  void dumpRegisters(Stream& out);

  void handleDio0Rise();

  bool getPacketValidity();

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
  bool _tcxo;
  uint8_t _preamble_e;
  uint8_t _preamble_m;
  uint32_t _last_preamble;
};
#endif
