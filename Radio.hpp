// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license.

// Modifications and additions copyright 2023 by Mark Qvist & Jacob Eva
// Obviously still under the MIT license.

#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
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
#endif
