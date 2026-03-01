#ifndef ARDUINO_H
#define ARDUINO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define LOW 0
#define HIGH 1
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#define LSBFIRST 0
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

typedef uint8_t byte;
typedef bool boolean;

#ifdef __cplusplus
extern "C" {
#endif

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int val);
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);
void attachInterrupt(uint8_t interruptNum, void (*userFunc)(void), int mode);
void detachInterrupt(uint8_t interruptNum);
long random(long max);
long random(long min, long max);
void randomSeed(unsigned long seed);

#ifdef __cplusplus
}
#endif

class Serial_ {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) {}
    void println(const char* s) {}
    void write(uint8_t c) {}
    int available() { return 0; }
    int read() { return -1; }
};

extern Serial_ Serial;

#endif
