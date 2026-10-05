// Host-side Arduino shim for the OBJECT screen emulator.
// Only what xiao_oled.ino and Adafruit GFX use; the harness (emulator.cpp)
// implements the functions declared here.
#pragma once
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Print.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef ARDUINO
#define ARDUINO 10819
#endif

typedef bool boolean;
typedef uint8_t byte;

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define FALLING 2
#define RISING 3
#define CHANGE 4

// XIAO nRF52840: An and Dn name the same pad.
enum { D0 = 0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10 };
enum { A0 = 0, A1, A2, A3, A4, A5 };
#define LED_BUILTIN 11

#define radians(d) ((d) * M_PI / 180.0)
#define degrees(r) ((r) * 180.0 / M_PI)
#define PROGMEM
#define F(x) (x)

// ---- virtual hardware, implemented by the harness ----
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
void yield();
void pinMode(int pin, int mode);
void digitalWrite(int pin, int v);
int digitalRead(int pin);
int analogRead(int pin);
inline void analogReadResolution(int) {}
void analogWrite(int pin, int v);
void attachInterrupt(int irq, void (*fn)(), int mode);
inline int digitalPinToInterrupt(int p) { return p; }
inline void noInterrupts() {}
inline void interrupts() {}
inline void enterOTADfu() {}

class HardwareSerial : public Print {
 public:
  explicit HardwareSerial(bool rxFromGps) : gps_(rxFromGps) {}
  void begin(unsigned long) { open_ = true; }
  void end() { open_ = false; }
  operator bool() const { return true; }
  int available();
  int read();
  using Print::write;
  size_t write(uint8_t c) override;
 private:
  bool gps_;
  bool open_ = false;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
