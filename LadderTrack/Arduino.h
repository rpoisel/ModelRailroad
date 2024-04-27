#pragma once

#include <cstdint>
#include <iostream>

using std::uint8_t;

constexpr int OUTPUT = 0;
constexpr int INPUT_PULLUP = 1;
constexpr int HIGH = 1;
constexpr int LOW = 0;

struct SerialPort {
  void begin(int baudRate);
  void print(const char *message);
  void print(int number);
  void println(const char *message);
  void println(int number);
};

extern SerialPort Serial;
extern void delay(int millis);
extern void pinMode(int pin, int mode);
extern void digitalWrite(int pin, int level);
extern int digitalRead(int pin);

extern void setup();
extern void loop();
