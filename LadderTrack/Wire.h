#pragma once

#include <Arduino.h>

class Wire_ {
public:
  void begin() {}
  void beginTransmission(uint8_t addr) { (void)addr; }
  void endTransmission() {}
  void write(uint8_t value) { (void)value; }
};

extern Wire_ Wire;
