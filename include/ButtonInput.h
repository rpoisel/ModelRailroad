#pragma once

#include <Arduino.h>

class ButtonInput {
public:
  explicit ButtonInput(bool debounce_enabled = true,
                       unsigned long debounce_ms = 30)
      : debounce_enabled_(debounce_enabled), debounce_ms_(debounce_ms) {}

  bool pressed_event(bool pressed_now) {
    if (!debounce_enabled_) {
      bool const result = !last_reported_pressed_ && pressed_now;
      last_reported_pressed_ = pressed_now;
      return result;
    }

    unsigned long const now = millis();
    if (pressed_now != last_raw_pressed_) {
      last_raw_pressed_ = pressed_now;
      last_changed_ms_ = now;
    }

    if ((now - last_changed_ms_) < debounce_ms_) {
      return false;
    }

    if (last_raw_pressed_ == last_reported_pressed_) {
      return false;
    }

    last_reported_pressed_ = last_raw_pressed_;
    return last_reported_pressed_;
  }

private:
  bool debounce_enabled_;
  unsigned long debounce_ms_;
  bool last_raw_pressed_ = false;
  bool last_reported_pressed_ = false;
  unsigned long last_changed_ms_ = 0;
};

inline uint8_t pin_mask(uint8_t pin) { return 0x01 << pin; }

inline bool is_bit_active(uint8_t value, uint8_t pin, bool active_high) {
  bool const bit_is_high = (value & pin_mask(pin)) != 0;
  return active_high ? bit_is_high : !bit_is_high;
}
