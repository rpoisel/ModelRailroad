#include <Arduino.h>
#include <ButtonInput.h>
#include <Wire.h>

/*
 * Button test:
 * - Read two active-low buttons from the PCF8574 input expander.
 * - Print one serial trace line for each debounced press event.
 */

constexpr uint8_t INPUTS_0_ADDR = 0x38;

constexpr uint8_t BUTTON_COUNT = 2;
constexpr uint8_t INPUT_BUTTON_0_PIN = 0;
constexpr uint8_t INPUT_BUTTON_1_PIN = 1;

constexpr bool BUTTON_ACTIVE_LOW = true;
constexpr bool DEBOUNCE_ENABLED = true;
constexpr unsigned long DEBOUNCE_MS = 30;

constexpr uint8_t BUTTON_PINS[BUTTON_COUNT] = {
    INPUT_BUTTON_0_PIN,
    INPUT_BUTTON_1_PIN,
};

ButtonInput buttons[BUTTON_COUNT] = {
    ButtonInput(DEBOUNCE_ENABLED, DEBOUNCE_MS),
    ButtonInput(DEBOUNCE_ENABLED, DEBOUNCE_MS),
};

void write_pcf8574(uint8_t address, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read_pcf8574(uint8_t address) {
  Wire.requestFrom(address, static_cast<uint8_t>(1));
  return Wire.available() ? Wire.read() : 0xFF;
}

void print_hex_byte(uint8_t value) {
  if (value < 0x10) {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void handle_buttons(uint8_t input_state) {
  for (uint8_t button_idx = 0; button_idx < BUTTON_COUNT; button_idx++) {
    uint8_t const pin = BUTTON_PINS[button_idx];
    bool const pressed_now =
        is_bit_active(input_state, pin, !BUTTON_ACTIVE_LOW);

    if (buttons[button_idx].pressed_event(pressed_now)) {
      Serial.print("button=");
      Serial.print(button_idx);
      Serial.print(" pin=");
      Serial.print(pin);
      Serial.print(" pressed input_state=0x");
      print_hex_byte(input_state);
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  write_pcf8574(INPUTS_0_ADDR, 0xFF);

  Serial.println("Button test ready");
}

void loop() { handle_buttons(read_pcf8574(INPUTS_0_ADDR)); }
