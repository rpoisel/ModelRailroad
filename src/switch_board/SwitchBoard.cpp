#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <ButtonInput.h>
#include <Wire.h>

/*
 * Switch board control loop:
 * - Read two active-low buttons from the PCF8574 input expander.
 * - On each debounced press event, toggle the corresponding switch target.
 * - Command the switch servo through the PCA9685 PWM driver.
 * - After the configured move delay, update the PCF8574 LED outputs to show
 *   the commanded position. There is no physical position feedback.
 */

constexpr uint8_t PWM_ADDR = 0x40;
constexpr uint8_t INPUTS_0_ADDR = 0x38;
constexpr uint8_t OUTPUTS_0_ADDR = 0x21;

constexpr uint8_t SWITCH_COUNT = 2;
constexpr uint8_t INPUT_BUTTON_0_PIN = 0;
constexpr uint8_t INPUT_BUTTON_1_PIN = 1;

constexpr uint8_t SWITCH_0_LEFT_LED_PIN = 0;
constexpr uint8_t SWITCH_0_RIGHT_LED_PIN = 1;
constexpr uint8_t SWITCH_1_LEFT_LED_PIN = 2;
constexpr uint8_t SWITCH_1_RIGHT_LED_PIN = 3;

constexpr uint8_t SWITCH_0_PWM_CHANNEL = 0;
constexpr uint8_t SWITCH_1_PWM_CHANNEL = 1;

constexpr uint16_t SWITCH_LEFT_US = 1000;
constexpr uint16_t SWITCH_RIGHT_US = 2000;
constexpr uint16_t SWITCH_MOVE_DELAY_MS = 500;
constexpr uint8_t SERVO_FREQ = 50;

constexpr bool BUTTON_ACTIVE_LOW = true;
constexpr bool LED_ACTIVE_HIGH = true;
constexpr bool DEBOUNCE_ENABLED = true;
constexpr unsigned long DEBOUNCE_MS = 30;

enum class SwitchPosition : uint8_t { Left, Right };

struct SwitchConfig {
  uint8_t button_pin;
  uint8_t left_led_pin;
  uint8_t right_led_pin;
  uint8_t pwm_channel;
};

constexpr SwitchConfig SWITCHES[SWITCH_COUNT] = {
    {
        INPUT_BUTTON_0_PIN,
        SWITCH_0_LEFT_LED_PIN,
        SWITCH_0_RIGHT_LED_PIN,
        SWITCH_0_PWM_CHANNEL,
    },
    {
        INPUT_BUTTON_1_PIN,
        SWITCH_1_LEFT_LED_PIN,
        SWITCH_1_RIGHT_LED_PIN,
        SWITCH_1_PWM_CHANNEL,
    },
};

Adafruit_PWMServoDriver pwm(PWM_ADDR);
ButtonInput buttons[SWITCH_COUNT] = {
    ButtonInput(DEBOUNCE_ENABLED, DEBOUNCE_MS),
    ButtonInput(DEBOUNCE_ENABLED, DEBOUNCE_MS),
};
SwitchPosition switch_positions[SWITCH_COUNT] = {
    SwitchPosition::Left,
    SwitchPosition::Left,
};

uint8_t led_state = LED_ACTIVE_HIGH ? 0x00 : 0xFF;

void write_pcf8574(uint8_t address, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t read_pcf8574(uint8_t address) {
  Wire.requestFrom(address, static_cast<uint8_t>(1));
  return Wire.available() ? Wire.read() : 0xFF;
}

void set_led_pin(uint8_t pin, bool on) {
  bool const write_high = LED_ACTIVE_HIGH ? on : !on;

  if (write_high) {
    led_state |= pin_mask(pin);
  } else {
    led_state &= ~pin_mask(pin);
  }
}

void commit_leds() { write_pcf8574(OUTPUTS_0_ADDR, led_state); }

void update_switch_leds(uint8_t switch_idx) {
  SwitchConfig const &config = SWITCHES[switch_idx];
  SwitchPosition const position = switch_positions[switch_idx];

  set_led_pin(config.left_led_pin, position == SwitchPosition::Left);
  set_led_pin(config.right_led_pin, position == SwitchPosition::Right);
  commit_leds();
}

void move_switch(uint8_t switch_idx, SwitchPosition position) {
  SwitchConfig const &config = SWITCHES[switch_idx];
  uint16_t const pulse_us =
      position == SwitchPosition::Left ? SWITCH_LEFT_US : SWITCH_RIGHT_US;

  pwm.writeMicroseconds(config.pwm_channel, pulse_us);
  switch_positions[switch_idx] = position;

  delay(SWITCH_MOVE_DELAY_MS);
  update_switch_leds(switch_idx);
}

SwitchPosition other_position(SwitchPosition position) {
  return position == SwitchPosition::Left ? SwitchPosition::Right
                                          : SwitchPosition::Left;
}

void handle_buttons(uint8_t input_state) {
  for (uint8_t switch_idx = 0; switch_idx < SWITCH_COUNT; switch_idx++) {
    SwitchConfig const &config = SWITCHES[switch_idx];
    bool const pressed_now =
        is_bit_active(input_state, config.button_pin, !BUTTON_ACTIVE_LOW);

    if (buttons[switch_idx].pressed_event(pressed_now)) {
      move_switch(switch_idx, other_position(switch_positions[switch_idx]));
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  write_pcf8574(INPUTS_0_ADDR, 0xFF);
  write_pcf8574(OUTPUTS_0_ADDR, led_state);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  delay(10);

  for (uint8_t switch_idx = 0; switch_idx < SWITCH_COUNT; switch_idx++) {
    move_switch(switch_idx, switch_positions[switch_idx]);
  }
}

void loop() { handle_buttons(read_pcf8574(INPUTS_0_ADDR)); }
