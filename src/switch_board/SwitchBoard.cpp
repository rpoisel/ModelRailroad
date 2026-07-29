#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <ButtonInput.h>
#include <Wire.h>

/*
 * Switch board control loop:
 * - Read two active-low buttons from the PCF8574 input expander.
 * - On each debounced press event, toggle the corresponding switch target.
 * - Command the switch servo through the PCA9685 PWM driver.
 * - After the configured move delay, update the configured LED outputs to show
 *   the commanded position. LEDs can be driven by PCF8574 or PCA9685 outputs.
 *   There is no physical position feedback.
 */

constexpr uint8_t SERVO_PWM_ADDR = 0x40;
constexpr uint8_t LED_PWM_ADDR = 0x41;
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

constexpr uint8_t SWITCH_LEFT_ANGLE_DEG = 0;
constexpr uint8_t SWITCH_RIGHT_ANGLE_DEG = 180;
constexpr uint16_t SWITCH_MOVE_DELAY_MS = 500;
constexpr uint8_t SERVO_MIN_ANGLE_DEG = 0;
constexpr uint8_t SERVO_MAX_ANGLE_DEG = 180;
constexpr uint16_t SERVO_MIN_US = 1000;
constexpr uint16_t SERVO_MAX_US = 2000;
constexpr uint8_t SERVO_FREQ = 50;
constexpr uint16_t LED_PWM_FREQ = 1000;
constexpr uint16_t LED_PWM_OFF = 0;
constexpr uint16_t LED_PWM_ON = 4095;

constexpr bool BUTTON_ACTIVE_LOW = true;
constexpr bool LED_ACTIVE_HIGH = true;
constexpr bool DEBOUNCE_ENABLED = true;
constexpr unsigned long DEBOUNCE_MS = 30;

enum class SwitchPosition : uint8_t { Left, Right };
enum class OutputDriver : uint8_t { Pcf8574, Pca9685 };

struct LedOutputConfig {
  OutputDriver driver;
  uint8_t address;
  uint8_t channel;
  bool active_high;
};

constexpr LedOutputConfig pcf8574_led(uint8_t address, uint8_t channel,
                                      bool active_high = LED_ACTIVE_HIGH) {
  return {OutputDriver::Pcf8574, address, channel, active_high};
}

constexpr LedOutputConfig pca9685_led(uint8_t address, uint8_t channel,
                                      bool active_high = LED_ACTIVE_HIGH) {
  return {OutputDriver::Pca9685, address, channel, active_high};
}

struct SwitchConfig {
  uint8_t button_pin;
  LedOutputConfig left_led;
  LedOutputConfig right_led;
  uint8_t pwm_channel;
  uint8_t left_angle_deg;
  uint8_t right_angle_deg;
};

constexpr SwitchConfig SWITCHES[SWITCH_COUNT] = {
    {
        INPUT_BUTTON_0_PIN,
        pcf8574_led(OUTPUTS_0_ADDR, SWITCH_0_LEFT_LED_PIN),
        pcf8574_led(OUTPUTS_0_ADDR, SWITCH_0_RIGHT_LED_PIN),
        SWITCH_0_PWM_CHANNEL,
        SWITCH_LEFT_ANGLE_DEG,
        SWITCH_RIGHT_ANGLE_DEG,
    },
    {
        INPUT_BUTTON_1_PIN,
        pcf8574_led(OUTPUTS_0_ADDR, SWITCH_1_LEFT_LED_PIN),
        pcf8574_led(OUTPUTS_0_ADDR, SWITCH_1_RIGHT_LED_PIN),
        SWITCH_1_PWM_CHANNEL,
        SWITCH_LEFT_ANGLE_DEG,
        SWITCH_RIGHT_ANGLE_DEG,
    },
};

struct Pcf8574OutputState {
  uint8_t address;
  uint8_t state;
};

constexpr uint8_t PCF8574_OUTPUT_COUNT = 1;
Pcf8574OutputState pcf8574_outputs[PCF8574_OUTPUT_COUNT] = {
    {OUTPUTS_0_ADDR, 0x00},
};

Adafruit_PWMServoDriver servo_pwm(SERVO_PWM_ADDR);
Adafruit_PWMServoDriver led_pwm(LED_PWM_ADDR);
ButtonInput buttons[SWITCH_COUNT] = {
    ButtonInput(DEBOUNCE_ENABLED, DEBOUNCE_MS),
    ButtonInput(DEBOUNCE_ENABLED, DEBOUNCE_MS),
};
SwitchPosition switch_positions[SWITCH_COUNT] = {
    SwitchPosition::Left,
    SwitchPosition::Left,
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

Pcf8574OutputState *find_pcf8574_output(uint8_t address) {
  for (uint8_t output_idx = 0; output_idx < PCF8574_OUTPUT_COUNT;
       output_idx++) {
    if (pcf8574_outputs[output_idx].address == address) {
      return &pcf8574_outputs[output_idx];
    }
  }

  return nullptr;
}

void set_pcf8574_led(LedOutputConfig const &led, bool on) {
  Pcf8574OutputState *output = find_pcf8574_output(led.address);
  if (output == nullptr) {
    return;
  }

  bool const write_high = led.active_high ? on : !on;

  if (write_high) {
    output->state |= pin_mask(led.channel);
  } else {
    output->state &= ~pin_mask(led.channel);
  }

  write_pcf8574(output->address, output->state);
}

void set_pca9685_led(LedOutputConfig const &led, bool on) {
  if (led.address != LED_PWM_ADDR) {
    return;
  }

  bool const write_high = led.active_high ? on : !on;
  led_pwm.setPin(led.channel, write_high ? LED_PWM_ON : LED_PWM_OFF);
}

void set_led(LedOutputConfig const &led, bool on) {
  switch (led.driver) {
  case OutputDriver::Pcf8574:
    set_pcf8574_led(led, on);
    break;
  case OutputDriver::Pca9685:
    set_pca9685_led(led, on);
    break;
  }
}

void initialize_pcf8574_outputs() {
  for (uint8_t output_idx = 0; output_idx < PCF8574_OUTPUT_COUNT;
       output_idx++) {
    write_pcf8574(pcf8574_outputs[output_idx].address,
                  pcf8574_outputs[output_idx].state);
  }
}

void update_switch_leds(uint8_t switch_idx) {
  SwitchConfig const &config = SWITCHES[switch_idx];
  SwitchPosition const position = switch_positions[switch_idx];

  set_led(config.left_led, position == SwitchPosition::Left);
  set_led(config.right_led, position == SwitchPosition::Right);
}

uint16_t servo_angle_to_microseconds(uint8_t angle_deg) {
  angle_deg = constrain(angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG);

  return map(angle_deg, SERVO_MIN_ANGLE_DEG, SERVO_MAX_ANGLE_DEG, SERVO_MIN_US,
             SERVO_MAX_US);
}

void write_servo_angle(uint8_t channel, uint8_t angle_deg) {
  servo_pwm.writeMicroseconds(channel, servo_angle_to_microseconds(angle_deg));
}

void move_switch(uint8_t switch_idx, SwitchPosition position) {
  SwitchConfig const &config = SWITCHES[switch_idx];
  uint8_t const angle_deg = position == SwitchPosition::Left
                                ? config.left_angle_deg
                                : config.right_angle_deg;

  write_servo_angle(config.pwm_channel, angle_deg);
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
  initialize_pcf8574_outputs();

  servo_pwm.begin();
  servo_pwm.setOscillatorFrequency(27000000);
  servo_pwm.setPWMFreq(SERVO_FREQ);

  led_pwm.begin();
  led_pwm.setOscillatorFrequency(27000000);
  led_pwm.setPWMFreq(LED_PWM_FREQ);

  delay(10);

  for (uint8_t switch_idx = 0; switch_idx < SWITCH_COUNT; switch_idx++) {
    move_switch(switch_idx, switch_positions[switch_idx]);
  }
}

void loop() { handle_buttons(read_pcf8574(INPUTS_0_ADDR)); }
