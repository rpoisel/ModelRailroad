# Model Railroad

## Switch Board

`src/switch_board/SwitchBoard.cpp` controls two railroad switches from two
buttons on an I2C input expander.

### I2C Devices

| Device | Hex Address | Decimal Address | Role | Module Name |
| --- | --- | --- | --- | --- |
| `Adafruit_PWMServoDriver` / PCA9685 | `0x41` | `65` | Servo PWM outputs | Outputs 1 |
| `I2EOK` / `PCF8574AN` | `0x71` | `112 + 1` | Button inputs | Inputs 0 |
| `I2AOK` / `PCF857N` | `0x40` | `64` | LED outputs | Outputs 0 |

### Mapping

- Input expander pin 0: button for switch 0.
- Input expander pin 1: button for switch 1.
- Output expander pin 0: switch 0 left LED.
- Output expander pin 1: switch 0 right LED.
- Output expander pin 2: switch 1 left LED.
- Output expander pin 3: switch 1 right LED.
- PWM channel 0: switch 0 servo.
- PWM channel 1: switch 1 servo.

### Behavior

Each button press toggles the matching switch between left and right. The code
commands the matching PWM channel to the configured servo pulse width, waits for
the configured movement delay, then updates the two LEDs for that switch.

The LEDs show the commanded position after the delay. The hardware does not
provide physical position feedback, so the firmware cannot verify that the
switch actually reached the requested position.

Button handling uses edge detection so holding a button does not repeatedly
toggle a switch. Debounce is isolated in `ButtonInput`; set
`DEBOUNCE_ENABLED` to `false` in `SwitchBoard.cpp` to bypass debounce timing
while keeping press-edge detection.
