# Model Railroad

Arduino firmware for model railroad control modules. The project is built with
PlatformIO for an Arduino Uno target and contains several selectable firmware
environments for the main ladder-track controller, switch-board controller, and
hardware test sketches.

## Project Layout

| Path | Purpose |
| --- | --- |
| `src/ladder_track/` | Main route-selection sketch for ladder-track switching. |
| `src/switch_board/` | Two-button, two-servo switch-board controller. |
| `src/button_test/` | Serial test sketch for PCF8574 button inputs. |
| `src/servo_test/` | Serial test sketch for PCA9685 servo movement. |
| `src/i2c_scan/` | I2C bus scanner sketch. |
| `include/` | Shared helpers such as `ButtonInput` and small utility functions. |
| `mock/` | Local Arduino/Wire mocks used by the experimental CMake setup. |

## Build

The Makefile wraps PlatformIO through `uv`.

```sh
make build
make build-all
```

By default, `make build` builds the `ladder_track` environment. To build a
specific environment:

```sh
make build PLATFORMIO_ENV=switch_board
make build PLATFORMIO_ENV=button_test
make build PLATFORMIO_ENV=servo_test
make build PLATFORMIO_ENV=i2c_scan
```

To upload, set the environment and serial port as needed:

```sh
make upload PLATFORMIO_ENV=switch_board UPLOAD_PORT=/dev/ttyUSB0
```

The configured PlatformIO environments are:

| Environment | Sketch | Notes |
| --- | --- | --- |
| `ladder_track` | `src/ladder_track/LadderTrack.cpp` | Default firmware. Uses AceRoutine and PCF8574-style I2C I/O modules. |
| `switch_board` | `src/switch_board/SwitchBoard.cpp` | Drives two switch servos and their position LEDs. |
| `button_test` | `src/button_test/ButtonTest.cpp` | Prints debounced button press events over serial. |
| `servo_test` | `src/servo_test/ServoTest.cpp` | Sweeps PCA9685 servo channels for calibration/testing. |
| `i2c_scan` | `src/i2c_scan/I2cScan.cpp` | Prints detected I2C device addresses. |

## Switch Board

`src/switch_board/SwitchBoard.cpp` controls two railroad switches from two
active-low buttons on an I2C input expander.

### I2C Devices

The firmware uses 7-bit I2C addresses, matching the addresses printed by the
`i2c_scan` sketch.

| Device | Address | Role | Constant |
| --- | --- | --- | --- |
| PCA9685 | `0x40` | Servo PWM outputs | `SERVO_PWM_ADDR` |
| PCA9685 | `0x41` | Optional LED PWM outputs | `LED_PWM_ADDR` |
| PCF8574-compatible input expander | `0x38` | Button inputs | `INPUTS_0_ADDR` |
| PCF8574-compatible output expander | `0x21` | LED outputs | `OUTPUTS_0_ADDR` |

### Mapping

Default switch-board mapping:

| Function | Pin/channel |
| --- | --- |
| Switch 0 button | Input expander pin 0 |
| Switch 1 button | Input expander pin 1 |
| Switch 0 left LED | PCF8574 output pin 0 |
| Switch 0 right LED | PCF8574 output pin 1 |
| Switch 1 left LED | PCF8574 output pin 2 |
| Switch 1 right LED | PCF8574 output pin 3 |
| Switch 0 servo | PCA9685 servo channel 0 |
| Switch 1 servo | PCA9685 servo channel 1 |

LEDs are configured with `LedOutputConfig`. Each LED can be assigned to either a
PCF8574 output pin with `pcf8574_led(...)` or a PCA9685 PWM channel with
`pca9685_led(...)`. The current default mapping uses the PCF8574 output
expander.

### Behavior

Each debounced button press toggles the matching switch between left and right.
The firmware writes the configured servo angle to the matching PCA9685 channel,
waits for `SWITCH_MOVE_DELAY_MS`, then updates that switch's two LEDs.

Servo endpoints are configured as angles:

| Constant | Default |
| --- | --- |
| `SWITCH_LEFT_ANGLE_DEG` | `0` |
| `SWITCH_RIGHT_ANGLE_DEG` | `180` |
| `SERVO_MIN_US` | `1000` |
| `SERVO_MAX_US` | `2000` |
| `SERVO_FREQ` | `50 Hz` |

The angle is constrained to `SERVO_MIN_ANGLE_DEG` through
`SERVO_MAX_ANGLE_DEG` and mapped to the configured microsecond pulse range
before writing to the servo driver.

The LEDs show the commanded position after the movement delay. The hardware
does not provide physical position feedback, so the firmware cannot verify that
the switch reached the requested position.

Button handling uses edge detection so holding a button does not repeatedly
toggle a switch. Debounce is isolated in `ButtonInput`; set
`DEBOUNCE_ENABLED` to `false` in `SwitchBoard.cpp` to bypass debounce timing
while keeping press-edge detection.

## Ladder Track

`src/ladder_track/LadderTrack.cpp` waits for two input selections: a source
position and a destination position. It resolves that pair through an internal
route table and pulses the configured I2C output modules for the required switch
actions.

Current ladder-track details:

| Item | Value |
| --- | --- |
| Interrupt pin | Arduino pin `2` |
| Input module read by firmware | `0x27` |
| Output modules | `0x20`, `0x22`, `0x24`, `0x26`, `0x28` |

The route table is currently defined directly in `Route::TABLE` inside
`LadderTrack.cpp`.

## Missing Documentation

The README is now aligned with the current source, but these sections still need
project-specific detail before the documentation is complete:

- Wiring diagrams or pinout tables for the actual hardware modules.
- Power requirements and servo/LED supply notes.
- Ladder-track route table documentation outside the source code.
- Servo calibration procedure for choosing angle and pulse-range constants.
- Serial monitor examples for each test environment.
- Decision on whether the CMake/mock workflow is still supported; the current
  root `CMakeLists.txt` does not match the current source layout.

## License

This project is licensed under GPL-3.0; see `LICENSE`.
