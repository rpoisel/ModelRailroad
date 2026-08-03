# Model Railroad

Arduino firmware for model railroad control modules. The project is built with
PlatformIO and currently targets Arduino Uno plus an initial Olimex ESP32-POE2
port for the I2C scanner.

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

By default, `make build` builds the `uno_ladder_track` environment. To build a
specific environment:

```sh
make build PLATFORMIO_ENV=uno_switch_board
make build PLATFORMIO_ENV=uno_button_test
make build PLATFORMIO_ENV=uno_servo_test
make build PLATFORMIO_ENV=uno_i2c_scan
make build PLATFORMIO_ENV=esp32_poe2_i2c_scan
```

To upload, set the environment and serial port as needed:

```sh
make upload PLATFORMIO_ENV=uno_switch_board UPLOAD_PORT=/dev/ttyUSB0
make upload PLATFORMIO_ENV=esp32_poe2_i2c_scan UPLOAD_PORT=/dev/ttyACM0
```

The configured PlatformIO environments are:

| Environment | Sketch | Notes |
| --- | --- | --- |
| `uno_ladder_track` | `src/ladder_track/LadderTrack.cpp` | Default Arduino Uno firmware. Uses AceRoutine and PCF8574-style I2C I/O modules. |
| `uno_switch_board` | `src/switch_board/SwitchBoard.cpp` | Arduino Uno build for driving two switch servos and their position LEDs. |
| `uno_button_test` | `src/button_test/ButtonTest.cpp` | Arduino Uno build that prints debounced button press events over serial. |
| `uno_servo_test` | `src/servo_test/ServoTest.cpp` | Arduino Uno build that sweeps PCA9685 servo channels for calibration/testing. |
| `uno_i2c_scan` | `src/i2c_scan/I2cScan.cpp` | Arduino Uno build that prints detected I2C device addresses. |
| `esp32_poe2_i2c_scan` | `src/i2c_scan/I2cScan.cpp` | ESP32-POE2 build of the I2C scanner. Uses UEXT I2C pins GPIO13 SDA and GPIO33 SCL. |

## ESP32-POE2

`esp32_poe2_i2c_scan` is the first ESP32-POE2 target. It uses the same scanner
sketch as the `uno_i2c_scan` environment, but selects Olimex's ESP32-POE2 board
definition and configures the UEXT I2C pins explicitly.

Current ESP32-POE2 details:

| Item | Value |
| --- | --- |
| PlatformIO environment | `esp32_poe2_i2c_scan` |
| Board manifest | `boards/esp32-poe2.json` |
| PlatformIO platform | `espressif32` |
| Board | `esp32-poe2` |
| Framework | Arduino |
| Serial baud | `115200` |
| UEXT SDA | GPIO13 |
| UEXT SCL | GPIO33 |

The local `boards/esp32-poe2.json` file is based on Olimex's PlatformIO board
manifest. The linker-script field is adapted to PlatformIO's current local board
schema by placing `esp32_out.ld` under `build.arduino.ldscript`.

Important hardware note from Olimex: ESP32-POE2 has no galvanic isolation from
Ethernet power. Disconnect PoE Ethernet while programming over USB unless the
setup is properly isolated.

Official Olimex resources used for this port:

| Resource | Link |
| --- | --- |
| Product page | https://www.olimex.com/Products/IoT/ESP32/ESP32-POE2/open-source-hardware |
| GitHub repository | https://github.com/OLIMEX/ESP32-POE2 |
| User manual PDF | https://github.com/OLIMEX/ESP32-POE2/blob/main/DOCUMENTS/ESP32-POE2-user-manual.pdf |
| Dimensions PDF | https://github.com/OLIMEX/ESP32-POE2/blob/main/DOCUMENTS/ESP32-PoE2-dimensions.pdf |
| UEXT pinout spreadsheet | https://github.com/OLIMEX/ESP32-POE2/blob/main/DOCUMENTS/UEXT-PINOUT.ods |
| Hardware revision notes | https://github.com/OLIMEX/ESP32-POE2/blob/main/HARDWARE/Hardware-revision-changes.txt |
| Rev. B schematic PDF | https://github.com/OLIMEX/ESP32-POE2/blob/main/HARDWARE/ESP32-PoE2_Rev_B/ESP32-PoE2_Rev_B.pdf |
| Rev. B KiCad hardware files | https://github.com/OLIMEX/ESP32-POE2/tree/main/HARDWARE/ESP32-PoE2_Rev_B |
| Official PlatformIO config | https://github.com/OLIMEX/ESP32-POE2/tree/main/SOFTWARE/PLATFORMIO |
| Official ESPHome config | https://github.com/OLIMEX/ESP32-POE2/tree/main/SOFTWARE/ESPHOME |
| Official Arduino examples | https://github.com/OLIMEX/ESP32-POE2/tree/main/SOFTWARE/ARDUINO |

The Olimex repository states that hardware is under CERN-OHL-S-2.0, software is
under GPL-3.0, documentation is under CC BY-SA 4.0, and box design files are
under CC BY 4.0. If official PDFs or hardware files are vendored into this
repository later, put them under a dedicated `docs/vendor/olimex/ESP32-POE2/`
directory and include their upstream URL, license, source revision, and download
date.

## Switch Board

`src/switch_board/SwitchBoard.cpp` controls two railroad switches from two
active-low buttons on an I2C input expander.

### I2C Devices

The firmware uses 7-bit I2C addresses, matching the addresses printed by the
`i2c_scan` sketch.

| Device | Address | Role | Constant |
| --- | --- | --- | --- |
| PCA9685 | `0x40` | Servo PWM outputs | `SERVO_PWM_ADDR` |
| PCA9685 | `0x41` | LED PWM outputs | `LED_PWM_ADDR` |
| PCF8574-compatible input expander | `0x38` | Button inputs | `INPUTS_0_ADDR` |

### Mapping

Default switch-board mapping:

| Function | Pin/channel |
| --- | --- |
| Switch 0 button | Input expander pin 0 |
| Switch 1 button | Input expander pin 1 |
| Switch 0 left LED | LED PWM channel 0 |
| Switch 0 right LED | LED PWM channel 1 |
| Switch 1 left LED | LED PWM channel 2 |
| Switch 1 right LED | LED PWM channel 3 |
| Switch 0 servo | PCA9685 servo channel 0 |
| Switch 1 servo | PCA9685 servo channel 1 |

LEDs are configured with `LedOutputConfig` and assigned to PCA9685 PWM channels
with `pca9685_led(...)`.

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
| Input read mode | Polling |
| Poll interval | `20 ms` |
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
