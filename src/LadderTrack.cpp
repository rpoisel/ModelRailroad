#include "Util.hpp"

#include <AceRoutine.h>

#include <Arduino.h>
#include <Wire.h>

using namespace ace_routine;

bool const STRAIGHT = true;
bool const TURN = false;

using Direction = bool;
using Position = int; // Position is synonymous to Track
using SignedIndex = int;
using UnsignedIndex = size_t;

constexpr uint8_t I2C_INTERRUPT_PIN = 2;

class I2COutputModule {
public:
  I2COutputModule(uint8_t addr) : addr{addr} {}
  inline void reset() { pin_states = 0; }
  inline void set_pin_on(uint8_t pin) { pin_states |= (0x01 << pin); }
  inline void commit() {
    // first, set all pin states according to their values
    Wire.beginTransmission(addr);
    Wire.write(pin_states); // TODO might need inversion
    Wire.endTransmission();

    delay(200);

    // then, reset all pin values so that coils are no
    // more supplied with current
    Wire.beginTransmission(addr);
    Wire.write(0x00); // TODO might need inversion
    Wire.endTransmission();
  }

private:
  uint8_t const addr;
  uint8_t pin_states = 0;
};

struct IOPin {
  uint8_t module_idx;
  uint8_t pin;

  void println() const {
    Serial.print(module_idx);
    Serial.print(":");
    Serial.println(pin);
  }

  bool get_position(Position &result) const {
    struct PinMapping {
      uint8_t module_idx;
      uint8_t pin;
      Position position;
    };

    constexpr PinMapping const PIN_MAPPINGS[] = {
        {
            0,
            0,
            1,
        },
        {
            0,
            1,
            2,
        },
        {
            1,
            1,
            4,
        },
        {
            1,
            4,
            7,
        },
    };

    for (auto const &pin_mapping : PIN_MAPPINGS) {
      if (pin_mapping.pin == pin) {
        result = pin_mapping.position;
        return true;
      }
    }
    return false;
  }
};

struct Switch {
  constexpr Switch(IOPin const &pin_straight, IOPin const &pin_turn)
      : pin_straight{pin_straight}, pin_turn{pin_turn} {}
  IOPin const pin_straight;
  IOPin const pin_turn;
};

template <class OutputModules> struct SwitchAction {
  Switch const *railroad_switch;
  Direction direction;

  void perform(OutputModules &output_modules) const {
    IOPin const *module_pin = direction == STRAIGHT
                                  ? &railroad_switch->pin_straight
                                  : &railroad_switch->pin_turn;

    Serial.print("  --> SwitchAction: ");
    Serial.print(module_pin->module_idx);
    Serial.print(":");
    Serial.print(module_pin->pin);
    Serial.println(direction == STRAIGHT ? " | STRAIGHT" : " | TURN");

    output_modules[module_pin->module_idx].set_pin_on(module_pin->pin);
  }
};

template <class OutputModules> struct InstructionList {
  UnsignedIndex length;
  SwitchAction<OutputModules> actions[4];

  inline bool is_valid() const { return length > 0; }

  void perform(OutputModules &output_modules) const {
    for (UnsignedIndex idx_awl = 0; idx_awl < length; idx_awl++) {
      actions[idx_awl].perform(output_modules);
    }

    for (I2COutputModule &output_module : output_modules) {
      output_module.commit();
    }
  }
};

using I2COutputModules = Array<I2COutputModule, 5>;

class Route {
public:
  Route() {}
  Route(Position from, Position to) : from_{from}, to_{to} {}

  InstructionList<I2COutputModules> instruction_list() const {
    for (auto const &row : TABLE) {
      if (row.from == from_ && row.to == to_) {
        return row.instruction_list;
      }
    }

    return IL_INVALID;
  }

  Route &operator=(Route &&other) {
    from_ = other.from_;
    to_ = other.to_;
    return *this;
  }

private:
  Position from_;
  Position to_;

  InstructionList<I2COutputModules> const IL_INVALID = {0, {}};
  constexpr static Switch const SWITCH_1{{1, 0}, {2, 1}};
  constexpr static Switch const SWITCH_2{{3, 0}, {4, 1}};
  constexpr static Switch const SWITCH_3{{3, 5}, {3, 6}};

  template <class OutputModules> struct Line {
    Position from;
    Position to;
    InstructionList<OutputModules> instruction_list;
  };

  constexpr static Line<I2COutputModules> const TABLE[] = {
      {
          1,
          3,
          {
              2,
              {
                  {&SWITCH_1, TURN},
                  {&SWITCH_2, TURN},
              },
          },
      },
      {
          1,
          5,
          {
              1,
              {
                  {&SWITCH_3, STRAIGHT},
              },
          },
      },
  };
};

template <typename... Addresses>
Array<uint8_t, sizeof...(Addresses)> read_i2c_inputs(Addresses... addresses) {
  Array<uint8_t, sizeof...(Addresses)> result;
  unsigned i = 0;
  (
      [&](const auto &address) {
        Wire.requestFrom(static_cast<uint8_t>(address),
                         static_cast<uint8_t>(1));
        // TODO might need inversion
        result[i] = Wire.available() ? Wire.read() : 0;
        i++;
      }(addresses),
      ...);
  return result;
}

/**
 * Searches through an IO image to identify the first set pin.
 *
 * @param result the first module/pin that could be found being set
 * @param io_image the IO image to search for set pins
 * @return true if the pin could be identified
 */
template <typename T> bool is_pin_set(IOPin &result, T const &io_image) {
  for (unsigned module_idx = 0; module_idx < io_image.size(); module_idx++) {
    for (unsigned pin = 0; pin < 8; pin++) {
      if (io_image[module_idx] & (0x01 << pin)) {
        result.module_idx = module_idx;
        result.pin = pin;
        return true;
      }
    }
  }
  return false;
}

static bool i2c_input_changed = false;

class LadderTrack : public Coroutine {
public:
  int runCoroutine() override {
    COROUTINE_LOOP() {
      do {
        COROUTINE_AWAIT(i2c_input_changed);
        i2c_input_changed = false;
        i2c_inputs = read_i2c_inputs(0x27);
        rc_ok = is_pin_set(from_pin, i2c_inputs);
        if (!rc_ok) {
          continue;
        }
        rc_ok = from_pin.get_position(from_position);
      } while (!rc_ok);
      Serial.print("From position: ");
      from_pin.println();

      do {
        COROUTINE_AWAIT(i2c_input_changed);
        i2c_input_changed = false;
        i2c_inputs = read_i2c_inputs(0x27);
        rc_ok = is_pin_set(to_pin, i2c_inputs);
        if (!rc_ok) {
          continue;
        }
        rc_ok = to_pin.get_position(to_position);
      } while (!rc_ok);
      Serial.print("To position: ");
      to_pin.println();

      route = {from_position, to_position};
      instruction_list = route.instruction_list();

      if (instruction_list.is_valid()) {
        instruction_list.perform(output_modules);
      } else {
        Serial.println("  Route is unknown.");
      }
    }
  }

private:
  I2COutputModules output_modules{0x20, 0x22, 0x24, 0x26, 0x28};
  bool rc_ok;
  IOPin from_pin;
  IOPin to_pin;
  Position from_position;
  Position to_position;
  Array<uint8_t, 1> i2c_inputs;
  Route route;
  InstructionList<I2COutputModules> instruction_list;
};

LadderTrack ladder_track;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // see: https://github.com/RalphBacon/PCF8574-Pin-Extender-I2C/tree/master
  pinMode(I2C_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(
      digitalPinToInterrupt(I2C_INTERRUPT_PIN),
      []() { i2c_input_changed = true; }, CHANGE);
}

void loop() { ladder_track.runCoroutine(); }
