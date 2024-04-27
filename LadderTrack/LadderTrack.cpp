#include <Arduino.h>
#include <Wire.h>

bool const STRAIGHT = true;
bool const TURN = false;

using Direction = bool;
using Position = int; // Position is synonymous to Track
using SignedIndex = int;
using UnsignedIndex = size_t;
SignedIndex const IDX_INVALID = -1;

class OutputModule {
public:
  OutputModule(uint8_t addr) : addr{addr} {}
  inline void reset() { pin_states = 0; }
  inline void set_pin_on(uint8_t pin) { pin_states |= (0x01 << pin); }
  inline void commit() {
    // first, set all pin states according to their values
    Wire.beginTransmission(addr);
    Wire.write(pin_states); // TODO might need inversion
    Wire.endTransmission();

    delay(200);

    // then, reset all pin values so that coils are no more supplied with
    // current
    Wire.beginTransmission(addr);
    Wire.write(0x00); // TODO might need inversion
    Wire.endTransmission();
  }

private:
  uint8_t const addr;
  uint8_t pin_states = 0;
};

struct OutputModulePin {
  uint8_t module_idx;
  uint8_t pin;
};

OutputModule output_modules[]{
    0x20, 0x22, 0x24, 0x26, 0x28,
};

struct Switch {
  Switch(OutputModulePin const &pin_straight, OutputModulePin const &pin_turn)
      : pin_straight{pin_straight}, pin_turn{pin_turn} {}
  OutputModulePin const pin_straight;
  OutputModulePin const pin_turn;
};

struct SwitchAction {
  Switch const *railroad_switch;
  Direction direction;

  void perform() const {
    OutputModulePin const *module_pin = direction == STRAIGHT
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

struct InstructionList {
  UnsignedIndex length;
  SwitchAction actions[4];

  inline bool is_valid() const { return length > 0; }

  void perform() const {
    for (UnsignedIndex idx_awl = 0; idx_awl < length; idx_awl++) {
      actions[idx_awl].perform();
    }

    for (OutputModule &output_module : output_modules) {
      output_module.commit();
    }
  }
};

InstructionList const IL_INVALID = {0, {}};

struct Route {
  Position from;
  Position to;

  void print() const {
    Serial.print("Route: ");
    Serial.print(from);
    Serial.print(" => ");
    Serial.print(to);
  }
};

bool operator==(Route const &first, Route const &second) {
  return first.from == second.from && first.to == second.to;
}

struct Line {
  Route route;
  InstructionList instruction_list;
};

struct PinMapping {
  uint8_t pin;
  Position position;
};

Switch const SWITCH_1{{1, 0}, {2, 1}};
Switch const SWITCH_2{{3, 0}, {4, 1}};
Switch const SWITCH_3{{3, 5}, {3, 6}};

Line const TABLE[] = {
    // First line
    {
        {1, 3}, // Route
        {
            // Instruction List
            2, // Number of elements in this list
            // Instructions
            {
                {&SWITCH_1, TURN},
                {&SWITCH_2, TURN},
            },
        },
    },
    // Second line
    {
        {1, 5}, // Route
        {
            // Instruction list
            1, // Number of elements in this list
            // Instructions
            {
                {&SWITCH_3, STRAIGHT},
            },
        },
    },
    // ..
};

PinMapping const PIN_MAPPINGS[] = {
    {
        2, // digital pin
        1, // position (= track)
    },
    {
        3, // digital pin
        2, // position (= track)
    },
    {
        4, // digital pin
        4, // position (= track)
    },
    {
        5, // digital pin
        7, // position (= track)
    },
    // ...
};

struct Pins {
  static size_t const NUM_PIN_MAPPINGS =
      sizeof(PIN_MAPPINGS) / sizeof(PIN_MAPPINGS[0]);

  bool states[NUM_PIN_MAPPINGS];

  Pins() {
    for (UnsignedIndex index = 0; index < NUM_PIN_MAPPINGS; index++) {
      states[index] = digitalRead(PIN_MAPPINGS[index].pin);
    }
  }

  SignedIndex diff(Pins const &other) const {
    for (UnsignedIndex index = 0; index < NUM_PIN_MAPPINGS; index++) {
      if (!states[index] && other.states[index]) {
        return index;
      }
    }
    return IDX_INVALID;
  }
};

Position determine_button_position() {
  Pins pin_states_prev{};
  SignedIndex pin_idx_pressed = IDX_INVALID;

  while (pin_idx_pressed == IDX_INVALID) {
    Pins pin_states_cur{};
    pin_idx_pressed = pin_states_prev.diff(pin_states_cur);
    pin_states_prev = pin_states_cur;
  }

  return PIN_MAPPINGS[pin_idx_pressed].position;
}

InstructionList determine_instruction_list(Route const &route) {
  for (Line const &row : TABLE) {
    if (row.route == route) {
      return row.instruction_list;
    }
  }

  return IL_INVALID;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  for (PinMapping const &pin_mapping : PIN_MAPPINGS) {
    pinMode(pin_mapping.pin, INPUT_PULLUP);
  }
}

void loop() {
  // see: https://github.com/RalphBacon/PCF8574-Pin-Extender-I2C/tree/master
  int from_position = determine_button_position();
  int to_position = determine_button_position();

  Route route{from_position, to_position};
  route.print();

  InstructionList instruction_list = determine_instruction_list(route);

  if (instruction_list.is_valid()) {
    instruction_list.perform();
  } else {
    Serial.println("  Route is unknown.");
  }
}
