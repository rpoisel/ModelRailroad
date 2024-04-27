#include <Arduino.h>

#include <chrono>
#include <thread>

auto main() -> int {
  using namespace std::chrono_literals;

  setup();
  for (;;) {
    loop();
    std::this_thread::sleep_for(500ms);
  }
  return 0; // never reached
}
