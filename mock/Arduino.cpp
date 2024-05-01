#include "Arduino.h"

void SerialPort::begin(int baudRate) {
  std::cout << "SerialPort initialized with baud rate: " << baudRate
            << std::endl;
}

void SerialPort::print(const char *message) { std::cout << message; }

void SerialPort::print(int number) { std::cout << number; }

void SerialPort::println(const char *message) {
  std::cout << message << std::endl;
}

void SerialPort::println(int number) { std::cout << number << std::endl; }

SerialPort Serial;

void delay(int millis) {
  std::cout << "Delaying for " << millis << " milliseconds" << std::endl;
}

void pinMode(int pin, int mode) {
  std::cout << "Setting pin " << pin << " mode to "
            << (mode == OUTPUT ? "OUTPUT" : "INPUT_PULLUP") << std::endl;
}

void digitalWrite(int pin, int level) {
  std::cout << "Setting pin " << pin << " to voltage "
            << (level == HIGH ? "HIGH" : "LOW") << std::endl;
}

int digitalRead(int pin) {
  std::cout << "Reading value from pin " << pin << std::endl;
  return LOW;
}
