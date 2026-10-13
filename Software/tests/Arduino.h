#pragma once
// Host-only Arduino shim. Arduino IDE does not compile this directory.
#include <stdint.h>
#include <stddef.h>
#include <string>
#include <sstream>
#include <iomanip>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
inline int digitalPinToInterrupt(uint8_t pin) { return pin; }
void attachInterrupt(int interrupt, void (*callback)(), int mode);
class __FlashStringHelper;
#define F(text) reinterpret_cast<const __FlashStringHelper*>(text)
unsigned long micros();
unsigned long millis();
void delayMicroseconds(unsigned int value);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
void pinMode(uint8_t pin, uint8_t mode);
void analogWrite(uint8_t pin, int value);
struct FakeSerial {
  std::string output;
  void print(const __FlashStringHelper* value) { output += reinterpret_cast<const char*>(value); }
  template<typename T> void print(T value) { std::ostringstream s; s << value; output += s.str(); }
  void print(float value, int digits) { std::ostringstream s; s << std::fixed << std::setprecision(digits) << value; output += s.str(); }
  template<typename T> void println(T value) { print(value); output += '\n'; }
};
extern FakeSerial Serial;
