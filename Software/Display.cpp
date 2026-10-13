#include "Display.h"
#include "Arduino.h"
#include "Config.h"
#include "Motion.h"

namespace {
constexpr unsigned long UPDATE_INTERVAL_MS = 100;
unsigned long lastUpdate;
uint8_t nextRow;
Display::Operation operation = Display::Operation::IDLE;

void pulseEnable() {
  digitalWrite(Config::LCD_ENABLE_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(Config::LCD_ENABLE_PIN, LOW);
  delayMicroseconds(50);
}

void writeByte(uint8_t value, bool data) {
  digitalWrite(Config::LCD_RS_PIN, data ? HIGH : LOW);
  digitalWrite(Config::LCD_RW_PIN, LOW);
  for (uint8_t bit = 0; bit < 8; ++bit)
    digitalWrite(Config::LCD_DATA_PINS[bit], (value >> bit) & 1);
  pulseEnable();
}

void command(uint8_t value) {
  writeByte(value, false);
  if (value == 0x01 || value == 0x02) delayMicroseconds(1600);
}

void line(uint8_t row, const char* value) {
  static const uint8_t addresses[4] = {0x00, 0x40, 0x14, 0x54};
  command(0x80 | addresses[row]);
  uint8_t column = 0;
  while (*value && column < 20) {
    writeByte(static_cast<uint8_t>(*value++), true);
    ++column;
  }
  while (column++ < 20) writeByte(' ', true);
}

const char* operationText() {
  switch (operation) {
    case Display::Operation::MOVING: return "MOVING";
    case Display::Operation::HOMING: return "HOMING";
    case Display::Operation::DWELLING: return "DWELL";
    case Display::Operation::SPINDLE: return "SPINDLE";
    case Display::Operation::ALARM: return "ALARM";
    default: return "IDLE";
  }
}

void formatPosition(char* result, char axis, float value) {
  char number[12];
  dtostrf(value, 7, 2, number);
  result[0] = axis;
  result[1] = ':';
  result[2] = ' ';
  for (uint8_t i = 0; i < 7; ++i) result[i + 3] = number[i];
  result[10] = '\0';
}
}

void Display::begin() {
  pinMode(Config::LCD_RS_PIN, OUTPUT);
  pinMode(Config::LCD_RW_PIN, OUTPUT);
  pinMode(Config::LCD_ENABLE_PIN, OUTPUT);
  for (uint8_t pin : Config::LCD_DATA_PINS) pinMode(pin, OUTPUT);
  digitalWrite(Config::LCD_RW_PIN, LOW);
  delayMicroseconds(15000);
  writeByte(0x38, false);
  delayMicroseconds(5000);
  writeByte(0x38, false);
  delayMicroseconds(150);
  command(0x08);
  command(0x01);
  command(0x06);
  command(0x0C);
}

void Display::update() {
  if (static_cast<unsigned long>(millis() - lastUpdate) < UPDATE_INTERVAL_MS) return;
  lastUpdate = millis();
  float position[3];
  Motion::position(position);
  char row[11];
  if (nextRow == 0) line(0, operationText());
  else if (nextRow == 1) { formatPosition(row, 'X', position[0]); line(1, row); }
  else if (nextRow == 2) { formatPosition(row, 'Y', position[1]); line(2, row); }
  else { formatPosition(row, 'Z', position[2]); line(3, row); }
  nextRow = (nextRow + 1) % 4;
}

void Display::setOperation(Display::Operation value) {
  operation = value;
}
