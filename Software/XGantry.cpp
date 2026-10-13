#include "XGantry.h"
#include "Config.h"

namespace {
constexpr uint8_t stepPins[2] = {Config::STEP_PINS[0], Config::X2_STEP_PIN};
constexpr uint8_t dirPins[2] = {Config::DIR_PINS[0], Config::X2_DIR_PIN};
constexpr uint8_t enablePins[2] = {Config::ENABLE_PINS[0], Config::X2_ENABLE_PIN};
constexpr bool inverted[2] = {Config::INVERT_DIR[0], Config::X2_INVERT_DIR};
uint32_t emitted[2] = {0, 0};
uint8_t pulseMask = 3;
}
void XGantry::begin() {
  pulseMask = 3;
  for (uint8_t side = 0; side < 2; ++side) {
    emitted[side] = 0;
    digitalWrite(enablePins[side], !Config::ENABLE_ACTIVE_LEVEL);
    pinMode(enablePins[side], OUTPUT);
    digitalWrite(stepPins[side], LOW);
    pinMode(stepPins[side], OUTPUT);
    pinMode(dirPins[side], OUTPUT);
  }
  pinMode(Config::LIMIT_PINS[0], INPUT_PULLUP);
  pinMode(Config::X2_HOME_PIN, INPUT_PULLUP);
  for (uint8_t side = 0; side < 2; ++side) pinMode(Config::X_HARD_LIMIT_PINS[side], INPUT_PULLUP);
}
void XGantry::enable(bool enabled) {
  for (uint8_t side = 0; side < 2; ++side)
    digitalWrite(enablePins[side], enabled ? Config::ENABLE_ACTIVE_LEVEL : !Config::ENABLE_ACTIVE_LEVEL);
}
void XGantry::direction(int8_t sign) {
  for (uint8_t side = 0; side < 2; ++side)
    digitalWrite(dirPins[side], (sign > 0) != inverted[side]);
}
void XGantry::pulseHigh(int8_t sign) {
  for (uint8_t side = 0; side < 2; ++side) {
    if (!(pulseMask & (1 << side))) continue;
    digitalWrite(stepPins[side], HIGH);
    // Unsigned arithmetic makes long-running counters wrap predictably.
    emitted[side] += sign > 0 ? uint32_t(1) : uint32_t(-1);
  }
}
void XGantry::pulseLow() {
  for (uint8_t side = 0; side < 2; ++side) digitalWrite(stepPins[side], LOW);
}
bool XGantry::homeSwitch(uint8_t side) {
  return digitalRead(side == 0 ? Config::LIMIT_PINS[0] : Config::X2_HOME_PIN) == LOW;
}
bool XGantry::hardLimit() {
  return digitalRead(Config::X_HARD_LIMIT_PINS[0]) == LOW ||
         digitalRead(Config::X_HARD_LIMIT_PINS[1]) == LOW;
}
void XGantry::setHomeMask(uint8_t mask) { pulseMask = mask & 3; }
uint8_t XGantry::homeMask() { return pulseMask; }
void XGantry::commandedSteps(uint32_t result[2]) {
  for (uint8_t side = 0; side < 2; ++side) result[side] = emitted[side];
}
