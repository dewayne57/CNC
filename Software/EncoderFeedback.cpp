#include "EncoderFeedback.h"
#include "Config.h"
#include "XGantry.h"
#include <math.h>
#if defined(ARDUINO_ARCH_AVR)
#include <util/atomic.h>
#endif

namespace {
volatile uint32_t counts[2] = {0, 0}, invalid[2] = {0, 0};
volatile uint8_t previous[2] = {0, 0};
uint32_t countReference[2] = {0, 0}, stepReference[2] = {0, 0};
EncoderFeedback::Fault latched = EncoderFeedback::NONE;
unsigned long lastSample, exceededAt[3];
bool exceeded[3] = {false, false, false};
#if defined(ARDUINO_ARCH_AVR)
volatile uint8_t* inputRegisters[2];
uint8_t aMasks[2], bMasks[2];
#endif

uint8_t readAB(uint8_t side) {
#if defined(ARDUINO_ARCH_AVR)
  // Both channels of each configured pair share a port: sample them together.
  uint8_t bits = *inputRegisters[side];
  return ((bits & aMasks[side]) ? 2 : 0) | ((bits & bMasks[side]) ? 1 : 0);
#else
  return (digitalRead(Config::X_ENCODER_A_PINS[side]) ? 2 : 0) |
         (digitalRead(Config::X_ENCODER_B_PINS[side]) ? 1 : 0);
#endif
}
void edge(uint8_t side) {
  uint8_t next = readAB(side), old = previous[side];
  previous[side] = next;
  if ((old ^ next) == 3) { ++invalid[side]; return; }
  // Positive sequence: 00 -> 01 -> 11 -> 10 -> 00.
  static const int8_t transitions[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
  int8_t change = transitions[(old << 2) | next];
  if (Config::X_ENCODER_INVERT[side]) change = -change;
  counts[side] += static_cast<uint32_t>(change);
}
void x1Edge() { edge(0); }
void x2Edge() { edge(1); }
void snapshot(uint32_t position[2], uint32_t errors[2]) {
#if defined(ARDUINO_ARCH_AVR)
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
#endif
    for (uint8_t side = 0; side < 2; ++side) { position[side] = counts[side]; errors[side] = invalid[side]; }
#if defined(ARDUINO_ARCH_AVR)
  }
#endif
}
}

void EncoderFeedback::begin() {
  for (uint8_t side = 0; side < 2; ++side) {
    pinMode(Config::X_ENCODER_A_PINS[side], INPUT_PULLUP);
    pinMode(Config::X_ENCODER_B_PINS[side], INPUT_PULLUP);
#if defined(ARDUINO_ARCH_AVR)
    inputRegisters[side] = portInputRegister(digitalPinToPort(Config::X_ENCODER_A_PINS[side]));
    aMasks[side] = digitalPinToBitMask(Config::X_ENCODER_A_PINS[side]);
    bMasks[side] = digitalPinToBitMask(Config::X_ENCODER_B_PINS[side]);
#endif
    counts[side] = invalid[side] = 0;
    previous[side] = readAB(side);
  }
  attachInterrupt(digitalPinToInterrupt(Config::X_ENCODER_A_PINS[0]), x1Edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Config::X_ENCODER_B_PINS[0]), x1Edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Config::X_ENCODER_A_PINS[1]), x2Edge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Config::X_ENCODER_B_PINS[1]), x2Edge, CHANGE);
  latched = NONE;
  for (uint8_t i = 0; i < 3; ++i) exceeded[i] = false;
  uint32_t unused[2]; snapshot(countReference, unused);
  XGantry::commandedSteps(stepReference);
  lastSample = micros();
}
void EncoderFeedback::status(Status& result) {
  uint32_t raw[2], steps[2];
  snapshot(raw, result.invalid);
  XGantry::commandedSteps(steps);
  for (uint8_t side = 0; side < 2; ++side) {
    result.counts[side] = static_cast<int32_t>(raw[side] - countReference[side]);
    int32_t commanded = static_cast<int32_t>(steps[side] - stepReference[side]);
    result.errorCounts[side] = Config::X_CALIBRATED
        ? result.counts[side] - commanded / Config::X_STEPS_PER_ENCODER_COUNT : 0;
  }
  result.fault = latched;
}
void EncoderFeedback::update() {
  unsigned long now = micros();
  if (latched != NONE || static_cast<unsigned long>(now - lastSample) < Config::X_ENCODER_SAMPLE_US) return;
  lastSample = now;
  Status measured; status(measured);
  if (measured.invalid[0] >= Config::X_MAX_INVALID_TRANSITIONS ||
      measured.invalid[1] >= Config::X_MAX_INVALID_TRANSITIONS) { latched = INVALID_TRANSITION; return; }
  if (!Config::X_CALIBRATED) return;
  float errors[3] = {fabsf(measured.errorCounts[0]), fabsf(measured.errorCounts[1]),
                     fabsf(measured.errorCounts[0] - measured.errorCounts[1])};
  const Fault reasons[3] = {X1_FOLLOWING, X2_FOLLOWING, SKEW};
  for (uint8_t i = 0; i < 3; ++i) {
    float threshold = i == 2 ? Config::X_MAX_SKEW_ERROR_COUNTS : Config::X_MAX_FOLLOWING_ERROR_COUNTS;
    if (errors[i] <= threshold) { exceeded[i] = false; continue; }
    if (!exceeded[i]) { exceeded[i] = true; exceededAt[i] = millis(); }
    if (static_cast<unsigned long>(millis() - exceededAt[i]) >= Config::X_ENCODER_FAULT_DELAY_MS) {
      latched = reasons[i]; return;
    }
  }
}
EncoderFeedback::Fault EncoderFeedback::fault() { return latched; }
void EncoderFeedback::report() {
  Status measured; status(measured);
  Serial.print(F("enc X1:")); Serial.print(measured.counts[0]);
  Serial.print(F(" X2:")); Serial.print(measured.counts[1]);
  Serial.print(F(" err1:")); Serial.print(measured.errorCounts[0], 2);
  Serial.print(F(" err2:")); Serial.print(measured.errorCounts[1], 2);
  Serial.print(F(" invalid1:")); Serial.print(measured.invalid[0]);
  Serial.print(F(" invalid2:")); Serial.print(measured.invalid[1]);
  Serial.print(F(" fault:")); Serial.println(static_cast<int>(latched));
}
