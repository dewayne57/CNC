#include "Motion.h"
#include "Config.h"
#include "MotionProfile.h"
#include "XGantry.h"
#include <math.h>

namespace {
long current[3] = {0, 0, 0};
unsigned long delta[3], accumulator[3], totalSteps, remaining, intervalUs, lastStepUs;
int8_t direction[3];
bool moving = false;
MotionProfile profile;

bool toSteps(const float position[3], long steps[3]) {
  for (uint8_t i = 0; i < 3; ++i) {
    if (!isfinite(position[i]) || fabsf(position[i]) > Config::MAX_COORDINATE_MM)
      return false;
    float pulses = position[i] * Config::STEPS_PER_MM[i];
    // Leave room for signed endpoint subtraction and unsigned DDA accumulation.
    if (!isfinite(pulses) || fabsf(pulses) > 1000000000.0f) return false;
    steps[i] = lroundf(pulses);
  }
  return true;
}
}

void Motion::begin() {
  XGantry::begin();
  for (uint8_t i = 1; i < 3; ++i) {
    digitalWrite(Config::ENABLE_PINS[i], !Config::ENABLE_ACTIVE_LEVEL);
    pinMode(Config::ENABLE_PINS[i], OUTPUT);
    digitalWrite(Config::STEP_PINS[i], LOW);
    pinMode(Config::STEP_PINS[i], OUTPUT);
    pinMode(Config::DIR_PINS[i], OUTPUT);
    pinMode(Config::HOME_PINS[i], INPUT_PULLUP);
    pinMode(Config::LIMIT_PINS[i], INPUT_PULLUP);
  }
  pinMode(Config::ESTOP_PIN, INPUT_PULLUP);
}

bool Motion::estopActive() { return digitalRead(Config::ESTOP_PIN) == LOW; }
bool Motion::homeActive(uint8_t axis) {
  return axis == 0 ? XGantry::homeSwitch(0) || XGantry::homeSwitch(1)
                   : digitalRead(Config::HOME_PINS[axis]) == LOW;
}
bool Motion::limitActive(uint8_t axis) {
  return axis == 0 ? XGantry::hardLimit()
                   : digitalRead(Config::LIMIT_PINS[axis]) == LOW;
}
bool Motion::interlockActive(uint8_t ignoredLimits) {
  if (estopActive() || XGantry::hardLimit()) return true;
  for (uint8_t i = 0; i < 3; ++i)
    if (!(ignoredLimits & (1 << i)) && limitActive(i)) return true;
  return false;
}

void Motion::enable(bool enabled) {
  XGantry::enable(enabled);
  for (uint8_t i = 1; i < 3; ++i)
    digitalWrite(Config::ENABLE_PINS[i], enabled ? Config::ENABLE_ACTIVE_LEVEL : !Config::ENABLE_ACTIVE_LEVEL);
}

bool Motion::start(const float target[3], float feedMmMin, uint8_t ignoredLimits) {
  long destination[3];
  if (moving || interlockActive(ignoredLimits) || !toSteps(target, destination) ||
      !isfinite(feedMmMin) || feedMmMin <= 0 || feedMmMin > Config::MAX_FEED_MM_MIN)
    return false;
  if (!Config::X_CALIBRATED && destination[0] != current[0]) return false;
  totalSteps = 0;
  float distanceSquared = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    long difference = destination[i] - current[i];
    direction[i] = difference >= 0 ? 1 : -1;
    delta[i] = difference >= 0 ? difference : -difference;
    accumulator[i] = 0;
    if (delta[i] > totalSteps) totalSteps = delta[i];
    float mm = difference / Config::STEPS_PER_MM[i];
    distanceSquared += mm * mm;
  }
  if (totalSteps == 0) return true;
  if (!profile.configure(sqrtf(distanceSquared), totalSteps, feedMmMin)) return false;
  intervalUs = profile.interval(0);
  enable(true);
  delayMicroseconds(Config::ENABLE_SETUP_US);
  XGantry::direction(direction[0]);
  for (uint8_t i = 1; i < 3; ++i)
    digitalWrite(Config::DIR_PINS[i], (direction[i] > 0) != Config::INVERT_DIR[i]);
  delayMicroseconds(Config::DIR_SETUP_US);
  remaining = totalSteps;
  lastStepUs = micros();
  moving = true;
  return true;
}

void Motion::update() {
  if (!moving) return;
  unsigned long now = micros();
  if (static_cast<unsigned long>(now - lastStepUs) < intervalUs) return;
  lastStepUs = now; // Never issue catch-up pulse bursts after a delayed loop.
  bool pulse[3] = {false, false, false};
  for (uint8_t i = 0; i < 3; ++i) {
    accumulator[i] += delta[i];
    if (accumulator[i] >= totalSteps) {
      accumulator[i] -= totalSteps;
      pulse[i] = true;
      if (i == 0) XGantry::pulseHigh(direction[i]);
      else digitalWrite(Config::STEP_PINS[i], HIGH);
      current[i] += direction[i];
    }
  }
  delayMicroseconds(Config::STEP_PULSE_US);
  for (uint8_t i = 0; i < 3; ++i)
    if (pulse[i]) {
      if (i == 0) XGantry::pulseLow();
      else digitalWrite(Config::STEP_PINS[i], LOW);
    }
  if (--remaining == 0) moving = false;
  else intervalUs = profile.interval(totalSteps - remaining);
}

bool Motion::busy() { return moving; }
void Motion::stop() { moving = false; enable(false); }
void Motion::halt() { moving = false; }
void Motion::position(float result[3]) {
  for (uint8_t i = 0; i < 3; ++i) result[i] = current[i] / Config::STEPS_PER_MM[i];
}
bool Motion::setPosition(const float position[3]) {
  long steps[3];
  if (moving || !toSteps(position, steps)) return false;
  for (uint8_t i = 0; i < 3; ++i) current[i] = steps[i];
  return true;
}
