#include "GantryHoming.h"
#include "XGantry.h"
#include "Motion.h"
#include "Config.h"
#include <math.h>

namespace {
enum Phase : uint8_t { CLEAR, SEEK, RETRACT, LATCH, PULL_OFF };
Phase phase;
bool active = false, fault = false, settling = false;
uint8_t ignoredLimits = 0;
unsigned long phaseStart, stableAt[2];
uint32_t initialSteps[2];

void fail() {
  Motion::stop(); XGantry::setHomeMask(3);
  active = false; fault = true;
}
bool move(Phase next, float distance, float feed) {
  phase = next; settling = false;
  phaseStart = millis();
  stableAt[0] = stableAt[1] = phaseStart;
  XGantry::setHomeMask(3);
  float target[3]; Motion::position(target);
  target[0] += distance * Config::HOME_DIRECTION[0];
  if (!Motion::start(target, feed, ignoredLimits)) { fail(); return false; }
  return true;
}
}

bool GantryHoming::start(uint8_t ignoredLogicalLimits) {
  if (active || Motion::busy() || !Config::X_CALIBRATED ||
      Motion::interlockActive(ignoredLogicalLimits)) return false;
  ignoredLimits = ignoredLogicalLimits;
  active = true; fault = false;
  XGantry::commandedSteps(initialSteps);
  if (XGantry::homeSwitch(0) || XGantry::homeSwitch(1))
    return move(CLEAR, -Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN);
  return move(SEEK, Config::HOME_MAX_TRAVEL_MM[0], Config::HOME_SEEK_MM_MIN);
}

void GantryHoming::update() {
  if (!active) return;
  if (Motion::interlockActive(ignoredLimits) ||
      static_cast<unsigned long>(millis() - phaseStart) >= Config::HOME_TIMEOUT_MS) {
    fail(); return;
  }
  uint32_t steps[2]; XGantry::commandedSteps(steps);
  int32_t left = static_cast<int32_t>(steps[0] - initialSteps[0]);
  int32_t right = static_cast<int32_t>(steps[1] - initialSteps[1]);
  if (fabsf(static_cast<float>(left) - right) >
      Config::X_HOME_MAX_DIFFERENTIAL_MM * Config::STEPS_PER_MM[0]) {
    fail(); return;
  }
  bool seeking = phase == SEEK || phase == LATCH;
  if (seeking) {
    uint8_t mask = XGantry::homeMask();
    for (uint8_t side = 0; side < 2; ++side) {
      if ((mask & (1 << side)) && XGantry::homeSwitch(side)) {
        mask &= ~(1 << side); stableAt[side] = millis();
      }
      if (!XGantry::homeSwitch(side)) stableAt[side] = millis();
    }
    XGantry::setHomeMask(mask);
    if (mask == 0 && !settling) { Motion::halt(); settling = true; }
    if (!Motion::busy() && mask != 0) { fail(); return; }
  } else if (!Motion::busy() && !settling) {
    settling = true; stableAt[0] = stableAt[1] = millis();
  }
  if (!settling) return;
  bool stable = true;
  for (uint8_t side = 0; side < 2; ++side) {
    if (XGantry::homeSwitch(side) != seeking) stableAt[side] = millis();
    if (static_cast<unsigned long>(millis() - stableAt[side]) < Config::HOME_DEBOUNCE_MS) stable = false;
  }
  if (!stable) return;
  switch (phase) {
    case CLEAR: move(SEEK, Config::HOME_MAX_TRAVEL_MM[0], Config::HOME_SEEK_MM_MIN); break;
    case SEEK: move(RETRACT, -Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN); break;
    case RETRACT: move(LATCH, 2 * Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN); break;
    case LATCH: {
      float position[3]; Motion::position(position);
      position[0] = Config::HOME_SWITCH_POSITION_MM[0];
      if (!Motion::setPosition(position)) { fail(); return; }
      move(PULL_OFF, -Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN); break;
    }
    case PULL_OFF: XGantry::setHomeMask(3); active = false; break;
  }
}

void GantryHoming::cancel() { active = fault = false; XGantry::setHomeMask(3); }
bool GantryHoming::busy() { return active; }
bool GantryHoming::failed() { return fault; }
