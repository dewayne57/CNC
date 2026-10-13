#include "Homing.h"
#include "Config.h"
#include "Motion.h"
#include "GantryHoming.h"

namespace {
enum Phase : uint8_t { CLEAR, SEEK, RETRACT, LATCH, PULL_OFF };
Phase phase;
uint8_t remainingAxes = 0, axis = 0;
bool active = false, fault = false, settling = false;
unsigned long phaseStart, settleStart;

void fail() { Motion::stop(); GantryHoming::cancel(); active = false; fault = true; remainingAxes = 0; }

bool move(Phase next, float distance, float feed) {
  phase = next;
  phaseStart = millis();
  settling = false;
  float target[3]; Motion::position(target);
  target[axis] += distance * Config::HOME_DIRECTION[axis];
  if (!Motion::start(target, feed, remainingAxes)) { fail(); return false; }
  return true;
}

void nextAxis() {
  if (!remainingAxes) { active = false; return; }
  for (uint8_t i = 0; i < 3; ++i)
    if (remainingAxes & (1 << Config::HOME_ORDER[i])) { axis = Config::HOME_ORDER[i]; break; }
  if (axis == 0) {
    if (!GantryHoming::start(remainingAxes)) fail();
    return;
  }
  if (Motion::limitActive(axis))
    move(CLEAR, -Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN);
  else move(SEEK, Config::HOME_MAX_TRAVEL_MM[axis], Config::HOME_SEEK_MM_MIN);
}
}

bool Homing::start(uint8_t axes) {
  if (active || Motion::busy() || !axes || (axes & ~7) || Motion::interlockActive(axes)) return false;
  if ((axes & 1) && !Config::X_CALIBRATED) return false;
  remainingAxes = axes;
  fault = false;
  active = true;
  nextAxis();
  return !fault;
}

void Homing::update() {
  if (!active) return;
  if (axis == 0) {
    GantryHoming::update();
    if (GantryHoming::failed()) { fail(); return; }
    if (!GantryHoming::busy()) { remainingAxes &= ~1; nextAxis(); }
    return;
  }
  if (Motion::interlockActive(remainingAxes) ||
      static_cast<unsigned long>(millis() - phaseStart) >= Config::HOME_TIMEOUT_MS) {
    fail(); return;
  }
  bool seeking = phase == SEEK || phase == LATCH;
  if (!settling) {
    if (seeking && Motion::limitActive(axis)) {
      Motion::halt(); // Capture contact before issuing another pulse.
      settling = true; settleStart = millis();
    } else if (!Motion::busy()) {
      if (seeking) { fail(); return; } // Travel exhausted without contact.
      settling = true; settleStart = millis();
    }
    return;
  }
  // Require the expected level continuously throughout the settle window.
  if (Motion::limitActive(axis) != seeking) { settleStart = millis(); return; }
  if (static_cast<unsigned long>(millis() - settleStart) < Config::HOME_DEBOUNCE_MS) return;
  switch (phase) {
    case CLEAR:
      move(SEEK, Config::HOME_MAX_TRAVEL_MM[axis], Config::HOME_SEEK_MM_MIN); break;
    case SEEK:
      move(RETRACT, -Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN); break;
    case RETRACT:
      move(LATCH, 2 * Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN); break;
    case LATCH: {
      float position[3]; Motion::position(position);
      position[axis] = Config::HOME_SWITCH_POSITION_MM[axis];
      if (!Motion::setPosition(position)) { fail(); return; }
      move(PULL_OFF, -Config::HOME_PULL_OFF_MM, Config::HOME_LATCH_MM_MIN); break;
    }
    case PULL_OFF:
      remainingAxes &= ~(1 << axis);
      nextAxis(); break;
  }
}

void Homing::cancel() { GantryHoming::cancel(); active = fault = false; remainingAxes = 0; }
bool Homing::busy() { return active; }
bool Homing::failed() { return fault; }
uint8_t Homing::ignoredLimits() { return active ? remainingAxes : 0; }
