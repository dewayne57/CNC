#include "Executor.h"
#include "Config.h"
#include "Motion.h"
#include "Spindle.h"
#include "ArcMotion.h"
#include "Homing.h"
#include "EncoderFeedback.h"
#include "Display.h"
#include <math.h>

namespace {
bool absoluteMode = true, inches = false, alarm = false, pending = false, dwelling = false;
int16_t modalMotion = 1;
uint8_t plane = 17;
float feed = Config::DEFAULT_FEED_MM_MIN, spindleRpm = 0;
unsigned long dwellStart, dwellDuration;
constexpr uint8_t XYZ = GCode::X | GCode::Y | GCode::Z;
void ok() { Serial.println(F("ok")); }
void error(const __FlashStringHelper* message) {
  Serial.print(F("error: ")); Serial.println(message);
}
bool only(const GCode& code, uint16_t allowed) {
  if (code.words & ~allowed) { error(F("word not valid for command")); return false; }
  return true;
}
}

void Executor::begin() {
  Motion::begin(); Spindle::begin(); EncoderFeedback::begin();
  if (!Config::X_CALIBRATED) Serial.println(F("config: set CNC_X_MICROSTEPS and CNC_X_AXLE_TRAVEL_MM before X motion"));
}
bool Executor::busy() { return pending; }

void Executor::emergencyStop() {
  ArcMotion::cancel();
  Homing::cancel();
  Motion::stop();
  Spindle::stop();
  Display::setOperation(Display::Operation::ALARM);
  pending = dwelling = false;
  if (!alarm) Serial.println(F("error: emergency stop; reset board to clear alarm"));
  alarm = true;
}

void Executor::update() {
  EncoderFeedback::update();
  if (!alarm && EncoderFeedback::fault() != EncoderFeedback::NONE) {
    emergencyStop(); // Shut outputs down before potentially blocking serial diagnostics.
    error(F("X encoder feedback fault")); EncoderFeedback::report(); return;
  }
  // Idle limits block normal commands but allow a home cycle to release them.
  // An E-stop always latches, including while idle and during homing.
  if (Motion::estopActive() || (pending && Motion::interlockActive(Homing::ignoredLimits()))) {
    emergencyStop(); return;
  }
  if (alarm) return;
  if (!pending && Motion::interlockActive()) { Motion::stop(); Spindle::stop(); }
  Homing::update(); // Detect home contact before the next pulse.
  if (Homing::failed()) { error(F("homing failed: switch or travel timeout")); emergencyStop(); return; }
  Motion::update();
  ArcMotion::update();
  if (ArcMotion::failed()) { error(F("arc motion failed")); emergencyStop(); return; }
  if (dwelling && static_cast<unsigned long>(millis() - dwellStart) >= dwellDuration)
    dwelling = false;
  if (pending && !Motion::busy() && !dwelling && !ArcMotion::busy() && !Homing::busy()) {
    pending = false;
    Display::setOperation(Display::Operation::IDLE);
    ok();
  }
}

void Executor::reportPosition() {
  float position[3]; Motion::position(position);
  Serial.print(F("X:")); Serial.print(position[0], 4);
  Serial.print(F(" Y:")); Serial.print(position[1], 4);
  Serial.print(F(" Z:")); Serial.print(position[2], 4);
  Serial.print(F(" units:mm alarm:")); Serial.println(alarm ? 1 : 0);
  EncoderFeedback::report();
}

void Executor::execute(const GCode& code) {
  if (code.m == 112 && only(code, 0)) { emergencyStop(); return; }
  if (code.m == 114 && only(code, 0)) { reportPosition(); ok(); return; }
  if (alarm || Motion::estopActive()) {
    if (!alarm) emergencyStop();
    else error(F("alarm latched; reset board"));
    return;
  }
  if (busy()) { error(F("busy; wait for ok before next command")); return; }
  if (EncoderFeedback::fault() != EncoderFeedback::NONE) { emergencyStop(); return; }
  if (code.g == 28) {
    if (!only(code, XYZ)) return;
    // Numeric zero is an axis selector; no intermediate positioning move.
    if ((code.has(GCode::X) && code.x != 0) || (code.has(GCode::Y) && code.y != 0) ||
        (code.has(GCode::Z) && code.z != 0)) {
      error(F("G28 axis selectors must be zero")); return;
    }
    Spindle::stop();
    if (!Config::X_CALIBRATED && (!code.has(XYZ) || code.has(GCode::X))) {
      error(F("configure X transmission before homing")); return;
    }
    if (!Homing::start(code.has(XYZ) ? code.words & XYZ : XYZ)) {
      error(F("cannot start homing")); return;
    }
    pending = true;
    Display::setOperation(Display::Operation::HOMING);
    return;
  }
  if (Motion::interlockActive()) {
    Spindle::stop(); error(F("limit active; home selected axes with G28")); return;
  }
  if (code.m >= 0) {
    switch (code.m) {
      case 3: case 4:
        if (!only(code, GCode::S)) return;
        if (code.has(GCode::S)) {
          if (code.s < 0 || code.s > Config::MAX_SPINDLE_RPM) {
            error(F("spindle speed out of range")); return;
          }
          spindleRpm = code.s;
        }
        Spindle::run(spindleRpm, code.m == 3);
        Display::setOperation(Display::Operation::SPINDLE); break;
      case 5: if (!only(code, 0)) return; Spindle::stop();
        Display::setOperation(Display::Operation::IDLE); break;
      case 17: if (!only(code, 0)) return; Motion::enable(true); break;
      case 18: case 84: if (!only(code, 0)) return; Motion::enable(false); break;
      case 400: if (!only(code, 0)) return; break;
      default: error(F("unsupported M command")); return;
    }
    ok(); return;
  }
  int16_t command = code.g;
  if (command < 0) {
    if (code.words == 0) { ok(); return; }
    if (code.words == GCode::F) {
      float nextFeed = code.f * (inches ? 25.4f : 1.0f);
      if (!isfinite(nextFeed) || nextFeed <= 0 || nextFeed > Config::MAX_FEED_MM_MIN) {
        error(F("feed out of range")); return;
      }
      feed = nextFeed; ok(); return;
    }
    command = modalMotion;
  }
  float scale = inches ? 25.4f : 1.0f;
  switch (command) {
    case 0: case 1: case 2: case 3: {
      bool arc = command == 2 || command == 3;
      uint16_t allowed = XYZ | GCode::F;
      if (arc) allowed |= GCode::I | GCode::J | GCode::K | GCode::R;
      if (!only(code, allowed)) return;
      if (!Config::X_CALIBRATED && (arc || code.has(GCode::X))) {
        error(F("configure X transmission before motion")); return;
      }
      float nextFeed = code.has(GCode::F) ? code.f * scale : feed;
      if (!isfinite(nextFeed) || nextFeed <= 0 || nextFeed > Config::MAX_FEED_MM_MIN) {
        error(F("feed out of range")); return;
      }
      float target[3]; Motion::position(target);
      const float values[3] = {code.x, code.y, code.z};
      for (uint8_t i = 0; i < 3; ++i)
        if (code.has(1 << i)) target[i] = (absoluteMode ? 0 : target[i]) + values[i] * scale;
      GCode resolved = code;
      resolved.g = command;
      bool started = arc ? ArcMotion::start(resolved, target, scale, nextFeed, plane)
                         : Motion::start(target, command == 0 ? Config::RAPID_MM_MIN : nextFeed);
      if (!started) {
        error(F("invalid motion target or rate")); return;
      }
      feed = nextFeed; modalMotion = command;
      if (Motion::busy() || ArcMotion::busy()) {
        pending = true;
        Display::setOperation(Display::Operation::MOVING);
      }
      else ok();
      return;
    }
    case 4:
      if (!only(code, GCode::P)) return;
      if (!code.has(GCode::P) || code.p < 0 || code.p > 3600000.0f) {
        error(F("G4 requires P milliseconds in range 0..3600000")); return;
      }
      dwellDuration = static_cast<unsigned long>(code.p);
      dwellStart = millis(); dwelling = pending = true;
      Display::setOperation(Display::Operation::DWELLING);
      return;
    case 17: case 18: case 19:
      if (!only(code, 0)) return;
      plane = command; break;
    case 20: case 21:
      if (!only(code, 0)) return;
      inches = command == 20; break;
    case 90: case 91:
      if (!only(code, 0)) return;
      absoluteMode = command == 90; break;
    case 92: {
      if (!only(code, XYZ)) return;
      if (!code.has(XYZ)) { error(F("G92 requires an axis")); return; }
      float position[3]; Motion::position(position);
      if (code.has(GCode::X)) position[0] = code.x * scale;
      if (code.has(GCode::Y)) position[1] = code.y * scale;
      if (code.has(GCode::Z)) position[2] = code.z * scale;
      if (!Motion::setPosition(position)) { error(F("coordinate out of range")); return; }
      break;
    }
    default: error(F("unsupported G command")); return;
  }
  ok();
}
