#include "Arduino.h"
#include "Config.h"
#include "GCodeParser.h"
#include "Motion.h"
#include "MotionProfile.h"
#include "ArcMotion.h"
#include "Homing.h"
#include "Executor.h"
#include <cstdio>
#include "XGantry.h"
#include "EncoderFeedback.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <algorithm>

FakeSerial Serial;
static uint32_t clockUs = 0;
static int pins[64], limitModes[4]; // Motor order: X1,Y,Z,X2. Switches at zero.
static long physical[4], pulses[4];
static void (*interrupts[64])();
static long encoderSteps[2], encoderCounts[2];
static bool stalled[2];
static uint8_t encoderPhase[2];
static std::vector<unsigned long> xPulseTimes;
static std::vector<int> axisEvents;
static int failures = 0;
#define CHECK(expr) do { if (!(expr)) { std::cerr << __LINE__ << ": " << #expr << "\n"; ++failures; } } while (0)
unsigned long micros() { return clockUs; }
unsigned long millis() { return clockUs / 1000; }
void delayMicroseconds(unsigned int value) { clockUs += value; }
void pinMode(uint8_t, uint8_t) {}
void analogWrite(uint8_t pin, int value) { pins[pin] = value; }
char* dtostrf(double value, signed char, unsigned char precision, char* buffer) {
  std::snprintf(buffer, 16, "%.*f", precision, value);
  return buffer;
}
void attachInterrupt(int interrupt, void (*callback)(), int) { interrupts[interrupt] = callback; }
static void encoderEdge(uint8_t side, int sign) {
  static const uint8_t sequence[4] = {0, 1, 3, 2};
  encoderPhase[side] = (encoderPhase[side] + (sign > 0 ? 1 : 3)) % 4;
  uint8_t ab = sequence[encoderPhase[side]];
  uint8_t a = Config::X_ENCODER_A_PINS[side], b = Config::X_ENCODER_B_PINS[side];
  int nextA = (ab >> 1) & 1, nextB = ab & 1;
  uint8_t changed = pins[a] != nextA ? a : b;
  pins[a] = nextA; pins[b] = nextB;
  if (interrupts[changed]) interrupts[changed]();
  encoderCounts[side] += sign;
}
void digitalWrite(uint8_t pin, uint8_t value) {
  for (int i = 0; i < 4; ++i) {
    uint8_t stepPin = i == 3 ? Config::X2_STEP_PIN : Config::STEP_PINS[i];
    uint8_t dirPin = i == 3 ? Config::X2_DIR_PIN : Config::DIR_PINS[i];
    bool invert = i == 3 ? Config::X2_INVERT_DIR : Config::INVERT_DIR[i];
    if (pin == stepPin && value == HIGH && pins[pin] == LOW) {
      int sign = (pins[dirPin] != invert) ? 1 : -1;
      physical[i] += sign;
      ++pulses[i]; axisEvents.push_back(i);
      if (i == 0) xPulseTimes.push_back(clockUs);
      if (i == 0 || i == 3) {
        uint8_t side = i == 0 ? 0 : 1;
        if (!stalled[side]) encoderSteps[side] += sign;
        long targetCount = static_cast<long>(std::floor(encoderSteps[side] / Config::X_STEPS_PER_ENCODER_COUNT));
        while (encoderCounts[side] != targetCount) encoderEdge(side, targetCount > encoderCounts[side] ? 1 : -1);
      }
    }
  }
  pins[pin] = value;
}
int digitalRead(uint8_t pin) {
  if (pin == Config::X2_HOME_PIN && limitModes[3])
    return (Config::HOME_DIRECTION[0] < 0 ? physical[3] <= 0 : physical[3] >= 0) ? LOW : HIGH;
  for (int i = 0; i < 3; ++i)
    if (pin == Config::HOME_PINS[i] && limitModes[i])
      return (Config::HOME_DIRECTION[i] < 0 ? physical[i] <= 0 : physical[i] >= 0) ? LOW : HIGH;
  return pins[pin];
}
static void reset() {
  Motion::stop(); ArcMotion::cancel(); Homing::cancel();
  std::fill(pins, pins + 64, HIGH);
  std::fill(limitModes, limitModes + 4, 0);
  std::fill(physical, physical + 4, 0);
  std::fill(pulses, pulses + 4, 0);
  std::fill(interrupts, interrupts + 64, nullptr);
  for (int i = 0; i < 2; ++i) {
    encoderSteps[i] = encoderCounts[i] = encoderPhase[i] = 0; stalled[i] = false;
    pins[Config::X_ENCODER_A_PINS[i]] = pins[Config::X_ENCODER_B_PINS[i]] = LOW;
  }
  clockUs = 0; xPulseTimes.clear(); axisEvents.clear(); Serial.output.clear();
  Motion::begin();
  EncoderFeedback::begin();
  float zero[3] = {}; CHECK(Motion::setPosition(zero));
}
static GCode parse(const char* text) {
  GCode code; const __FlashStringHelper* error;
  CHECK(GCodeParser::parse(text, code, error)); return code;
}
static void at(float x, float y, float z) {
  float p[3]; Motion::position(p);
  CHECK(std::fabs(p[0] - x) < 0.013f);
  CHECK(std::fabs(p[1] - y) < 0.013f);
  CHECK(std::fabs(p[2] - z) < 0.003f);
}
static void finishMotion() {
  unsigned long ticks = 0;
  while ((Motion::busy() || ArcMotion::busy()) && ticks++ < 2000000) {
    clockUs += 100; Motion::update(); ArcMotion::update(); EncoderFeedback::update();
  }
  CHECK(ticks < 2000000); CHECK(!ArcMotion::failed());
  CHECK(EncoderFeedback::fault() == EncoderFeedback::NONE);
}
static void finishHome() {
  unsigned long ticks = 0;
  while (Homing::busy() && ticks++ < 2000000) {
    clockUs += 100; Homing::update(); Motion::update(); EncoderFeedback::update();
  }
  CHECK(ticks < 2000000);
}
static void finishExecutor() {
  unsigned long ticks = 0;
  while (Executor::busy() && ticks++ < 2000000) { clockUs += 100; Executor::update(); }
  CHECK(ticks < 2000000);
}

static void parserAndProfile() {
  auto code = parse("g2x10y20i5j0f120 ; arc");
  CHECK(code.g == 2 && code.i == 5 && code.has(GCode::J));
  code = parse("G3 K2 R-5"); CHECK(code.has(GCode::K | GCode::R) && code.r == -5);
  const char* bad[] = {"G2 I1 I2", "G1 X1 X2", "G90 G21", "G1 Xnan", "G2 R", "G2 (bad"};
  for (auto text : bad) { const __FlashStringHelper* error; CHECK(!GCodeParser::parse(text, code, error)); }
  MotionProfile p;
  CHECK(p.configure(100, 8000, 1200));
  CHECK(p.interval(0) > p.interval(100));
  CHECK(p.interval(100) > p.interval(4000));
  for (unsigned long i = 0; i < 4000; ++i) {
    CHECK(p.interval(i) == p.interval(7999 - i));
    CHECK(p.interval(i) >= p.interval(i + 1));
    CHECK(p.interval(i) >= Config::MIN_STEP_INTERVAL_US);
  }
  CHECK(p.configure(0.1f, 8, 1200)); CHECK(p.interval(0) > p.interval(3));
  CHECK(p.configure(0.0125f, 1, 1200)); CHECK(p.interval(0) > 1000);
  CHECK(!p.configure(1, 80, 0));
  CHECK(!p.configure(1, 80, 0.00000001f));
}

static void linearAndArcs() {
  reset(); float target[3] = {10, 5, -1};
  CHECK(Motion::start(target, 600)); finishMotion(); at(10, 5, -1);
  CHECK(pulses[0] == 800 && pulses[1] == 400 && pulses[2] == 400);
  CHECK(pulses[3] == pulses[0]);
  CHECK(xPulseTimes[1] - xPulseTimes[0] > xPulseTimes[400] - xPulseTimes[399]);
  CHECK(xPulseTimes[799] - xPulseTimes[798] > xPulseTimes[400] - xPulseTimes[399]);
  reset(); clockUs = UINT32_MAX - 10000; target[0] = 1; target[1] = target[2] = 0;
  CHECK(Motion::start(target, 120)); finishMotion(); at(1, 0, 0); // micros wrap.

  reset(); target[0] = target[1] = 10; target[2] = 2;
  CHECK(ArcMotion::start(parse("G3 I0 J10"), target, 1, 300, 17));
  finishMotion(); at(10, 10, 2); CHECK(pulses[0] == 800 && pulses[1] == 800);
  reset(); target[2] = 0;
  CHECK(ArcMotion::start(parse("G2 I10 J0"), target, 1, 300, 17));
  finishMotion(); at(10, 10, 0); CHECK(pulses[0] == 800 && pulses[1] == 800);
  reset(); CHECK(ArcMotion::start(parse("G3 R10"), target, 1, 300, 17));
  finishMotion(); CHECK(pulses[0] == 800 && pulses[1] == 800);
  reset(); CHECK(ArcMotion::start(parse("G3 R-10"), target, 1, 300, 17));
  finishMotion(); at(10, 10, 0); CHECK(pulses[0] > 1600 && pulses[1] > 1600);
  reset(); target[0] = target[1] = target[2] = 0;
  CHECK(ArcMotion::start(parse("G2 I5"), target, 1, 300, 17));
  finishMotion(); at(0, 0, 0); CHECK(pulses[0] > 1500 && pulses[1] > 1500);
  reset(); target[0] = 10; target[1] = 2; target[2] = 10;
  CHECK(ArcMotion::start(parse("G3 I10 K0"), target, 1, 300, 18));
  finishMotion(); at(10, 2, 10);
  reset(); target[0] = 2; target[1] = 10; target[2] = 10;
  CHECK(ArcMotion::start(parse("G3 J0 K10"), target, 1, 300, 19));
  finishMotion(); at(2, 10, 10);

  reset(); target[0] = target[1] = 10; target[2] = 0;
  const char* invalid[] = {"G2", "G2 R1", "G2 I0 J0", "G2 I1", "G2 R10 I10", "G2 K10"};
  for (auto text : invalid) CHECK(!ArcMotion::start(parse(text), target, 1, 300, 17));
  CHECK(pulses[0] == 0 && pulses[1] == 0);
  target[0] = target[1] = 0;
  CHECK(!ArcMotion::start(parse("G2 R10"), target, 1, 300, 17));
  CHECK(!ArcMotion::start(parse("G2 I50000"), target, 1, 300, 17)); // Segment cap.
}

static void homing() {
  reset();
  for (int i = 0; i < 3; ++i) {
    limitModes[i] = 1; physical[i] = -Config::HOME_DIRECTION[i] * 8 * Config::STEPS_PER_MM[i];
  }
  limitModes[3] = 1; physical[3] = physical[0];
  CHECK(Homing::start(7)); finishHome(); CHECK(!Homing::failed()); at(3, 3, -3);
  CHECK(axisEvents.front() == 2);
  CHECK(physical[0] == 240 && physical[1] == 240 && physical[2] == -1200);
  reset(); limitModes[0] = 1; physical[0] = -80; // Initially active switch.
  limitModes[3] = 1; physical[3] = -80;
  CHECK(Homing::start(1)); finishHome(); CHECK(!Homing::failed()); at(3, 0, 0);
  reset(); CHECK(Homing::start(1)); finishHome(); CHECK(Homing::failed()); // Never triggers.
  reset(); pins[Config::HOME_PINS[0]] = LOW;
  CHECK(Homing::start(1)); finishHome(); CHECK(Homing::failed()); // Never releases.
  reset(); pins[Config::HOME_PINS[1]] = LOW;
  CHECK(!Homing::start(1)); // Unselected active limit is never ignored.
  reset(); limitModes[0] = 1; physical[0] = 800;
  CHECK(Homing::start(1)); pins[Config::ESTOP_PIN] = LOW;
  Homing::update(); CHECK(Homing::failed()); CHECK(!Motion::busy());
}

static void executor() {
  reset(); Executor::begin();
  limitModes[0] = 1; physical[0] = -80;
  limitModes[3] = 1; physical[3] = -80;
  Executor::update(); // An idle home switch must not prevent starting G28.
  Executor::execute(parse("G28 X0")); CHECK(Executor::busy()); finishExecutor();
  CHECK(Serial.output == "ok\n"); at(3, 0, 0);
  limitModes[0] = limitModes[3] = 0;
  Executor::execute(parse("G92 X0 Y0 Z0"));
  Executor::execute(parse("G20"));
  Executor::execute(parse("G3 X1 Y1 I0 J1 F10"));
  CHECK(Executor::busy()); finishExecutor(); at(25.4f, 25.4f, 0);
  Executor::execute(parse("G21")); Executor::execute(parse("G91"));
  Executor::execute(parse("X-10 Y10 I-10 J0 F120")); // Modal CCW arc.
  finishExecutor(); at(15.4f, 35.4f, 0);
  Executor::execute(parse("F60")); CHECK(!Executor::busy());
  Serial.output.clear();
  Executor::execute(parse("G28 X1")); CHECK(Serial.output.find("error:") == 0);
  Serial.output.clear();
  Executor::execute(parse("G3 I-5")); CHECK(Executor::busy());
  for (int i = 0; i < 1000; ++i) { clockUs += 100; Executor::update(); }
  Executor::emergencyStop(); CHECK(!Executor::busy() && !ArcMotion::busy() && !Motion::busy());
  long stopped = pulses[0] + pulses[1];
  for (int i = 0; i < 1000; ++i) { clockUs += 100; Executor::update(); }
  CHECK(stopped == pulses[0] + pulses[1]);
  Executor::execute(parse("G1 X1")); CHECK(Serial.output.find("alarm latched") != std::string::npos);
}

static void dualXAndEncoders() {
  reset();
  // One axle turn needs 4 motor revolutions and yields 800 decoded edges.
  float target[3] = {Config::X_AXLE_TRAVEL_MM, 0, 0};
  CHECK(Motion::start(target, 1200)); finishMotion();
  EncoderFeedback::Status status; EncoderFeedback::status(status);
  CHECK(pulses[0] == Config::X_STEPS_PER_AXLE_REV && pulses[3] == pulses[0]);
  CHECK(status.counts[0] == 800 && status.counts[1] == 800);
  CHECK(std::fabs(status.errorCounts[0]) < 0.01f && std::fabs(status.errorCounts[1]) < 0.01f);
  float zero[3] = {}; CHECK(Motion::setPosition(zero));
  EncoderFeedback::status(status); CHECK(status.counts[0] == 800); // G92 cannot rebase feedback.

  reset();
  for (int i = 0; i < 800; ++i) encoderEdge(0, 1);
  for (int i = 0; i < 804; ++i) encoderEdge(1, -1);
  EncoderFeedback::status(status); CHECK(status.counts[0] == 800 && status.counts[1] == -804);
  CHECK(status.invalid[0] == 0 && status.invalid[1] == 0);

  reset();
  for (int i = 0; i < 9; ++i) encoderEdge(0, 1);
  clockUs += 1000; EncoderFeedback::update(); CHECK(EncoderFeedback::fault() == EncoderFeedback::NONE);
  for (int i = 0; i < 9; ++i) encoderEdge(0, -1);
  clockUs += 1000; EncoderFeedback::update();
  clockUs += 30000; EncoderFeedback::update(); CHECK(EncoderFeedback::fault() == EncoderFeedback::NONE);

  reset();
  for (int i = 0; i < 5; ++i) { encoderEdge(0, 1); encoderEdge(1, -1); }
  clockUs += 1000; EncoderFeedback::update();
  clockUs += 30000; EncoderFeedback::update(); CHECK(EncoderFeedback::fault() == EncoderFeedback::SKEW);

  reset();
  for (int i = 0; i < 4; ++i) {
    pins[Config::X_ENCODER_A_PINS[0]] ^= 1;
    pins[Config::X_ENCODER_B_PINS[0]] ^= 1;
    interrupts[Config::X_ENCODER_A_PINS[0]](); // Impossible two-bit jump.
  }
  clockUs += 1000; EncoderFeedback::update();
  CHECK(EncoderFeedback::fault() == EncoderFeedback::INVALID_TRANSITION);

  reset(); limitModes[0] = limitModes[3] = 1;
  physical[0] = 800; physical[3] = 560; // X2 reaches home 3 mm sooner.
  CHECK(Homing::start(1)); finishHome(); CHECK(!Homing::failed());
  CHECK(physical[0] == 240 && physical[3] == 240);
  CHECK(pulses[0] - pulses[3] == 240);
  CHECK(EncoderFeedback::fault() == EncoderFeedback::NONE);
  CHECK(XGantry::homeMask() == 3); at(3, 0, 0);

  reset(); limitModes[0] = limitModes[3] = 1;
  physical[0] = 800; physical[3] = 80; // More than configured squaring allowance.
  CHECK(Homing::start(1)); finishHome(); CHECK(Homing::failed()); CHECK(!Motion::busy());
  CHECK(XGantry::homeMask() == 3);
  reset(); pins[Config::X_HARD_LIMIT_PINS[0]] = LOW;
  CHECK(!Homing::start(1)); // Hard limits cannot be ignored by homing.
  reset(); limitModes[0] = 1; physical[0] = 80; // Only X1's home switch works.
  CHECK(Homing::start(1)); finishHome(); CHECK(Homing::failed());
  CHECK(pulses[3] < Config::HOME_MAX_TRAVEL_MM[0] * Config::STEPS_PER_MM[0]);
}

static void feedbackShutdown() {
  reset(); Executor::begin();
  Executor::execute(parse("M3 S12000")); CHECK(pins[Config::SPINDLE_PWM_PIN] == 255);
  stalled[1] = true; // X2 receives steps but its encoder stops.
  Executor::execute(parse("G1 X20 F600")); finishExecutor();
  CHECK(EncoderFeedback::fault() == EncoderFeedback::X2_FOLLOWING);
  CHECK(!Motion::busy()); CHECK(pins[Config::SPINDLE_PWM_PIN] == 0);
  CHECK(pins[Config::ENABLE_PINS[0]] != Config::ENABLE_ACTIVE_LEVEL);
  CHECK(pins[Config::X2_ENABLE_PIN] != Config::ENABLE_ACTIVE_LEVEL);
  CHECK(Serial.output.find("X encoder feedback fault") != std::string::npos);
  long stopped = pulses[0] + pulses[3];
  Executor::execute(parse("G92 X0")); Executor::execute(parse("G28 X0"));
  for (int i = 0; i < 1000; ++i) { clockUs += 100; Executor::update(); }
  CHECK(stopped == pulses[0] + pulses[3]);
}

static void homeHardLimitShutdown() {
  reset(); Executor::begin();
  limitModes[0] = limitModes[3] = 1; physical[0] = physical[3] = 800;
  Executor::execute(parse("G28 X0")); CHECK(Executor::busy());
  for (int i = 0; i < 1000; ++i) { clockUs += 100; Executor::update(); }
  pins[Config::X_HARD_LIMIT_PINS[1]] = LOW;
  long stopped = pulses[0] + pulses[3]; Executor::update();
  CHECK(!Executor::busy() && !Motion::busy() && !Homing::busy());
  CHECK(pins[Config::X2_ENABLE_PIN] != Config::ENABLE_ACTIVE_LEVEL);
  CHECK(stopped == pulses[0] + pulses[3]);
}

static void unconfigured() {
  reset(); Executor::begin(); CHECK(!Config::X_CALIBRATED);
  float target[3] = {1, 0, 0}; CHECK(!Motion::start(target, 120));
  CHECK(!Homing::start(1)); CHECK(!ArcMotion::start(parse("G3 I1"), target, 1, 120, 17));
  Executor::execute(parse("G1 X1")); CHECK(!Executor::busy());
  Executor::execute(parse("G28 X0")); CHECK(!Executor::busy());
  CHECK(Serial.output.find("configure X transmission") != std::string::npos);
  Executor::execute(parse("G1 Y1 F120")); finishExecutor();
  CHECK(pulses[0] == 0 && pulses[3] == 0 && pulses[1] == 80);
}

int main(int argc, char** argv) {
  if (argc > 1 && std::string(argv[1]) == "unconfigured") unconfigured();
  else if (argc > 1 && std::string(argv[1]) == "feedback") feedbackShutdown();
  else if (argc > 1 && std::string(argv[1]) == "hardlimit") homeHardLimitShutdown();
  else { parserAndProfile(); linearAndArcs(); homing(); dualXAndEncoders(); executor(); }
  if (failures) { std::cerr << failures << " checks failed\n"; return EXIT_FAILURE; }
  std::cout << "All checks passed: " << (argc > 1 ? argv[1] : "motion, arcs, dual X, quadrature and homing") << ".\n";
}
