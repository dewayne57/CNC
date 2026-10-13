#pragma once
#include <Arduino.h>

namespace Motion {
void begin();
void update();
bool start(const float target[3], float feedMmMin, uint8_t ignoredLimits = 0);
bool busy();
void stop();
void halt(); // Stop pulses immediately but retain holding torque.
void enable(bool enabled);
void position(float result[3]);
bool setPosition(const float position[3]);
bool interlockActive(uint8_t ignoredLimits = 0);
bool estopActive();
bool limitActive(uint8_t axis);
}
