#pragma once
#include <Arduino.h>

namespace Homing {
bool start(uint8_t axes);
void update();
void cancel();
bool busy();
bool failed();
uint8_t ignoredLimits();
}
