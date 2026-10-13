#pragma once
#include <Arduino.h>

namespace GantryHoming {
bool start(uint8_t ignoredLogicalLimits);
void update();
void cancel();
bool busy();
bool failed();
}
