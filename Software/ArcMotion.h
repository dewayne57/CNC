#pragma once
#include "GCodeParser.h"

namespace ArcMotion {
// Plane is 17 (XY), 18 (ZX), or 19 (YZ). Offsets are always start-relative.
bool start(const GCode& code, const float target[3], float scale, float feed, uint8_t plane);
void update();
void cancel();
bool busy();
bool failed();
}
