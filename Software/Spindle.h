#pragma once
#include <Arduino.h>
namespace Spindle {
void begin();
void run(float rpm, bool clockwise);
void stop();
}
