#pragma once
#include "GCodeParser.h"
namespace Executor {
void begin();
void update();
void execute(const GCode& code);
void emergencyStop();
bool busy();
void reportPosition();
}
