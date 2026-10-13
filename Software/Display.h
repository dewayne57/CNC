#pragma once
#include <stdint.h>

namespace Display {
enum class Operation : uint8_t {
  IDLE,
  MOVING,
  HOMING,
  DWELLING,
  SPINDLE,
  ALARM
};

void begin();
void update();
void setOperation(Operation operation);
}
