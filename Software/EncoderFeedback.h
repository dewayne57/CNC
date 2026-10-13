#pragma once
#include <Arduino.h>

namespace EncoderFeedback {
enum Fault : uint8_t { NONE, X1_FOLLOWING, X2_FOLLOWING, SKEW, INVALID_TRANSITION };
struct Status {
  int32_t counts[2];
  float errorCounts[2]; // Measured minus commanded displacement since reference.
  uint32_t invalid[2];
  Fault fault;
};
void begin();
void update();
Fault fault();
void status(Status& result);
void report();
}
