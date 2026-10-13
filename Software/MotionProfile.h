#pragma once
#include <Arduino.h>

// Distance-domain trapezoid; a short move automatically becomes triangular.
// Each interval uses mean entry/exit speed, including the first/last step.
class MotionProfile {
 public:
  bool configure(float distanceMm, unsigned long events, float feedMmMin);
  unsigned long interval(unsigned long event) const;
 private:
  float eventDistance = 0, maxSpeed = 0, acceleration = 0;
  unsigned long count = 0;
  float speedAt(unsigned long boundary) const;
};
