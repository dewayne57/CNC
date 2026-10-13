#include "MotionProfile.h"
#include "Config.h"
#include <math.h>

bool MotionProfile::configure(float distanceMm, unsigned long events, float feedMmMin) {
  if (!events || !isfinite(distanceMm) || distanceMm <= 0 ||
      !isfinite(feedMmMin) || feedMmMin <= 0 || feedMmMin > Config::MAX_FEED_MM_MIN ||
      Config::ACCELERATION_MM_S2 <= 0) return false;
  count = events;
  eventDistance = distanceMm / events;
  maxSpeed = fminf(feedMmMin / 60.0f,
                  eventDistance * 1000000.0f / Config::MIN_STEP_INTERVAL_US);
  acceleration = Config::ACCELERATION_MM_S2;
  float firstSpeed = fminf(maxSpeed, sqrtf(2 * acceleration * eventDistance));
  float worstInterval = 2 * eventDistance / firstSpeed * 1000000.0f;
  if (count == 1) {
    float peak = fminf(maxSpeed, sqrtf(acceleration * eventDistance));
    worstInterval = (eventDistance / peak + peak / acceleration) * 1000000.0f;
  }
  return isfinite(worstInterval) && worstInterval <= 2000000000.0f;
}

float MotionProfile::speedAt(unsigned long boundary) const {
  unsigned long rampEvents = boundary < count - boundary ? boundary : count - boundary;
  return fminf(maxSpeed, sqrtf(2 * acceleration * eventDistance * rampEvents));
}

unsigned long MotionProfile::interval(unsigned long event) const {
  float seconds;
  if (count == 1) {
    // A complete accelerate/decelerate cycle can fit within a single step.
    float peak = fminf(maxSpeed, sqrtf(acceleration * eventDistance));
    seconds = eventDistance / peak + peak / acceleration;
  } else {
    seconds = 2 * eventDistance / (speedAt(event) + speedAt(event + 1));
  }
  unsigned long result = static_cast<unsigned long>(ceilf(seconds * 1000000.0f));
  return result < Config::MIN_STEP_INTERVAL_US ? Config::MIN_STEP_INTERVAL_US : result;
}
