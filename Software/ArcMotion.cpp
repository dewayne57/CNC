#include "ArcMotion.h"
#include "Config.h"
#include "Motion.h"
#include <math.h>

namespace {
constexpr float FULL_TURN = 6.28318530718f;
float origin[3], endPoint[3], centerU, centerV, radius, startAngle, sweep, feedRate;
uint8_t u, v, w;
uint16_t segments, nextSegment;
bool active = false, fault = false;

float directedSweep(float from, float to, bool clockwise) {
  float angle = to - from;
  if (clockwise) { if (angle >= 0) angle -= FULL_TURN; }
  else if (angle <= 0) angle += FULL_TURN;
  return angle;
}
}

bool ArcMotion::start(const GCode& code, const float target[3], float scale,
                      float feed, uint8_t plane) {
  if (active || Motion::busy() || !isfinite(feed) || feed <= 0 ||
      feed > Config::MAX_FEED_MM_MIN) return false;
  if (!Config::X_CALIBRATED) return false;
  if (plane == 17) { u = 0; v = 1; w = 2; }
  else if (plane == 18) { u = 2; v = 0; w = 1; }
  else if (plane == 19) { u = 1; v = 2; w = 0; }
  else return false;
  const uint16_t offsetWords[3] = {GCode::I, GCode::J, GCode::K};
  if (code.has(offsetWords[w])) return false;
  bool offsets = code.has(offsetWords[u] | offsetWords[v]);
  if (code.has(GCode::R) == offsets) return false; // Exactly one format.
  Motion::position(origin);
  for (uint8_t i = 0; i < 3; ++i) {
    if (!isfinite(target[i]) || fabsf(target[i]) > Config::MAX_COORDINATE_MM) return false;
    endPoint[i] = target[i];
  }
  bool clockwise = code.g == 2;
  float du = target[u] - origin[u], dv = target[v] - origin[v];
  if (code.has(GCode::R)) {
    radius = fabsf(code.r * scale);
    float chord = hypotf(du, dv);
    if (!isfinite(radius) || radius <= 0 || chord <= 0 || chord > 2 * radius) return false;
    float height = sqrtf(fmaxf(0, radius * radius - chord * chord * 0.25f));
    if (clockwise != (code.r < 0)) height = -height;
    centerU = origin[u] + du * 0.5f - dv * height / chord;
    centerV = origin[v] + dv * 0.5f + du * height / chord;
  } else {
    const float offsetsMm[3] = {code.i * scale, code.j * scale, code.k * scale};
    centerU = origin[u] + offsetsMm[u];
    centerV = origin[v] + offsetsMm[v];
    radius = hypotf(offsetsMm[u], offsetsMm[v]);
    float endRadius = hypotf(target[u] - centerU, target[v] - centerV);
    if (!isfinite(radius) || radius <= 0 ||
        fabsf(endRadius - radius) > Config::ARC_RADIUS_TOLERANCE_MM) return false;
  }
  // Bound the entire circle before moving, including intermediate coordinates.
  if (!isfinite(centerU) || !isfinite(centerV) ||
      fabsf(centerU) + radius > Config::MAX_COORDINATE_MM ||
      fabsf(centerV) + radius > Config::MAX_COORDINATE_MM) return false;
  startAngle = atan2f(origin[v] - centerV, origin[u] - centerU);
  sweep = directedSweep(startAngle, atan2f(target[v] - centerV, target[u] - centerU), clockwise);
  if (du == 0 && dv == 0) sweep = clockwise ? -FULL_TURN : FULL_TURN;
  // Stable sagitta formula even when the tolerance/radius ratio is small.
  float maxAngle = fminf(1.57079632679f,
      4 * asinf(sqrtf(fminf(0.5f, Config::ARC_CHORD_ERROR_MM / (2 * radius)))));
  float length = hypotf(radius * sweep, target[w] - origin[w]);
  float count = ceilf(fmaxf(fabsf(sweep) / maxAngle, length / Config::ARC_MAX_SEGMENT_MM));
  if (!isfinite(count) || count < 1 || count > Config::ARC_MAX_SEGMENTS) return false;
  segments = static_cast<uint16_t>(count);
  nextSegment = 1;
  feedRate = feed;
  fault = false;
  active = true;
  return true;
}

void ArcMotion::update() {
  if (!active || Motion::busy()) return;
  if (nextSegment > segments) { active = false; return; }
  float target[3];
  if (nextSegment == segments) {
    for (uint8_t i = 0; i < 3; ++i) target[i] = endPoint[i];
  } else {
    float fraction = static_cast<float>(nextSegment) / segments;
    float angle = startAngle + sweep * fraction;
    target[u] = centerU + radius * cosf(angle);
    target[v] = centerV + radius * sinf(angle);
    target[w] = origin[w] + (endPoint[w] - origin[w]) * fraction;
  }
  if (!Motion::start(target, feedRate)) { active = false; fault = true; return; }
  ++nextSegment;
}

void ArcMotion::cancel() { active = fault = false; }
bool ArcMotion::busy() { return active; }
bool ArcMotion::failed() { return fault; }
