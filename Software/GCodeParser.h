#pragma once
#include <Arduino.h>

struct GCode {
  int16_t g;
  int16_t m;
  float x, y, z, f, s, p, i, j, k, r;
  uint16_t words;
  enum Word : uint16_t {
    X = 1, Y = 2, Z = 4, F = 8, S = 16, P = 32,
    I = 64, J = 128, K = 256, R = 512
  };
  bool has(uint16_t mask) const { return (words & mask) != 0; }
};

namespace GCodeParser {
// Supports one G or M command per line, compact words and two comment styles.
bool parse(const char* line, GCode& code, const __FlashStringHelper*& error);
}
