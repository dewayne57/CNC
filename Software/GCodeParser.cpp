#include "GCodeParser.h"
#include <ctype.h>
#include <math.h>
#include <stdlib.h>

bool GCodeParser::parse(const char* line, GCode& code,
                        const __FlashStringHelper*& error) {
  code = {};
  code.g = code.m = -1;
  error = nullptr;
  const char* cursor = line;
  while (*cursor) {
    if (isspace(static_cast<unsigned char>(*cursor))) { ++cursor; continue; }
    if (*cursor == ';') break;
    if (*cursor == '(') {
      ++cursor;
      while (*cursor && *cursor != ')') {
        if (*cursor == '(') { error = F("nested comment"); return false; }
        ++cursor;
      }
      if (!*cursor) { error = F("unclosed comment"); return false; }
      ++cursor;
      continue;
    }
    char letter = toupper(static_cast<unsigned char>(*cursor++));
    if (letter < 'A' || letter > 'Z') {
      error = F("expected a word letter"); return false;
    }
    while (isspace(static_cast<unsigned char>(*cursor))) ++cursor;
    // Deliberately accept decimal numbers only: no exponent, NaN or infinity.
    const char* start = cursor;
    if (*cursor == '+' || *cursor == '-') ++cursor;
    bool digit = false;
    while (isdigit(static_cast<unsigned char>(*cursor))) { digit = true; ++cursor; }
    if (*cursor == '.') {
      ++cursor;
      while (isdigit(static_cast<unsigned char>(*cursor))) { digit = true; ++cursor; }
    }
    if (!digit) { error = F("missing number"); return false; }
    float value = atof(start);
    if (!isfinite(value)) { error = F("number out of range"); return false; }
    if (letter == 'G' || letter == 'M') {
      if (code.g >= 0 || code.m >= 0) {
        error = F("one G or M command per line"); return false;
      }
      if (value < 0 || value > 999 || value != floorf(value)) {
        error = F("invalid command number"); return false;
      }
      if (letter == 'G') code.g = static_cast<int16_t>(value);
      else code.m = static_cast<int16_t>(value);
      continue;
    }
    uint16_t flag = 0;
    float* destination = nullptr;
    switch (letter) {
      case 'X': flag = GCode::X; destination = &code.x; break;
      case 'Y': flag = GCode::Y; destination = &code.y; break;
      case 'Z': flag = GCode::Z; destination = &code.z; break;
      case 'F': flag = GCode::F; destination = &code.f; break;
      case 'S': flag = GCode::S; destination = &code.s; break;
      case 'P': flag = GCode::P; destination = &code.p; break;
      case 'I': flag = GCode::I; destination = &code.i; break;
      case 'J': flag = GCode::J; destination = &code.j; break;
      case 'K': flag = GCode::K; destination = &code.k; break;
      case 'R': flag = GCode::R; destination = &code.r; break;
      default: error = F("unsupported word"); return false;
    }
    if (code.has(flag)) { error = F("duplicate word"); return false; }
    code.words |= flag;
    *destination = value;
  }
  return true;
}
