#include "SerialProtocol.h"
#include "Config.h"
#include "Executor.h"
#include "GCodeParser.h"

namespace {
char line[Config::LINE_CAPACITY];
uint8_t length = 0;
bool discard = false;
bool previousCR = false;
}

void SerialProtocol::begin() {
  Serial.println(F("Mega2560 G-code executor ready; send one line, wait for ok"));
}

void SerialProtocol::update() {
  // Bound parsing work so incoming bytes cannot indefinitely starve stepping.
  for (uint8_t count = 0; count < 16 && Serial.available(); ++count) {
    char c = static_cast<char>(Serial.read());
    // These bytes are real-time controls, even inside comments or long lines.
    if (c == '!') {
      Executor::emergencyStop(); length = 0; discard = true; continue;
    }
    if (c == '?') { Executor::reportPosition(); continue; }
    if (c == '\r' || c == '\n') {
      if (c == '\n' && previousCR) { previousCR = false; continue; }
      previousCR = c == '\r';
      if (!discard && length) {
        line[length] = '\0';
        GCode code;
        const __FlashStringHelper* error;
        if (GCodeParser::parse(line, code, error)) Executor::execute(code);
        else { Serial.print(F("error: ")); Serial.println(error); }
      }
      length = 0; discard = false;
      // One command per loop keeps interlock polling frequent.
      return;
    }
    previousCR = false;
    if (discard) continue;
    if ((static_cast<uint8_t>(c) < 32 && c != '\t') || static_cast<uint8_t>(c) > 126) {
      Serial.println(F("error: invalid input byte; line discarded")); discard = true;
    } else if (length >= Config::LINE_CAPACITY - 1) {
      Serial.println(F("error: line too long; line discarded")); discard = true;
    } else line[length++] = c;
  }
}
