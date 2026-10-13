#pragma once
#include <Arduino.h>

namespace XGantry {
void begin();
void enable(bool enabled);
void direction(int8_t sign);
void pulseHigh(int8_t sign);
void pulseLow();
bool homeSwitch(uint8_t side);
bool hardLimit();
void setHomeMask(uint8_t mask); // Bit 0=X1, bit 1=X2. Normal operation uses 3.
uint8_t homeMask();
void commandedSteps(uint32_t result[2]); // Physical pulse totals, unaffected by G92.
}
