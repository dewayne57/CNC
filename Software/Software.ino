#include "Config.h"
#include "SerialProtocol.h"
#include "Executor.h"
#include "Display.h"

void setup() {
  Serial.begin(Config::BAUD_RATE);
  Display::begin();
  Executor::begin();
  SerialProtocol::begin();
}

void loop() {
  SerialProtocol::update();
  Executor::update();
  Display::update();
}
