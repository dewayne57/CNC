#include "Config.h"
#include "SerialProtocol.h"
#include "Executor.h"

void setup() {
  Serial.begin(Config::BAUD_RATE);
  Executor::begin();
  SerialProtocol::begin();
}

void loop() {
  SerialProtocol::update();
  Executor::update();
}
