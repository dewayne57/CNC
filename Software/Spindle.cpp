#include "Spindle.h"
#include "Config.h"

void Spindle::begin() {
  digitalWrite(Config::SPINDLE_PWM_PIN, LOW);
  pinMode(Config::SPINDLE_PWM_PIN, OUTPUT);
  pinMode(Config::SPINDLE_DIR_PIN, OUTPUT);
  stop();
}
void Spindle::run(float rpm, bool clockwise) {
  analogWrite(Config::SPINDLE_PWM_PIN, 0);
  digitalWrite(Config::SPINDLE_DIR_PIN, clockwise ? HIGH : LOW);
  analogWrite(Config::SPINDLE_PWM_PIN,
              static_cast<int>(rpm * 255.0f / Config::MAX_SPINDLE_RPM + 0.5f));
}
void Spindle::stop() { analogWrite(Config::SPINDLE_PWM_PIN, 0); }
