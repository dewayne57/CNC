#pragma once
#include <Arduino.h>

// Set these to the physical machine values (or provide compiler -D overrides).
// Zero deliberately marks an unconfigured X transmission; X motion is blocked.
#ifndef CNC_X_MICROSTEPS
#define CNC_X_MICROSTEPS 0
#endif
#ifndef CNC_X_AXLE_TRAVEL_MM
#define CNC_X_AXLE_TRAVEL_MM 0.0f
#endif

#ifndef CNC_X1_STEP_PIN
#define CNC_X1_STEP_PIN 30
#endif
#ifndef CNC_Y_STEP_PIN
#define CNC_Y_STEP_PIN 33
#endif
#ifndef CNC_Z_STEP_PIN
#define CNC_Z_STEP_PIN 36
#endif
#ifndef CNC_X1_DIR_PIN
#define CNC_X1_DIR_PIN 31
#endif
#ifndef CNC_Y_DIR_PIN
#define CNC_Y_DIR_PIN 34
#endif
#ifndef CNC_Z_DIR_PIN
#define CNC_Z_DIR_PIN 37
#endif
#ifndef CNC_X1_ENABLE_PIN
#define CNC_X1_ENABLE_PIN 32
#endif
#ifndef CNC_Y_ENABLE_PIN
#define CNC_Y_ENABLE_PIN 35
#endif
#ifndef CNC_Z_ENABLE_PIN
#define CNC_Z_ENABLE_PIN 38
#endif
#ifndef CNC_X2_STEP_PIN
#define CNC_X2_STEP_PIN 30
#endif
#ifndef CNC_X2_DIR_PIN
#define CNC_X2_DIR_PIN 31
#endif
#ifndef CNC_X2_ENABLE_PIN
#define CNC_X2_ENABLE_PIN 32
#endif
#ifndef CNC_X1_HOME_PIN
#define CNC_X1_HOME_PIN 2
#endif
#ifndef CNC_X2_HOME_PIN
#define CNC_X2_HOME_PIN 3
#endif
#ifndef CNC_X1_HARD_LIMIT_PIN
#define CNC_X1_HARD_LIMIT_PIN 6
#endif
#ifndef CNC_X2_HARD_LIMIT_PIN
#define CNC_X2_HARD_LIMIT_PIN 7
#endif
#ifndef CNC_Y_HOME_PIN
#define CNC_Y_HOME_PIN 4
#endif
#ifndef CNC_Z_HOME_PIN
#define CNC_Z_HOME_PIN 5
#endif
#ifndef CNC_X_ENCODER_X1_A_PIN
#define CNC_X_ENCODER_X1_A_PIN 22
#endif
#ifndef CNC_X_ENCODER_X2_A_PIN
#define CNC_X_ENCODER_X2_A_PIN 24
#endif
#ifndef CNC_X_ENCODER_X1_B_PIN
#define CNC_X_ENCODER_X1_B_PIN 23
#endif
#ifndef CNC_X_ENCODER_X2_B_PIN
#define CNC_X_ENCODER_X2_B_PIN 25
#endif
#ifndef CNC_SPINDLE_PWM_PIN
#define CNC_SPINDLE_PWM_PIN 50
#endif
#ifndef CNC_SPINDLE_DIR_PIN
#define CNC_SPINDLE_DIR_PIN 51
#endif
#ifndef CNC_ESTOP_PIN
#define CNC_ESTOP_PIN 10
#endif
#ifndef CNC_Y_LIMIT_PIN
#define CNC_Y_LIMIT_PIN 8
#endif
#ifndef CNC_Z_LIMIT_PIN
#define CNC_Z_LIMIT_PIN 9
#endif
#ifndef CNC_LCD_RS_PIN
#define CNC_LCD_RS_PIN 39
#endif
#ifndef CNC_LCD_RW_PIN
#define CNC_LCD_RW_PIN 40
#endif
#ifndef CNC_LCD_ENABLE_PIN
#define CNC_LCD_ENABLE_PIN 41
#endif
#ifndef CNC_LCD_D0_PIN
#define CNC_LCD_D0_PIN 42
#endif
#ifndef CNC_LCD_D1_PIN
#define CNC_LCD_D1_PIN 43
#endif
#ifndef CNC_LCD_D2_PIN
#define CNC_LCD_D2_PIN 44
#endif
#ifndef CNC_LCD_D3_PIN
#define CNC_LCD_D3_PIN 45
#endif
#ifndef CNC_LCD_D4_PIN
#define CNC_LCD_D4_PIN 46
#endif
#ifndef CNC_LCD_D5_PIN
#define CNC_LCD_D5_PIN 47
#endif
#ifndef CNC_LCD_D6_PIN
#define CNC_LCD_D6_PIN 48
#endif
#ifndef CNC_LCD_D7_PIN
#define CNC_LCD_D7_PIN 49
#endif

// Mega 2560 + external DM556 drivers, common-cathode 5 V control wiring.
namespace Config {
constexpr unsigned long BAUD_RATE = 115200;
constexpr uint8_t AXES = 3;
constexpr uint8_t STEP_PINS[AXES] = {CNC_X1_STEP_PIN, CNC_Y_STEP_PIN, CNC_Z_STEP_PIN};
constexpr uint8_t DIR_PINS[AXES] = {CNC_X1_DIR_PIN, CNC_Y_DIR_PIN, CNC_Z_DIR_PIN};
constexpr uint8_t ENABLE_PINS[AXES] = {CNC_X1_ENABLE_PIN, CNC_Y_ENABLE_PIN, CNC_Z_ENABLE_PIN};
constexpr uint8_t ENABLE_ACTIVE_LEVEL = LOW; // Verify with your DM556 variant.
constexpr bool INVERT_DIR[AXES] = {false, false, false};
constexpr uint8_t X2_STEP_PIN = CNC_X2_STEP_PIN, X2_DIR_PIN = CNC_X2_DIR_PIN,
                  X2_ENABLE_PIN = CNC_X2_ENABLE_PIN;
constexpr uint8_t X2_HOME_PIN = CNC_X2_HOME_PIN;
constexpr uint8_t X_HARD_LIMIT_PINS[2] = {CNC_X1_HARD_LIMIT_PIN, CNC_X2_HARD_LIMIT_PIN};
constexpr bool X2_INVERT_DIR = false; // Set for mirrored motor installation.
constexpr float X_GEAR_REDUCTION = 4.0f; // Motor revolutions per axle revolution.
constexpr uint16_t X_MOTOR_FULL_STEPS_PER_REV = 200; // Verify motor step angle.
constexpr uint16_t X_MICROSTEPS = CNC_X_MICROSTEPS; // Both DM556 modules must match.
constexpr float X_AXLE_TRAVEL_MM = CNC_X_AXLE_TRAVEL_MM;
constexpr float X_STEPS_PER_AXLE_REV = static_cast<float>(X_MOTOR_FULL_STEPS_PER_REV) * X_MICROSTEPS * X_GEAR_REDUCTION;
constexpr bool X_CALIBRATED = X_MICROSTEPS > 0 && X_AXLE_TRAVEL_MM > 0;
// Fallback avoids division by zero; it is never used for real X motion.
constexpr float STEPS_PER_MM[AXES] = {
    X_CALIBRATED ? X_STEPS_PER_AXLE_REV / X_AXLE_TRAVEL_MM : 1.0f, 80.0f, 400.0f};
constexpr uint8_t X_ENCODER_A_PINS[2] = {CNC_X_ENCODER_X1_A_PIN, CNC_X_ENCODER_X2_A_PIN};
constexpr uint8_t X_ENCODER_B_PINS[2] = {CNC_X_ENCODER_X1_B_PIN, CNC_X_ENCODER_X2_B_PIN};
constexpr bool X_ENCODER_INVERT[2] = {false, false};
constexpr uint16_t X_ENCODER_SLOTS = 200;
constexpr uint16_t X_ENCODER_COUNTS_PER_REV = X_ENCODER_SLOTS * 4;
constexpr float X_STEPS_PER_ENCODER_COUNT = X_STEPS_PER_AXLE_REV / X_ENCODER_COUNTS_PER_REV;
// Tune these in decoded axle counts, not motor pulses. Each count = axle travel / 800.
constexpr float X_MAX_FOLLOWING_ERROR_COUNTS = 8.0f;
constexpr float X_MAX_SKEW_ERROR_COUNTS = 8.0f;
constexpr unsigned long X_ENCODER_FAULT_DELAY_MS = 25;
constexpr uint32_t X_MAX_INVALID_TRANSITIONS = 4;
constexpr unsigned long X_ENCODER_SAMPLE_US = 1000;
constexpr float X_HOME_MAX_DIFFERENTIAL_MM = 5.0f; // Maximum cumulative correction during X homing.
constexpr uint8_t SPINDLE_PWM_PIN = CNC_SPINDLE_PWM_PIN;
constexpr uint8_t SPINDLE_DIR_PIN = CNC_SPINDLE_DIR_PIN;
constexpr float MAX_SPINDLE_RPM = 12000.0f;
constexpr uint8_t ESTOP_PIN = CNC_ESTOP_PIN; // INPUT_PULLUP; contact to GND asserts stop.
constexpr uint8_t HOME_PINS[AXES] = {CNC_X1_HOME_PIN, CNC_Y_HOME_PIN, CNC_Z_HOME_PIN};
constexpr uint8_t LIMIT_PINS[AXES] = {CNC_X1_HARD_LIMIT_PIN, CNC_Y_LIMIT_PIN, CNC_Z_LIMIT_PIN};
constexpr uint8_t LCD_RS_PIN = CNC_LCD_RS_PIN;
constexpr uint8_t LCD_RW_PIN = CNC_LCD_RW_PIN;
constexpr uint8_t LCD_ENABLE_PIN = CNC_LCD_ENABLE_PIN;
constexpr uint8_t LCD_DATA_PINS[8] = {CNC_LCD_D0_PIN, CNC_LCD_D1_PIN, CNC_LCD_D2_PIN, CNC_LCD_D3_PIN,
                                      CNC_LCD_D4_PIN, CNC_LCD_D5_PIN, CNC_LCD_D6_PIN, CNC_LCD_D7_PIN};
constexpr float DEFAULT_FEED_MM_MIN = 300.0f;
constexpr float RAPID_MM_MIN = 600.0f;
constexpr float MAX_FEED_MM_MIN = 1200.0f;
constexpr float ACCELERATION_MM_S2 = 50.0f; // Path acceleration, exact stop per move/chord.
constexpr float ARC_CHORD_ERROR_MM = 0.02f;
constexpr float ARC_MAX_SEGMENT_MM = 1.0f;
constexpr float ARC_RADIUS_TOLERANCE_MM = 0.05f;
constexpr uint16_t ARC_MAX_SEGMENTS = 10000;
// Homing uses the existing limit inputs. Configure these for the actual machine.
constexpr uint8_t HOME_ORDER[AXES] = {2, 0, 1}; // Z, X, Y, one at a time.
constexpr int8_t HOME_DIRECTION[AXES] = {-1, -1, 1};
constexpr float HOME_MAX_TRAVEL_MM[AXES] = {300.0f, 300.0f, 100.0f};
constexpr float HOME_PULL_OFF_MM = 3.0f;
constexpr float HOME_SEEK_MM_MIN = 300.0f;
constexpr float HOME_LATCH_MM_MIN = 60.0f;
constexpr float HOME_SWITCH_POSITION_MM[AXES] = {0.0f, 0.0f, 0.0f};
constexpr unsigned long HOME_TIMEOUT_MS = 120000; // Per phase, including settle time.
constexpr unsigned long HOME_DEBOUNCE_MS = 10;
constexpr unsigned long MIN_STEP_INTERVAL_US = 250; // Maximum 4000 steps/s.
constexpr unsigned int STEP_PULSE_US = 5;
constexpr unsigned int DIR_SETUP_US = 5;
constexpr unsigned int ENABLE_SETUP_US = 10;
constexpr float MAX_COORDINATE_MM = 100000.0f; // Numeric bound, not travel limits.
constexpr uint8_t LINE_CAPACITY = 128;
}
