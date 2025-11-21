#pragma once
/**
 * Config.h
 *
 * Centralized configuration constants used by the esp32 sketch
 *
 * - make sure to `#include "Config.h"` then use config::XXXXX throughout the
 * projects for no magic numbers
 * - units are encoded in the name (e.g. _MS, _US)
 */

#include <Arduino.h>

namespace config {
// sensor
constexpr uint16_t SENSOR_TIMEOUT_MS = 500; // VL53L1X timeout
constexpr uint8_t ROI_W = 6;                // 4..16 typical
constexpr uint8_t ROI_H = 6;
constexpr uint8_t ROI_CENTER =
    199; // depends on module; 199 is common for M5Stack unit
constexpr uint16_t TIMING_BUDGET_US = 15000; // 15 ms budget
constexpr uint16_t INTER_MEAS_MS = 5;        // continuous period

// valid sensor reading window (mm)
constexpr float MIN_VALID_MM = 20.0f; // 4 cm is lowest reading recommended from
                                      // sensor docs, but let's stick with 2 cm
constexpr float MAX_VALID_MM = 4000.0f; // 4 m

// motion detection / windows
constexpr float MIN_MOTION_DELTA_MM =
    3.0f; // minimum change in position to detect motion (mm)
constexpr uint32_t MOTION_TIMEOUT_MS =
    3000; // reset tracking after this many ms of no motion
constexpr float MIN_REASONABLE_THROW =
    2.0f; // minimum detected throw that makes sense (mm)
constexpr float MAX_REASONABLE_THROW =
    150.0f; // maximum detected throw that makes sense (mm)
constexpr float MIN_THROW_FOR_MOTION_MM =
    2.0f; // minimum throw to consider system "in motion" (mm)
constexpr float MIN_MAX_HYST_MM =
    2.0f; // must exceed previous min/max by this amount (+ or -) to update
constexpr uint16_t PEAK_MIN_MAX_WINDOW_SIZE =
    64; // max ring buffer size for min/max window
constexpr uint16_t PEAK_PERIOD_WINDOW_SIZE =
    6; // max ring buffer size for period count window

// engine model
enum class PistonSize : uint8_t { SMALL, MEDIUM, LARGE };
// NOTE: piston size display type must be different for each installation and
// manually changed here a little tedious to manually change and reflash, maybe
// in the future we could use dip switches for the 3 piston sizes
constexpr PistonSize DISPLAY_TYPE = PistonSize::SMALL;
constexpr float PISTON_AREA_CM2 = 50.0f;
constexpr float MIN_DETECTED_RPM = 2.0f;
constexpr float MAX_DETECTED_RPM = 200.0f;
constexpr float MAX_DISPLAY_TORQUE = 1200.0f;
constexpr float MAX_DISPLAY_HP = 200.0f;
constexpr float PRESSURE_MULTIPLIER = 0.5f; // simplification factor
constexpr float TORQUE_SCALE_FACTOR =
    0.2f; // scale torque for reasonable LED range

// EMA filter constants (0...1) smaller = heavier smoothing
constexpr float EMA_POS_ALPHA = 0.4f; // position smoothing
constexpr float EMA_RPM_ALPHA = 0.3f; // rpm smoothing

// LEDC (PWM)
constexpr int TORQUE_LED_PIN = 18;
constexpr int HP_LED_PIN = 19;
constexpr uint32_t PWM_FREQ_HZ = 5000; // 5kHz PWM frequency
constexpr uint8_t PWM_RES_BITS = 8;    // 8-bit resolution (0...255)
constexpr int PWM_MIN_DUTY =
    20; // allow small readings for torque and hp to pass through
// NOTE: although changing the max duty cycle for torque and horsepower pwm
// signals across displays is not technically accurate, the educational impact
// of seeing changes in LED bar height across displays is higher
constexpr int SMALL_PISTON_MAX_DUTY = 190;
constexpr int MEDIUM_PISTON_MAX_DUTY = 220;
constexpr int LARGE_PISTON_MAX_DUTY = 255;

// error handling
constexpr uint8_t MAX_SENSOR_ERRORS = 5;
constexpr uint32_t ERROR_RECOVERY_PERIOD = 5000;

// util function
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}
} // namespace config
