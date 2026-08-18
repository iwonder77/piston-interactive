#pragma once
/**
 * App.h
 *
 * Main project controller, coordinates all classes
 * - project state machine runs in loopOnce()
 */

#include <Arduino.h>

#include "Health.h"
#include "LedOutput.h"
#include "MotionTracker.h"
#include "ToFSensor.h"

// ======= HEALTH / Finite-State Machine (FSM) ========
enum class AppState : uint8_t { INIT, IDLE, TRACKING, ERROR_RECOVERY };

class App {
public:
  App() = default;
  App(const App &) = delete;
  App &operator=(const App &) = delete;

  /**
   * @brief handles sensor and pwm setup
   */
  void setup();

  /**
   * @brief finite-state machine logic
   * ensures app is in healthy state before running
   */
  void loopOnce(); // runs in main loop (FINITE STATE MACHINE)

private:
  /**
   * @brief main method for coordination of all other classes
   * ensures sensor data is valid/filtered, updates motion tracker with these
   * readings, and computes engine parameters
   */
  void run();

  /**
   * @brief attempts app recovery by re-initializing sensor in place
   */
  void recover();

  uint32_t last_sensor_read_ = 0;
  float last_pos_ = 0.0f;

  ToFSensor sensor;
  MotionTracker tracker;
  LedOutput led;
  Health health;
  AppState state = AppState::INIT;
};
