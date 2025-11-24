#pragma once

#include <Arduino.h>

#include "EngineModel.h"
#include "Health.h"
#include "LedOutput.h"
#include "MotionTracker.h"
#include "ToFSensor.h"

// ======= HEALTH / Finite-State Machine (FSM) ========
enum AppState { INIT, IDLE, TRACKING, ERROR_RECOVERY };

class App {
public:
  void setup();
  void loopOnce(); // runs in main loop (FINITE STATE MACHINE)

private:
  void run();
  void recover();

  ToFSensor sensor;
  MotionTracker tracker;
  EngineModel engine;
  LedOutput led;
  Health health;
  AppState state = AppState::INIT;
};
