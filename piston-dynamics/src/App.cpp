#include "App.h"
#include "Debug.h"

/**
 * @brief handles sensor and pwm setup
 */
void App::setup() {
  DEBUG_PRINTLN("Pos(mm),Throw(mm),RPM,Torque,HP");

  if (!sensor.configure()) {
    DEBUG_PRINTLN("Sensor init failed");
    state = AppState::ERROR_RECOVERY;
    return;
  }
  if (!led.beginPWM()) {
    DEBUG_PRINTLN("LED PWM init failed");
    state = AppState::ERROR_RECOVERY;
    return;
  }

  // prime the motion tracker with a first reading if available
  float pos;
  if (sensor.read(pos)) {
    tracker.reset(pos);
  } else {
    tracker.reset(0);
  }
  state = AppState::IDLE;
}

/**
 * @brief finite-state machine logic
 * ensures app is in healthy state before running
 */
void App::loopOnce() {
  switch (state) {
  case AppState::IDLE:
  case AppState::TRACKING:
    run();
    break;
  case AppState::ERROR_RECOVERY:
    recover();
    break;
  default:
    break;
  }
}

/**
 * @brief main method for coordination of all other classes
 * ensures sensor data is valid/filtered, updates motion tracker with these
 * readings, and computes engine parameters
 */
void App::run() {
  // NOTE: non-blocking delay logic here to ensure sensor runs smoothly
  // according to the timing budget variable we set
  if (millis() - lastSensorRead >= config::TIMING_BUDGET_US / 1000) {
    lastSensorRead = millis();
    // "scratchpad" for fresh position reading (is modified on every read)
    float pos = 0.0f;
    bool ok = false;

    // read sensor only if data is ready
    if (sensor.ready()) {
      ok = sensor.read(pos);
      // notify watchdog of reading success or failure
      if (!ok) {
        health.onBadRead();
      } else {
        health.onGoodRead();
      }
    }

    // bail immediately into recovery state if health check fails
    if (!health.healthy()) {
      state = AppState::ERROR_RECOVERY;
      return;
    }

    // only update motion tracker if we successfully read valid sensor data
    if (ok) {
      tracker.update(pos);
    }
    tracker.decayRPM();

    // compute outputs
    EngineReadout r =
        engine.compute(tracker.getCrankshaftThrowMM(), tracker.getRPMs());
    led.show(r.torque, r.hp);

    // CSV for Serial Plotter
    DEBUG_PRINT(pos, 2);
    DEBUG_PRINT(',');
    DEBUG_PRINT(tracker.getCrankshaftThrowMM(), 2);
    DEBUG_PRINT(',');
    DEBUG_PRINT(tracker.getRPMs(), 2);
    DEBUG_PRINT(',');
    DEBUG_PRINT(r.torque, 2);
    DEBUG_PRINT(',');
    DEBUG_PRINTLN(r.hp, 2);

    state = tracker.isMoving() ? AppState::TRACKING : AppState::IDLE;
  }
}

/**
 * @brief attempts app recover by re-initializing sensor
 */
void App::recover() {
  if (!health.allowRecovery())
    return; // wait between attempts
  DEBUG_PRINTLN("Attempting recovery...");

  // Re-init sensor
  ToFSensor newSensor;
  if (newSensor.configure()) {
    sensor = newSensor;
    health.markHealthy();
    // reseed tracker
    float pos;
    if (sensor.read(pos))
      tracker.reset(pos);
    else
      tracker.reset(0);
    DEBUG_PRINTLN("Recovery OK");
    state = AppState::IDLE;
  } else {
    DEBUG_PRINTLN("Recovery failed");
  }
}
