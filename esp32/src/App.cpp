#include "App.h"
#include "Debug.h"

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

void App::run() {
  // temporary "scratchpad" for fresh position reading
  float pos;
  bool ok = false;

  // read sensor only if data is ready
  if (sensor.ready()) {
    ok = sensor.read(pos);
    // notify watchdog of reading success/failure
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

  if (ok) {
    // now that the position was read successfully, update motion + RPM
    // estimator
    tracker.update(pos);
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
    DEBUG_PRINT(r.rpm, 2);
    DEBUG_PRINT(',');
    DEBUG_PRINT(r.torque, 2);
    DEBUG_PRINT(',');
    DEBUG_PRINTLN(r.hp, 2);

    state = tracker.isMoving() ? AppState::TRACKING : AppState::IDLE;
  }
}

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
