#include "App.h"
#include "Debug.h"
#include "EngineModel.h"

void App::setup() {
  // NOTE: boot banner - reports which variant is flashed, since nothing else
  // does so at runtime. Compiled out with the rest of the logging when
  // DEBUG_LEVEL is 0. Keep the CSV header last so the Serial Plotter picks it
  // up as the column labels.
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("=== Piston Interactive :: piston-dynamics ===");
  DEBUG_PRINT("Variant: ");
  DEBUG_PRINTLN(config::pistonSizeName(config::DISPLAY_TYPE));
  DEBUG_PRINTLN("Pos(mm),Throw(mm),RPM,Torque,HP");

  if (!sensor.init()) {
    DEBUG_PRINTLN("Sensor init failed");
    state = AppState::ERROR_RECOVERY;
    return;
  }
  if (!led.init()) {
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
  uint32_t now = millis();
  // NOTE: non-blocking delay logic here to ensure sensor runs smoothly
  // according to the timing budget variable we set
  if (now - last_sensor_read_ >= config::SENSOR_POLL_MS) {
    last_sensor_read_ = now;
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
      last_pos_ = pos;
      tracker.update(pos);
    }
    tracker.decayRPM();

    // compute outputs
    EngineReadout r =
        EngineModel::compute(tracker.getCrankshaftThrowMM(), tracker.getRPMs());
    led.show(r.torque, r.hp);

    // CSV for Serial Plotter
    DEBUG_PRINT(last_pos_, 2);
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

void App::recover() {
  if (!health.allowRecovery())
    return; // wait between attempts
  DEBUG_PRINTLN("Attempting recovery...");

  // NOTE: re-initialize the existing sensor object; we deleted the copy
  // assignment, so assigning a replacement over this object won't compile
  if (!sensor.init()) {
    DEBUG_PRINTLN("Recovery failed");
    return;
  }
  health.markHealthy();

  // NOTE: continuous ranging just restarted so no measurement can be ready yet,
  // seed the tracker from a known state and let run() pick up the first real
  // sample via its ready() guard
  tracker.reset(0.0f);
  DEBUG_PRINTLN("Recovery OK");
  state = AppState::IDLE;
}
