#include <Arduino.h>

#include "Config.h"
#include "EngineModel.h"
#include "Health.h"
#include "LedOutput.h"
#include "MotionTracker.h"
#include "ToFSensor.h"

// ======= HEALTH / Finite-State Machine (FSM) ========
enum class AppState : uint8_t { INIT, IDLE, TRACKING, ERROR_RECOVERY };
// ======================= APP ========================
class App {
public:
  void setup() {
    Serial.begin(115200);
    Serial.println("Pos(mm),Throw(mm),RPM,Torque,HP");

    if (!sensor.configure()) {
      Serial.println("Sensor init failed");
      state = AppState::ERROR_RECOVERY;
      return;
    }
    if (!led.beginPWM()) {
      Serial.println("LED PWM init failed");
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

  // runs in main loop (FINITE STATE MACHINE)
  void loopOnce() {
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

private:
  void run() {
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
      Serial.print(pos, 2);
      Serial.print(',');
      Serial.print(tracker.getCrankshaftThrowMM(), 2);
      Serial.print(',');
      Serial.print(r.rpm, 2);
      Serial.print(',');
      Serial.print(r.torque, 2);
      Serial.print(',');
      Serial.println(r.hp, 2);

      state = tracker.isMoving() ? AppState::TRACKING : AppState::IDLE;
    }
  }

  void recover() {
    if (!health.allowRecovery())
      return; // wait between attempts
    Serial.println("Attempting recovery...");

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
      Serial.println("Recovery OK");
      state = AppState::IDLE;
    } else {
      Serial.println("Recovery failed");
    }
  }

  // Components
  ToFSensor sensor;
  MotionTracker tracker;
  EngineModel engine;
  LedOutput led;
  Health health;
  AppState state = AppState::INIT;
};
// ====================================================
