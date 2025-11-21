#include <Arduino.h>

#include "Config.h"

// ===================== UTILS =====================
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// =================== ENGINE MODEL ===================
struct EngineReadout {
  float rpm = 0;
  float torque = 0;
  float hp = 0;
};

class EngineModel {
public:
  EngineReadout compute(float throwMM, float rpm) {
    EngineReadout r;
    r.rpm = config::clampf(rpm, 0.0f, config::MAX_DETECTED_RPM);
    if (r.rpm <= 0 || throwMM < config::MIN_THROW_FOR_MOTION_MM) {
      r.torque = 0;
      r.hp = 0;
      return r;
    }

    // simplified pressure estimate ∝ RPM^2
    float pressure = (r.rpm * r.rpm) / 10000.0f * config::PRESSURE_MULTIPLIER;
    float torque = throwMM * config::PISTON_AREA_CM2 * pressure *
                   config::TORQUE_SCALE_FACTOR;
    torque = config::clampf(torque, 0.0f, config::MAX_DISPLAY_TORQUE);
    float hp = (torque * r.rpm) / 1000.0f;
    hp = config::clampf(hp, 0.0f, config::MAX_DISPLAY_HP);

    r.torque = torque;
    r.hp = hp;
    return r;
  }
};
// ====================================================
