#pragma once
/**
 * EngineModel.h
 *
 * A stateless computation helper that calculates and provides
 * simplified engine parameters for torque and horsepower
 */

#include <Arduino.h>

#include "Config.h"

struct EngineReadout {
  float torque = 0;
  float hp = 0;
};

class EngineModel {
public:
  /**
   * @brief Class constructor
   */
  EngineModel() = default;

  /**
   * @brief Performs the actual calculations for engine parameters
   *
   * @return The result of the calculations as an EngineReadout struct
   */
  EngineReadout compute(float throwMM, float rpm) {
    EngineReadout r;
    if (rpm <= 0 || throwMM < config::MIN_THROW_FOR_MOTION_MM) {
      r.torque = 0;
      r.hp = 0;
      return r;
    }

    // simplified pressure estimate ∝ RPM^2
    float pressure = (rpm * rpm) / 10000.0f * config::PRESSURE_MULTIPLIER;
    float torque = throwMM * config::PISTON_AREA_CM2 * pressure *
                   config::TORQUE_SCALE_FACTOR;
    torque = config::clampf(torque, 0.0f, config::MAX_DISPLAY_TORQUE);
    float hp = (torque * rpm) / 1000.0f;
    hp = config::clampf(hp, 0.0f, config::MAX_DISPLAY_HP);

    r.torque = torque;
    r.hp = hp;
    return r;
  }
};
