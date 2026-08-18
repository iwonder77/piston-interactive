#pragma once
/**
 * Health.h
 *
 * A lightweight health monitoring/watchdog class
 */

#include <Arduino.h>

#include "Config.h"

class Health {
public:
  void onBadRead() {
    if (error_count_ < 255)
      ++error_count_;
    if (error_count_ >= config::MAX_SENSOR_ERRORS)
      unhealthy_ = true;
  }
  void onGoodRead() { error_count_ = 0; }

  bool healthy() const { return !unhealthy_; }
  void markHealthy() {
    unhealthy_ = false;
    error_count_ = 0;
  }

  bool allowRecovery() {
    uint32_t now = millis();
    if (now - last_recovery_time_ >= config::ERROR_RECOVERY_PERIOD) {
      last_recovery_time_ = now;
      return true;
    }
    return false;
  }

private:
  uint8_t error_count_ = 0;
  bool unhealthy_ = false;
  uint32_t last_recovery_time_ = 0;
};
