#pragma once
/**
 * Health.h
 *
 * A lightweight health monitoring/watchdog class
 */

#include <Arduino.h>

#include "Config.h"

// lighteweight WATCHDOG
class Health {
public:
  void onBadRead() {
    if (errCount < 255)
      ++errCount;
    if (errCount >= config::MAX_SENSOR_ERRORS)
      unhealthy = true;
  }
  void onGoodRead() { errCount = 0; }

  bool healthy() const { return !unhealthy; }
  void markHealthy() {
    unhealthy = false;
    errCount = 0;
  }

  bool allowRecovery() {
    uint32_t now = millis();
    if (now - lastRecoveryTime >= config::ERROR_RECOVERY_PERIOD) {
      lastRecoveryTime = now;
      return true;
    }
    return false;
  }

private:
  uint8_t errCount = 0;
  bool unhealthy = false;
  uint32_t lastRecoveryTime = 0;
};
