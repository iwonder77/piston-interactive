#pragma once
/**
 * Health.h
 *
 * A lightweight health monitoring/watchdog class
 */

#include <Arduino.h>
#include <cstdint>

#include "Config.h"

class Health {
public:
  /**
   * @brief Records a failed sensor read.
   *
   * Saturates the error counter so it cannot wrap, and flags the system
   * unhealthy once MAX_SENSOR_ERRORS consecutive failures have been seen.
   */
  void onBadRead() {
    if (error_count_ < UINT8_MAX)
      ++error_count_;
    if (error_count_ >= config::MAX_SENSOR_ERRORS)
      unhealthy_ = true;
  }
  /**
   * @brief Records a successful read, clearing the consecutive-error count.
   *
   * Only consecutive failures matter, so one good read resets the tally.
   */
  void onGoodRead() { error_count_ = 0; }

  /**
   * @brief Whether the sensor is currently considered trustworthy.
   *
   * @return false once MAX_SENSOR_ERRORS consecutive bad reads have occurred,
   * and stays false until markHealthy() clears it
   */
  bool healthy() const { return !unhealthy_; }
  /**
   * @brief Clears the unhealthy flag and the error count.
   *
   * Call after a recovery attempt succeeds - nothing else clears the flag.
   */
  void markHealthy() {
    unhealthy_ = false;
    error_count_ = 0;
  }

  /**
   * @brief Rate limits recovery attempts to one per ERROR_RECOVERY_PERIOD.
   *
   * NOTE: not a pure query - returning true records the attempt, so two calls
   * inside the same window will not both succeed. Call it once per attempt.
   *
   * @return true if enough time has passed to retry
   */
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
