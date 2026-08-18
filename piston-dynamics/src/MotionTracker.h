#pragma once
/**
 * MotionTracker.h
 *
 * Tracks piston motion, detects throw measurement, and estimates RPM with
 * zero-crossing algorithm
 */
#include <Arduino.h>

#include "Config.h"
#include "RingWindow.h"

class MotionTracker {
public:
  MotionTracker() = default;

  /**
   * @brief Updates motion tracking with new position reading
   *
   * Detects motion, updates all ring windows, calculates throw and RPM
   *
   * @param pos Filtered sensor reading
   */
  void update(float pos);

  /**
   * @brief Resets the class members, ring window objects, and center crossing
   * state machine
   *
   * @param pos Value to reset members with
   */
  void reset(float pos);

  /**
   * @brief Applies gentle decay to rpm value when, for example, a
   * kid stops spinning the crankshaft
   */
  void decayRPM();

  /**
   * @brief Whether the crankshaft is turning with a usable throw measurement.
   *
   * Requires all three: motion was detected recently, a throw has passed its
   * validity checks, and that throw is large enough to be worth displaying.
   *
   * @return true when the tracker's output can be trusted
   */
  bool isMoving() const {
    return moving_ && is_throw_valid_ &&
           crankshaft_throw_ >= config::MIN_THROW_FOR_MOTION_MM;
  }
  /**
   * @brief Most recent crankshaft throw measurement.
   *
   * @return throw in millimetres, or 0.0f when no valid throw is known
   */
  float getCrankshaftThrowMM() const { return crankshaft_throw_; }
  /**
   * @brief Current RPM estimate.
   *
   * Smoothed and decayed, so it trails the crankshaft rather than tracking it
   * instantaneously.
   *
   * @return revolutions per minute, 0.0f when stopped
   */
  float getRPMs() const { return rpm_; }

private:
  enum class Edge : uint8_t { UNKNOWN, ABOVE, BELOW };

  RingWindow<uint32_t, config::PEAK_PERIOD_WINDOW_SIZE> period_window_;
  RingWindow<float, config::PEAK_MIN_MAX_WINDOW_SIZE> pos_window_;

  bool moving_ = false;
  bool is_throw_valid_ = false;
  // --- all in mm ---
  float last_pos_ = 0;
  float max_pos_ = 0;
  float min_pos_ = 0;
  float crankshaft_throw_ = 0;
  // -----------------
  float rpm_ = 0;
  bool rpm_init_ = false;
  uint32_t last_motion_ms_ = 0;
  uint32_t last_zero_cross_ms_ = 0;
  uint32_t last_rpm_update_ms_ = 0;
  Edge last_edge_ = Edge::UNKNOWN;
};
