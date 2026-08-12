#pragma once
/**
 * MotionTracker.14:14:37
 *
 * Tracks piston motion, detects throw measurement, and estimates RPM with
 * zero-crossing algorithm
 */
#include "Config.h"
#include "RingWindow.h"
#include <Arduino.h>

class MotionTracker {
public:
  MotionTracker() = default;

  void update(float pos);
  void reset(float pos);
  void decayRPM();

  bool isMoving() const {
    return moving_ && is_throw_valid_ &&
           crankshaft_throw_ >= config::MIN_THROW_FOR_MOTION_MM;
  }
  float getCrankshaftThrowMM() const { return crankshaft_throw_; }
  float getRPMs() const { return rpm_; }
  uint32_t getLastRPMUpdate() const { return last_rpm_update_ms_; }

private:
  enum class Edge : uint8_t { UNKNOWN, ABOVE, BELOW };

  RingWindow<uint32_t, config::PEAK_PERIOD_WINDOW_SIZE> period_window_;
  RingWindow<float, config::PEAK_MIN_MAX_WINDOW_SIZE> pos_window_;

  bool moving_ = false;
  bool is_throw_valid_ = false;
  float last_pos_ = 0;
  float max_pos_ = 0;
  float min_pos_ = 0;
  float crankshaft_throw_ = 0;
  float rpm_ = 0;
  bool rpm_init_ = false;
  uint32_t last_motion_ms_ = 0;
  uint32_t last_zero_cross_ms_ = 0;
  uint32_t last_rpm_update_ms_ = 0;
  Edge last_edge_ = Edge::UNKNOWN;
};
