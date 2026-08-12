#include "MotionTracker.h"
#include "Config.h"
#include "Debug.h"

void MotionTracker::update(float pos) {
  uint32_t now = millis();
  float delta_mm = fabsf(pos - last_pos_);

  if (delta_mm > config::MIN_MOTION_DELTA_MM) {
    if (!moving_) {
      moving_ = true;
      DEBUG_PRINTLN("Motion detected — tracking");
    }
    last_motion_ms_ = now;

    // update ring window and hysteretic min/max
    pos_window_.add(pos);
    float w_max = pos_window_.getMax();
    float w_min = pos_window_.getMin();
    // hysteretic min/max updates
    if (fabsf(w_max - max_pos_) > config::MIN_MAX_HYST_MM) {
      max_pos_ = w_max;
    }
    if (fabsf(w_min - min_pos_) > config::MIN_MAX_HYST_MM) {
      min_pos_ = w_min;
    }

    // throw (radius) ~ (max-min)/2
    float raw_throw = (max_pos_ - min_pos_) * 0.5f;
    if (raw_throw < config::MIN_REASONABLE_THROW) {
      crankshaft_throw_ = 0.0f;
      is_throw_valid_ = false;
    } else if (raw_throw > config::MAX_REASONABLE_THROW) {
      // likely bogus — reset window around current pos
      max_pos_ = pos;
      min_pos_ = pos;
      pos_window_.clear();
      crankshaft_throw_ = 0;
      is_throw_valid_ = false;
    } else {
      crankshaft_throw_ = raw_throw;
      is_throw_valid_ = (crankshaft_throw_ >= config::MIN_THROW_FOR_MOTION_MM);
    }

    // center crossing RPM estimator (two crossings = full cycle) with
    // hysteresis band
    float center = (max_pos_ + min_pos_) * 0.5f;
    const float hysteresis_band = config::MIN_MAX_HYST_MM * 0.5f;
    Edge curr_edge = Edge::UNKNOWN;
    if (pos > center + hysteresis_band) {
      curr_edge = Edge::ABOVE;
    } else if (pos < center - hysteresis_band) {
      curr_edge = Edge::BELOW;
    }
    // if we're still inside band, keep previous edge (no new crossing)
    // now check if a cross happened with the following condition
    if (curr_edge != Edge::UNKNOWN && curr_edge != last_edge_ &&
        is_throw_valid_) {
      if (last_zero_cross_ms_ != 0) {
        // a half cycle occurs when we hit the second center cross timestamp,
        // calculate that time interval here
        uint32_t half = now - last_zero_cross_ms_;
        // reject unreasonable half cycle calculations
        if (half > config::MIN_HALF_CYCLE_MS &&
            half < config::MAX_HALF_CYCLE_MS) {
          uint32_t full_period = half * 2;
          period_window_.add(full_period);
          float avg_period = period_window_.average();
          if (avg_period > 0.0f) {
            // calculate raw RPM value from period, sanity check, and apply
            // EMA filtering, and then clamp
            float raw_rpm = 60000.0f / avg_period;
            if (raw_rpm > config::MIN_DETECTED_RPM &&
                raw_rpm < config::MAX_DETECTED_RPM) {
              if (!rpm_init_) {
                rpm_ = raw_rpm;
                rpm_init_ = true;
              } else {
                rpm_ = rpm_ + config::EMA_RPM_ALPHA * (raw_rpm - rpm_);
                rpm_ = config::clampf(rpm_, 0.0f, config::MAX_DETECTED_RPM);
              }
              last_rpm_update_ms_ = now;
            }
          }
        }
      }
      last_zero_cross_ms_ = now;
      last_edge_ = curr_edge;
    }
  }

  // if motion times out, we reset everything
  if (moving_ && (now - last_motion_ms_ > config::MOTION_TIMEOUT_MS)) {
    DEBUG_PRINTLN("Motion timeout — reset");
    reset(pos);
  }

  last_pos_ = pos;
}

void MotionTracker::reset(float pos) {
  period_window_.clear();
  pos_window_.clear();
  moving_ = false;
  last_pos_ = pos;
  max_pos_ = pos;
  min_pos_ = pos;
  crankshaft_throw_ = 0.0f;
  last_zero_cross_ms_ = 0;
  last_rpm_update_ms_ = 0;
  rpm_init_ = false;
  rpm_ = 0.0f;
  last_edge_ = Edge::UNKNOWN;
}

void MotionTracker::decayRPM() {
  uint32_t now = millis();
  if (rpm_ > 0 && (now - last_rpm_update_ms_ > config::RPM_DECAY_TIMEOUT_MS)) {
    rpm_ *= config::RPM_DECAY_FACTOR;
    if (rpm_ < config::RPM_DECAY_MIN)
      rpm_ = 0.0f;
  }
}
