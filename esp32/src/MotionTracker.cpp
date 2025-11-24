#include "MotionTracker.h"
#include "Debug.h"
#include "src/Config.h"

/**
 * @brief Updates motion tracking with new position reading
 *
 * Detects motion, updates all ring windows, calculates throw and RPM
 *
 * @param pos Filtered sensor reading
 */
void MotionTracker::update(float pos) {
  uint32_t now = millis();
  float d = fabsf(pos - lastPos);

  if (d > config::MIN_MOTION_DELTA_MM) {
    if (!moving) {
      moving = true;
      DEBUG_PRINTLN("Motion detected — tracking");
    }
    lastMotionMs = now;

    // update ring window and hysteretic min/max
    window.add(pos);
    float wMax = window.getMax();
    float wMin = window.getMin();
    // hysteretic min/max updates
    if (!isnan(wMax) && fabs(wMax - maxPos) > config::MIN_MAX_HYST_MM) {
      maxPos = wMax;
    }
    if (!isnan(wMin) && fabs(wMin - minPos) > config::MIN_MAX_HYST_MM) {
      minPos = wMin;
    }

    // throw (radius) ~ (max-min)/2
    float rawThrow = (maxPos - minPos) * 0.5f;
    if (rawThrow < config::MIN_REASONABLE_THROW) {
      crankshaftThrow = 0.0f;
      throwValid = false;
    } else if (rawThrow > config::MAX_REASONABLE_THROW) {
      // likely bogus — reset window around current pos
      maxPos = pos;
      minPos = pos;
      window.clear();
      crankshaftThrow = 0;
      throwValid = false;
    } else {
      crankshaftThrow = rawThrow;
      throwValid = (crankshaftThrow >= config::MIN_THROW_FOR_MOTION_MM);
    }

    // center crossing RPM estimator (two crossings = full cycle) with
    // hysteresis band
    float center = (maxPos + minPos) * 0.5f;
    const float hysteresisBand = config::MIN_MAX_HYST_MM * 0.5f;
    Edge currEdge = Edge::UNKNOWN;
    if (pos > center + hysteresisBand) {
      currEdge = Edge::ABOVE;
    } else if (pos < center - hysteresisBand) {
      currEdge = Edge::BELOW;
    }
    // if we're still inside band, keep previous edge (no new crossing)
    // now check if a cross happened with the following condition
    if (currEdge != Edge::UNKNOWN && currEdge != lastEdge && throwValid) {
      if (lastZeroCrossMs != 0) {
        // a half cycle occurs when we hit the second center cross timestamp,
        // calculate that time interval here
        uint32_t half = now - lastZeroCrossMs;
        // reject unreasonable half cycle calculations
        if (half > config::MIN_HALF_CYCLE_MS &&
            half < config::MAX_HALF_CYCLE_MS) {
          uint32_t fullPeriod = half * 2;
          periodWindow.add(fullPeriod);
          float avgPeriod = periodWindow.average();
          if (avgPeriod > 0.0f) {
            // calculate raw RPM value from period, sanity check, and apply
            // EMA filtering, and then clamp
            float rawRPM = 60000.0f / avgPeriod;
            if (rawRPM > config::MIN_DETECTED_RPM &&
                rawRPM < config::MAX_DETECTED_RPM) {
              if (!rpm_init) {
                rpm = rawRPM;
                rpm_init = true;
              } else {
                rpm = rpm + config::EMA_RPM_ALPHA * (rawRPM - rpm);
                rpm = config::clampf(rpm, 0.0f, config::MAX_DETECTED_RPM);
              }
              lastRPMUpdateMs = now;
            }
          }
        }
      }
      lastZeroCrossMs = now;
      lastEdge = currEdge;
    }
  }

  // if motion times out, we reset everything
  if (moving && (now - lastMotionMs > config::MOTION_TIMEOUT_MS)) {
    DEBUG_PRINTLN("Motion timeout — reset");
    reset(pos);
  }

  lastPos = pos;
}

/**
 * @brief Resets the class members, ring window objects, and center crossing
 * state machine
 *
 * @param pos Value to reset members with
 */
void MotionTracker::reset(float pos) {
  periodWindow.clear();
  window.clear();
  moving = false;
  lastPos = pos;
  maxPos = pos;
  minPos = pos;
  crankshaftThrow = 0.0f;
  lastZeroCrossMs = 0;
  lastRPMUpdateMs = 0;
  rpm_init = false;
  rpm = 0.0f;
  lastEdge = Edge::UNKNOWN;
}

/**
 * @brief Applies gentle decay to rpm value when, for example, a
 * kid stops spinning the crankshaft
 */
void MotionTracker::decayRPM() {
  uint32_t now = millis();
  if (rpm > 0 && (now - lastRPMUpdateMs > config::RPM_DECAY_TIMEOUT_MS)) {
    rpm *= config::RPM_DECAY_FACTOR;
    if (rpm < config::RPM_DECAY_MIN)
      rpm = 0.0f;
  }
}
