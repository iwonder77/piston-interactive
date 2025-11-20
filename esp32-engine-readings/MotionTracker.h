#include "Config.h"
#include "RingWindow.h"
#include <Arduino.h>

// ================ MOTION TRACKER ================
class MotionTracker {
public:
  MotionTracker()
      : window(config::PEAK_MIN_MAX_WINDOW_SIZE),
        periodWindow(config::PEAK_PERIOD_WINDOW_SIZE) {}

  void reset(float pos) {
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

  void update(float pos) {
    uint32_t now = millis();
    float d = fabsf(pos - lastPos);

    if (d > config::MIN_MOTION_DELTA_MM) {
      if (!moving) {
        moving = true;
        Serial.println("Motion detected — tracking");
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
          // reject unreasonable calcs
          if (half > 10 && half < 10000) {
            uint32_t fullPeriod = half * 2;
            periodWindow.add(fullPeriod);
            float avgPeriod = periodWindow.average();
            if (avgPeriod > 0.0f) {
              // calculate raw RPM value from period, sanity check, and apply
              // EMA filtering
              float rawRPM = 60000.0f / avgPeriod;
              if (rawRPM > config::MIN_DETECTED_RPM &&
                  rawRPM < config::MAX_DETECTED_RPM) {
                if (!rpm_init) {
                  rpm = rawRPM;
                  rpm_init = true;
                } else {
                  rpm = rpm + config::EMA_RPM_ALPHA * (rawRPM - rpm);
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

    // Motion timeout → reset
    if (moving && (now - lastMotionMs > config::MOTION_TIMEOUT_MS)) {
      Serial.println("Motion timeout — reset");
      reset(pos);
    }

    lastPos = pos;
  }

  bool isMoving() const {
    return moving && throwValid &&
           crankshaftThrow >= config::MIN_THROW_FOR_MOTION_MM;
  }
  float getCrankshaftThrowMM() const { return crankshaftThrow; }
  float getRPMs() const { return rpm; }
  uint32_t lastRPMms() const { return lastRPMUpdateMs; }

  void decayRPM() {
    // gentle decay if no fresh update (kid slows down spinning)
    uint32_t now = millis();
    if (rpm > 0 && (now - lastRPMUpdateMs > 1000)) {
      rpm *= 0.99f;
      if (rpm < 10.0f)
        rpm = 0.0f;
    }
  }

private:
  enum class Edge : uint8_t { UNKNOWN, ABOVE, BELOW };

  // --- period buffer for averaging ---
  RingWindow<size_t> periodWindow;

  RingWindow<size_t> window;
  bool moving = false;
  bool throwValid = false;
  float lastPos = 0;
  float maxPos = 0;
  float minPos = 0;
  float crankshaftThrow = 0;
  uint32_t lastMotionMs = 0;
  uint32_t lastZeroCrossMs = 0;
  uint32_t lastRPMUpdateMs = 0;
  float rpm = 0;
  bool rpm_init = false;
  Edge lastEdge = Edge::UNKNOWN;
};
// ====================================================
