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
    return moving && throwValid &&
           crankshaftThrow >= config::MIN_THROW_FOR_MOTION_MM;
  }
  float getCrankshaftThrowMM() const { return crankshaftThrow; }
  float getRPMs() const { return rpm; }
  uint32_t getLastRPMUpdate() const { return lastRPMUpdateMs; }

private:
  enum class Edge : uint8_t { UNKNOWN, ABOVE, BELOW };

  RingWindow<size_t, config::PEAK_PERIOD_WINDOW_SIZE> periodWindow;
  RingWindow<size_t, config::PEAK_MIN_MAX_WINDOW_SIZE> window;

  bool moving = false;
  bool throwValid = false;
  float lastPos = 0;
  float maxPos = 0;
  float minPos = 0;
  float crankshaftThrow = 0;
  float rpm = 0;
  bool rpm_init = false;
  uint32_t lastMotionMs = 0;
  uint32_t lastZeroCrossMs = 0;
  uint32_t lastRPMUpdateMs = 0;
  Edge lastEdge = Edge::UNKNOWN;
};
