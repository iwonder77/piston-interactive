#pragma once
/**
 * BarDisplay.h
 *
 * Owns the WS2815 strip and the FastLED controller, and renders the bar in
 * one of two looks:
 * - active: a solid run of LEDs proportional to the measured duty cycle
 * - idle: the whole strip lit, breathing in brightness
 */

#include <Arduino.h>
#include <FastLED.h>

#include "Config.h"

class BarDisplay {
public:
  BarDisplay() = default;
  // NOTE: owns the pixel buffer and the FastLED controller for this strip, so
  // copying would produce two objects each believing they drive it
  BarDisplay(const BarDisplay &) = delete;
  BarDisplay &operator=(const BarDisplay &) = delete;

  /**
   * @brief Registers the strip with FastLED, sets brightness, and blanks it.
   *
   * @return true always - FastLED reports no status from addLeds()
   */
  bool init();

  /**
   * @brief Renders one frame and pushes it to the strip.
   *
   * Advances the idle animation's phase internally, so call this exactly once
   * per frame.
   *
   * @param count LEDs to light while active; ignored while idling, since the
   * idle animation always spans the whole strip
   * @param is_idling true to play the breathing animation instead of a bar
   */
  void update(uint16_t count, bool is_idling);

private:
  void renderSolid(uint16_t count);
  void renderIdle();

  // brightness scale (0.0 .. 1.0) for one LED in the current idle frame;
  // `breath` is the frame-wide term, computed once by renderIdle()
  float intensityAt(uint16_t index, float breath) const;

  CRGB leds_[config::NUM_LEDS];
  float wave_phase_ = 0.0f;
};
