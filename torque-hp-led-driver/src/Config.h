#pragma once
/**
 * Config.h
 *
 * Centralized configuration constants used by the QuinLED Dig Uno sketch.
 * - make sure to `#include "Config.h"` then use config::XXXXX
 * - units are encoded in the name (e.g. _MS, _US)
 */

#include <Arduino.h>

namespace config {
// --- WATCHDOG ---
// NOTE: the ESP32 core boots with the task watchdog already running (5 s,
// panic enabled), so the sketch reconfigures rather than initializes it
constexpr uint32_t WDT_TIMEOUT_MS = 5000;
// bitmask of cores whose idle task the WDT also monitors; (1 << 0) preserves
// the core's boot default of watching CPU0's idle task - passing 0 would
// silently drop that protection
constexpr uint32_t WDT_IDLE_CORE_MASK = (1 << 0);

// --- PINS ---
constexpr uint8_t PWM_INPUT_PIN = 32; // PWM from the piston-dynamics ESP32
constexpr uint8_t LED_DATA_PIN = 3;   // WS2815 data line

// --- LED STRIP ---
constexpr uint16_t NUM_LEDS = 14;
constexpr uint8_t LED_BRIGHTNESS_PCT = 20;
// FastLED wants 0..255 rather than a percentage (styleguide 5.1)
constexpr uint8_t LED_BRIGHTNESS = (255 * LED_BRIGHTNESS_PCT) / 100;
// midpoint of the strip, used by the idle animation's edge fade
// (styleguide 5.1)
constexpr float HALF_STRIP_LEDS = NUM_LEDS * 0.5f;

// --- COLOURS ---
// kept as components so Config.h needn't pull in FastLED
constexpr uint8_t ACTIVE_COLOR_R = 0; // solid green while tracking the PWM
constexpr uint8_t ACTIVE_COLOR_G = 255;
constexpr uint8_t ACTIVE_COLOR_B = 0;
constexpr uint8_t IDLE_COLOR_R = 255; // warm amber while idling
constexpr uint8_t IDLE_COLOR_G = 80;
constexpr uint8_t IDLE_COLOR_B = 0;

// --- SHARED SCALES ---
constexpr uint8_t PERCENT_MAX = 100; // basis for percentage math and random()

// --- PWM CAPTURE ---
// NOTE: reject implausibly short periods before dividing by them - a glitch on
// the input line would otherwise produce a nonsense duty cycle
constexpr uint32_t MIN_VALID_PERIOD_US = 100;
constexpr uint16_t AVERAGE_SAMPLES = 5; // running average window

// --- DISPLAY RESPONSE ---
// NOTE: the LED count must move by at least this much before we redraw, which
// stops the bar flickering between two adjacent counts
constexpr uint8_t LED_HYSTERESIS_THRESHOLD = 2;
// NOTE: 50 fps. Must stay above the time FastLED.show() needs to clock out the
// strip - roughly 30 us per LED on WS2815, so ~1.4 ms for a 47-LED bar. A fixed
// interval is what keeps the idle animation's speed independent of how long the
// rest of the loop takes.
constexpr uint32_t FRAME_INTERVAL_MS = 20;

// --- IDLE ANIMATION ---
// NOTE: the idle animation lights the whole strip and varies brightness; it
// does not animate the LED count, so the last real reading has no effect on it
constexpr uint32_t IDLE_TIMEOUT_MS =
    5000;                                 // start idling after 5 s of no change
constexpr float WAVE_PHASE_WRAP = TWO_PI; // float copy of TWO_PI (styleguide 6)
// tune the breath by its period, not by a per-frame increment - the phase step
// is derived so changing FRAME_INTERVAL_MS cannot change the animation's speed
constexpr uint32_t BREATH_PERIOD_MS = 4000;
constexpr float WAVE_PHASE_STEP =
    WAVE_PHASE_WRAP * FRAME_INTERVAL_MS / BREATH_PERIOD_MS; // (styleguide 5.1)
constexpr float WAVE_AMPLITUDE = 0.25f;   // brightness swing: 1.0 down to 0.5
constexpr float WAVE_SPATIAL_FREQ = 0.3f; // phase offset per LED along strip
constexpr float WAVE_INTENSITY_SCALE = 0.3f;  // these two map the travelling
constexpr float WAVE_INTENSITY_OFFSET = 0.7f; // wave onto the 0.4 .. 1.0 range
constexpr float EDGE_FADE = 0.5f;             // 50% dimmer at the strip ends

// --- UTILITY FUNCTION ---
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}
} // namespace config
