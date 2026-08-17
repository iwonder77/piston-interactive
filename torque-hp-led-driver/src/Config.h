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
constexpr uint16_t INITIAL_TARGET_LEDS = 20;
constexpr uint32_t FRAME_INTERVAL_MS = 2;

// --- IDLE ANIMATION ---
constexpr uint32_t IDLE_TIMEOUT_MS =
    5000; // start idling after 5 s of no change
constexpr uint16_t NUM_ANIMATION_LEDS = 30;
constexpr float WAVE_PHASE_STEP = 0.01f;     // breathing speed per frame
constexpr float WAVE_AMPLITUDE = 0.25f;      // oscillate +/-25% around target
constexpr float WAVE_SPATIAL_FREQ = 0.3f;    // phase offset per LED along strip
constexpr float WAVE_INTENSITY_SCALE = 0.3f; // these two map the wave onto
constexpr float WAVE_INTENSITY_OFFSET = 0.7f; // the 0.4 .. 1.0 range
constexpr float EDGE_FADE = 0.5f;             // 50% dimmer at the strip edges
constexpr uint8_t JITTER_CHANCE_PCT = 10;     // chance per frame of +/-1 LED
// idle colour, kept as components so Config.h needn't pull in FastLED
constexpr uint8_t IDLE_COLOR_R = 255;
constexpr uint8_t IDLE_COLOR_G = 80;
constexpr uint8_t IDLE_COLOR_B = 0;
} // namespace config
