/**
 * Interactive: Piston Interactive
 * File: torque-hp-led-driver.ino
 * Description: QuinLED Dig Uno receives PWM signal whose strength is
 * proportional to the torque or horsepower produced by spinning crankshaft,
 * lights LED strip accordingly
 *
 * Author: Isai Sanchez
 * Date: 7-16-25
 * Board Used: QuinLED Dig Uno
 *
 * (c) Thanksgiving Point Exhibits Electronics Team — 2025
 */

#include <esp_task_wdt.h>

#include "src/BarDisplay.h"
#include "src/Config.h"
#include "src/Debug.h"
#include "src/PwmReader.h"

PwmReader pwm_reader;
BarDisplay display;

// last time the reading moved enough to count as activity
uint32_t last_activity_ms = 0;
// last time a frame was pushed to the strip
uint32_t last_frame_ms = 0;
// LED count the bar is currently showing
uint16_t current_target_leds = 0;
// NOTE: false until the first valid PWM measurement, which keeps the bar in its
// idle animation at boot rather than showing an LED count nothing measured
bool has_reading = false;

void setup() {
  Serial.begin(115200);

  if (!display.init()) {
    DEBUG_PRINTLN("LED display init failed");
  }
  if (!pwm_reader.init()) {
    DEBUG_PRINTLN("PWM reader init failed");
  }

  last_activity_ms = millis();
  last_frame_ms = last_activity_ms;

  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = config::WDT_TIMEOUT_MS,
      .idle_core_mask = config::WDT_IDLE_CORE_MASK,
      .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK) {
    DEBUG_PRINTLN("WDT reconfigure failed");
  }
  if (esp_task_wdt_add(NULL) != ESP_OK) {
    DEBUG_PRINTLN("WDT subscribe failed");
  }
}

void loop() {
  esp_task_wdt_reset();

  // NOTE: one time base for the whole pass (styleguide 6.1)
  const uint32_t now_ms = millis();

  float duty_cycle_pct = 0.0f;
  if (pwm_reader.read(duty_cycle_pct)) {
    // NOTE: mapped in float and rounded rather than using map(), which is
    // long-based and would truncate the fractional percent the running
    // average produces - biasing the bar low by up to a whole LED
    const float leds_f =
        duty_cycle_pct / config::PERCENT_MAX * (config::NUM_LEDS - 1);
    const uint16_t target_leds = static_cast<uint16_t>(
        config::clampf(leds_f + 0.5f, 0.0f, config::NUM_LEDS - 1));

    // only redraw once the reading has moved enough to be worth it
    const int32_t delta_leds = static_cast<int32_t>(target_leds) -
                               static_cast<int32_t>(current_target_leds);
    if (abs(delta_leds) >= config::LED_HYSTERESIS_THRESHOLD) {
      current_target_leds = target_leds;
      last_activity_ms = now_ms; // reset the idle timer
      has_reading = true;
    }
  }

  // NOTE: fixed-cadence, non-blocking frame gate (styleguide 6). Holding the
  // interval steady is also what keeps the breathing animation running at the
  // same speed regardless of strip length or how long a read() takes.
  if (now_ms - last_frame_ms >= config::FRAME_INTERVAL_MS) {
    last_frame_ms = now_ms;
    const bool is_idling =
        !has_reading || (now_ms - last_activity_ms) > config::IDLE_TIMEOUT_MS;
    display.update(current_target_leds, is_idling);
  }
}
