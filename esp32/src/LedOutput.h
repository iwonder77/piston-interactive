#pragma once
/**
 * LedOutput.h
 *
 * initializes esp32 PWM channels, sends PWM signals, and performs calculations
 * to map a specific float to a specific duty cycle
 */

#include <Arduino.h>

#include "Config.h"

class LedOutput {
public:
  /**
   * @brief Initialize PWM channels
   */
  bool beginPWM() {
    pinMode(config::TORQUE_LED_PIN, OUTPUT);
    pinMode(config::HP_LED_PIN, OUTPUT);
    bool torqueSuccess = ledcAttach(config::TORQUE_LED_PIN, config::PWM_FREQ_HZ,
                                    config::PWM_RES_BITS);
    bool hpSuccess = ledcAttach(config::HP_LED_PIN, config::PWM_FREQ_HZ,
                                config::PWM_RES_BITS);
    return torqueSuccess && hpSuccess;
  }

  /**
   * @brief Sends PWM signals
   */
  void show(float torque, float hp) {
    int t = mapFloatToDuty(torque, config::MAX_DISPLAY_TORQUE);
    int h = mapFloatToDuty(hp, config::MAX_DISPLAY_HP);
    ledcWrite(config::TORQUE_LED_PIN, t);
    ledcWrite(config::HP_LED_PIN, h);
  }

private:
  /**
   * @brief Maps a float value to a specific duty cycle
   */
  static int mapFloatToDuty(float v, float vmax) {
    v = config::clampf(v, 0.0f, vmax);
    if (v <= 0.0f) {
      return 0;
    }

    // Map 0..vmax → minDuty..maxDuty
    // but ensure max duty for PWM is display-specific based on piston size
    uint32_t effectiveMax = config::SMALL_PISTON_MAX_DUTY; // safe default
    switch (config::DISPLAY_TYPE) {
    case config::PistonSize::SMALL:
      effectiveMax = config::SMALL_PISTON_MAX_DUTY;
      break;
    case config::PistonSize::MEDIUM:
      effectiveMax = config::MEDIUM_PISTON_MAX_DUTY;
      break;
    case config::PistonSize::LARGE:
      effectiveMax = config::LARGE_PISTON_MAX_DUTY;
      break;
    }

    int duty = int((v / vmax) * float(effectiveMax - config::PWM_MIN_DUTY) +
                   config::PWM_MIN_DUTY + 0.5f);
    return duty;
  }
};
