#pragma once
/**
 * LedOutput.h
 *
 * initializes esp32 PWM channels, sends PWM signals, and performs calculations
 * to map a specific float to a specific duty cycle
 */

#include <Arduino.h>

class LedOutput {
public:
  LedOutput() = default;
  LedOutput(const LedOutput &) = delete;
  LedOutput &operator=(const LedOutput &) = delete;

  /**
   * @brief Initialize PWM channels
   *
   * @return success of PWM pin initialization
   */
  bool init();

  /**
   * @brief Sends PWM signals
   */
  void show(float torque, float hp);

private:
  /**
   * @brief Maps a float value to a specific duty cycle
   */
  static uint32_t mapFloatToDuty(float v, float vmax);
};
