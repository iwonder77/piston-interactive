#pragma once
/**
 * PwmReader.h
 *
 * Class to handle input PWM duty cycle measurements using an ISR
 */
#include <Arduino.h>

#include "Config.h"
#include "RingWindow.h"

class PwmReader {
public:
  PwmReader() = default;
  PwmReader(const PwmReader &) = delete;
  PwmReader &operator=(const PwmReader &) = delete;

  bool init();
  bool read(float &duty_cycle_pct);

private:
  RingWindow<float, config::AVERAGE_SAMPLES> duty_cycle_filter_;
  // ----- TIMER INTERRUPT VARIABLES FOR PWM CALC -----
  volatile uint32_t pulse_start_time_ = 0;
  volatile uint32_t pulse_width_ = 0;
  volatile uint32_t last_period_start_ = 0;
  volatile uint32_t period_length_ = 0;
  volatile bool new_data_available_ = false;

  static PwmReader *instance_;
  static void IRAM_ATTR isrTrampoline();
  void IRAM_ATTR handleEdge();
};
