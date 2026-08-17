#include "PwmReader.h"

PwmReader *PwmReader::instance_ = nullptr;

bool PwmReader::init() {
  pinMode(config::PWM_INPUT_PIN, INPUT_PULLUP);
  instance_ = this;
  attachInterrupt(digitalPinToInterrupt(config::PWM_INPUT_PIN), isrTrampoline,
                  CHANGE);
  duty_cycle_filter_.clear();
  return true;
}

bool PwmReader::read(float &duty_cycle_pct) {
  if (!new_data_available_) {
    return false;
  }

  noInterrupts();
  uint32_t safe_pulse_width = pulse_width_;
  uint32_t safe_period = period_length_;
  new_data_available_ = false;
  interrupts();

  if (safe_period < config::MIN_VALID_PERIOD_US) {
    return false;
  }

  float duty_cycle =
      (float)safe_pulse_width / safe_period * config::PERCENT_MAX;
  duty_cycle = constrain(duty_cycle, 0, config::PERCENT_MAX);
  duty_cycle_filter_.add(duty_cycle);
  duty_cycle_pct = duty_cycle_filter_.average();
  return true;
}

void IRAM_ATTR PwmReader::isrTrampoline() {
  if (instance_)
    instance_->handleEdge();
}

void IRAM_ATTR PwmReader::handleEdge() {
  uint32_t current_time = micros();
  uint8_t pin_state = digitalRead(config::PWM_INPUT_PIN);

  switch (pin_state) {
  // rising edge (start of a new pulse)
  case HIGH:
    // calculate period (time between consecutive rising edges)
    if (last_period_start_ != 0) {
      period_length_ = current_time - last_period_start_;
    }
    last_period_start_ = current_time;
    pulse_start_time_ = current_time;
    break;
  // falling edge, record pulse width time
  case LOW:
    pulse_width_ = current_time - pulse_start_time_;
    new_data_available_ = true; // signal main loop that data is ready
    break;
  }
}
