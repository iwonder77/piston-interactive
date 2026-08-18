#include "LedOutput.h"
#include "Config.h"

bool LedOutput::init() {
  pinMode(config::TORQUE_LED_PIN, OUTPUT);
  pinMode(config::HP_LED_PIN, OUTPUT);
  bool torque_success = ledcAttach(config::TORQUE_LED_PIN, config::PWM_FREQ_HZ,
                                   config::PWM_RES_BITS);
  bool hp_success =
      ledcAttach(config::HP_LED_PIN, config::PWM_FREQ_HZ, config::PWM_RES_BITS);
  return torque_success && hp_success;
}

void LedOutput::show(float torque, float hp) {
  uint32_t torque_duty = mapFloatToDuty(torque, config::MAX_DISPLAY_TORQUE);
  uint32_t hp_duty = mapFloatToDuty(hp, config::MAX_DISPLAY_HP);
  ledcWrite(config::TORQUE_LED_PIN, torque_duty);
  ledcWrite(config::HP_LED_PIN, hp_duty);
}

uint32_t LedOutput::mapFloatToDuty(float v, float vmax) {
  v = config::clampf(v, 0.0f, vmax);
  if (v <= 0.0f) {
    return 0;
  }

  // Map 0..vmax → minDuty..maxDuty
  // but ensure max duty for PWM is display-specific based on piston size
  uint32_t effective_max = config::SMALL_PISTON_MAX_DUTY; // safe default
  switch (config::DISPLAY_TYPE) {
  case config::PistonSize::SMALL:
    effective_max = config::SMALL_PISTON_MAX_DUTY;
    break;
  case config::PistonSize::MEDIUM:
    effective_max = config::MEDIUM_PISTON_MAX_DUTY;
    break;
  case config::PistonSize::LARGE:
    effective_max = config::LARGE_PISTON_MAX_DUTY;
    break;
  }

  uint32_t duty = static_cast<uint32_t>(
      (v / vmax) * static_cast<float>(effective_max - config::PWM_MIN_DUTY) +
      config::PWM_MIN_DUTY + 0.5f);
  return duty;
}
