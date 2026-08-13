#pragma once
/**
 * Config.h
 *
 * Centralized configuration constants used by the esp32 sketch.
 * - make sure to `#include "Config.h"` then use config::XXXXX
 * - units are encoded in the name (e.g. _MS, _US)
 */
#include <Arduino.h>
namespace config {
// --- WATCHDOG & BUS TIMEOUTS ---
constexpr uint32_t WDT_TIMEOUT_MS = 5000;
constexpr uint32_t WDT_IDLE_CORE_MASK = (1 << 0);
constexpr uint16_t I2C_TIMEOUT_MS = 100;
} // namespace config
