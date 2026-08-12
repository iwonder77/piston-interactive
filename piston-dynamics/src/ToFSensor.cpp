#include "ToFSensor.h"
#include "Config.h"
#include <cstdint>

bool ToFSensor::init() {
  // NOTE: re-init path - clear filter state so the next read() seeds the EMA
  // rather than blending fresh readings
  first_reading_ = true;
  current_position_ = 0.0f;

  // see Pololu's VL53L1X github repo for more information
  tof_.setTimeout(config::SENSOR_TIMEOUT_MS);
  if (!tof_.init()) {
    return false;
  }
  // set the region of interest (ROI) to 6x6 pixels (smaller ROI = narrower FoV
  // = better accuracy, less noise)
  tof_.setROISize(config::ROI_W, config::ROI_H);

  // set the center of the sensor's Region of Interest (ROI)
  tof_.setROICenter(config::ROI_CENTER);

  // set the distance mode to short (available choices are Short, Medium, Long)
  tof_.setDistanceMode(VL53L1X::Short);

  // measurement timing budget
  tof_.setMeasurementTimingBudget(config::TIMING_BUDGET_US);

  // the specified inter-measurement period in milliseconds determines how often
  // the sensor takes a measurement
  tof_.startContinuous(config::INTER_MEAS_MS);
  return true;
}

bool ToFSensor::ready() { return tof_.dataReady(); }

bool ToFSensor::read(float &pos) {
  // Pololu read() clears data ready flag and returns a uint16_t type
  uint16_t raw_reading = tof_.read();

  // ensure reading makes sense
  if (raw_reading == 0 || raw_reading == UINT16_MAX)
    return false;

  float f = static_cast<float>(raw_reading);
  if (f < config::MIN_VALID_MM || f > config::MAX_VALID_MM)
    return false;

  // smooth readings with EMA filter
  if (first_reading_) {
    current_position_ = f;
    first_reading_ = false;
  } else {
    current_position_ =
        current_position_ + config::EMA_POS_ALPHA * (f - current_position_);
  }

  pos = current_position_;

  return true;
}
