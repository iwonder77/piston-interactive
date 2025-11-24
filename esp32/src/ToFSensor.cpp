#include "ToFSensor.h"
#include "Config.h"

/**
 * @brief Custom VL53L1X sensor configuration
 *
 * @return Successful sensor initialization
 */
bool ToFSensor::configure() {
  // see Pololu's VL53L1X github repo for more inforamtion
  tof.setTimeout(config::SENSOR_TIMEOUT_MS);
  if (!tof.init()) {
    return false;
  }
  // set the region of interest (ROI) to 6x6 pixels (smaller ROI = narrower FoV
  // = better accuracy, less noise)
  tof.setROISize(config::ROI_W, config::ROI_H);

  // set the center of the sensor's Region of Interest (ROI)
  tof.setROICenter(config::ROI_CENTER);

  // set the distance mode to short (available choices are Short, Medium, Long)
  tof.setDistanceMode(VL53L1X::Short);

  // measurement timing budget
  tof.setMeasurementTimingBudget(config::TIMING_BUDGET_US);

  // the specified inter-measurement period in milliseconds determines how often
  // the sensor takes a measurement
  tof.startContinuous(config::INTER_MEAS_MS);
  return true;
}

/**
 * @brief We must check Pololu's dataReady() before reading sensor data
 *
 * @return Sensor ready boolean
 */
bool ToFSensor::ready() { return tof.dataReady(); }

/**
 * @brief Method called to read sensor data
 *
 * @return Returns true when reading was successful, and modifies position
 * variable that was passed by reference
 */
bool ToFSensor::read(float &pos) {
  // Pololu read() clears data ready flag and returns a uint16_t type
  uint16_t rawReading = tof.read();

  // ensure reading makes sense
  if (rawReading == 0 || rawReading == 65535)
    return false;

  float f = static_cast<float>(rawReading);
  if (f < config::MIN_VALID_MM || f > config::MAX_VALID_MM)
    return false;

  // smooth readings with EMA filter
  if (firstReading) {
    currentPosition = f;
    firstReading = false;
  } else {
    currentPosition =
        currentPosition + config::EMA_POS_ALPHA * (f - currentPosition);
  }

  pos = currentPosition;
  lastSensorRead = millis();

  return true;
}
