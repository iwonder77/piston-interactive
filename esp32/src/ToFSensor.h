#include "Config.h"
#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h>

// ==================== SENSOR ====================
class ToFSensor {
public:
  // sensor configuration
  bool configure() {
    Wire.begin();

    // see Pololu's VL53L1X github repo for more inforamtion on these config
    // methods
    tof.setTimeout(config::SENSOR_TIMEOUT_MS); // set timeout
    if (!tof.init())
      return false; // ensure init
    tof.setROISize(config::ROI_W,
                   config::ROI_H); // set the region of interest (ROI) to 6x6
                                   // pixels (smaller ROI = narrower FoV =
                                   // better accuracy, less noise)
    tof.setROICenter(config::ROI_CENTER); // set the center of the sensor's ROI
    tof.setDistanceMode(VL53L1X::Short);  // set the distance mode to short
                                          // (available are Short, Medium, Long)
    tof.setMeasurementTimingBudget(
        config::TIMING_BUDGET_US); // measurement timing budget
    tof.startContinuous(
        config::INTER_MEAS_MS); // the specified inter-measurement period in
                                // milliseconds determines how often the sensor
                                // takes a measurement
    return true;
  }

  // we must check Pololu's dataReady() before reading sensor data
  bool ready() { return tof.dataReady(); }

  // Returns true when reading was successful, and modifies position variable
  // that was passed by reference
  bool read(float &pos) {
    uint16_t rawReading = tof.read(); // Pololu read() clears data ready flag,
                                      // and returns a uint16_t type
    if (rawReading == 0 || rawReading == 65535)
      return false; // timeout or comm error
    float f = static_cast<float>(rawReading);
    if (f < config::MIN_VALID_MM || f > config::MAX_VALID_MM)
      return false;

    // EMA smoothing
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

  // get timestamp of last sensor read, allows us to detect stale sensors
  uint32_t getLastSensorRead() const { return lastSensorRead; }

private:
  VL53L1X tof;
  bool firstReading = true;
  float currentPosition = 1.0f;
  uint32_t lastSensorRead = 0;
};
// ================================================
