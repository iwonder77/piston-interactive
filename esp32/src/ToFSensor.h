#pragma once
/**
 * ToFSensor.h
 *
 * Centralized sensor configuration and communication (via I2C)
 * - solely focuses on
 *   -- custom configuration (see configure() method)
 *   -- ensuring successful readings
 *   -- modifying position variable passed into read() with raw reading values
 */

#include <Arduino.h>
#include <VL53L1X.h>

class ToFSensor {
public:
  bool configure();
  bool ready();
  bool read(float &pos);

private:
  VL53L1X tof;
  bool firstReading = true;
  float currentPosition = 1.0f;
};
