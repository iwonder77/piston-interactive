#pragma once
/**
 * ToFSensor.h
 *
 * Centralized sensor configuration and communication (via I2C),
 * solely focuses on:
 * -- custom sensor configuration (see init() method)
 * -- ensuring successful readings
 * -- modifying position variable passed into read() with raw reading values
 */

#include <Arduino.h>
#include <VL53L1X.h>

class ToFSensor {
public:
  ToFSensor() = default;
  // delete copy constructor AND assignment because this class must be
  // the SINGLE OWNER of the VL53L1X on the I2C bus (to re-initialize, simply
  // call init() on the existing object)
  ToFSensor(const ToFSensor &) = delete;
  ToFSensor &operator=(const ToFSensor &) = delete;

  /**
   * @brief Custom VL53L1X sensor configuration.
   *
   * Safe to call again on a live object to recover a stalled sensor; resets the
   * EMA filter state so the next read() seeds from scratch
   *
   * @return true if the sensor initialized and continuous ranging started
   */
  bool init();

  /**
   * @brief We must check Pololu's dataReady() before reading sensor data
   *
   * @return Sensor ready boolean
   */
  bool ready();

  /**
   * @brief Method called to read sensor data
   *
   * @param pos modified with the filtered position reading on success
   * @return Returns true when reading was valid
   */
  bool read(float &pos);

private:
  VL53L1X tof_;
  bool first_reading_ = true;
  float current_position_ = 0.0f;
};
