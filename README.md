# Piston Interactive

## Project Overview
This document provides a comprehensive breakdown of the Arduino sketch flow for the Piston Interactive system, including the main ESP32 code for sensor readings and engine parameter calculations, and the QuinLED driver code for LED bar visualization.

## Hardware Components
- Microcontroller: ESP32-DevKitC-V4
- ToF Sensor: M5Stack's VL53L1X ToF Distance Unit. Found [here](https://shop.m5stack.com/products/time-of-flight-distance-unit-vl53l1x?srsltid=AfmBOoprDGgPCZlY4ets509p4m7cXj-nKKdMHRDG5hY20O5jZdbu7gsj)
- LED Drivers: 2 x [QuinLED Dig Uno](https://quinled.info/quinled-dig-uno/) drivers
- LED Strips: 10 x 12V WS2815 LED Strips

## ESP32 Software Architecture

### Overview
The esp32's sketch handles proper ToF sensor data acquisition and filtering, motion tracking logic, engine parameter calculations (rpms, torque, and horsepower), and creates corresponding PWM signal for the QuinLED driver board. Below is an overview of each class in the sketch and the role it plays in this project. I haven't figured out how to split these files into their respective .h and .cpp files without the Arduino to ESP32 build process getting angry at me, so if you have advice let me know!

### `RingWindow` class
Purpose: a lightweight ring window (a.k.a circular buffer) data structure used to hold the last N samples and provides the min, max, and average of that latest window

### `ToFSensor` Class
Purpose: The interface for the VL53L1X sensor, measuring piston head position

Key Methods:
1. `configure()`: sensor initialization with ROI and timing settings (see [Polulu's](https://github.com/pololu/vl53l1x-arduino) library for this sensor for more information on config settings)
    - note: in the future I would like to add auto sensor calibration based on scene/environment but these settings worked alright
2. `read()`: position reading with EMA smoothing filter
3. `ready()`: non-blocking data availability check

Features: built-in timeout handling, validity range checking, exponential moving average filter for raw position filtering

### `MotionTracker` Class
Purpose: motion detection and RPM calculation from position data (zero-cross detection)

Key Methods:
1. `update()`: motion detection, throw calculation, zero-crossing detection logic, and motion timeout reset (more on these below)
2. `decayRPM()`: RPM decaying function when motion of crankshaft stops to simulate engine slowing down

Key Features:
- Motion Detection: simple calculation of change in position, only move forward if the absolute value of this change is greater than a pre-specified constant
```cpp
    float d = fabsf(pos - lastPos);

    if (d > cfg::MIN_MOTION_DELTA_MM) {...}
```
- Crankshaft Throw Calculation: once ring window for min/max are updated, the throw is just (min-max)/2, if this calculated throw passes some validity checks, we update crankshaftThrow and set the throwValid flag to move on to center crossing
- Center Crossing RPM calculation: this is the fun part, 

### Serial Output Format
```
CSV Header: "Position,Min,Max,Throw,RPM,Torque,HP"
Data Line:  "crankshaftThrow,rpm,torque,horsepower"
```
