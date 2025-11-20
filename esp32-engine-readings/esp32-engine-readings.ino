/* 
* ----------------------------------------------
* PROJECT NAME: Piston Interactive
* File Name: engine_readings_w_VL53L1X.ino
* Description: full implementation of engine reading logic with M5Stack's ToF sensor which will be used in the piston interactive
* 
* Author: Isai Sanchez
* Original Contributions: Mike Heaton
* Changes:
*   - More comments, variable renaming and class creation
*   - Error handling for readings and calculations
*   - Sensor config might be different, was playing around with it for a while
*   - Motion detection logic
*   - Enhanced min/max readings with ring window to better calculate crankshaft throw
*   - Implemented hysteresis
*   - PWM signal sent from main ESP32 to QuinLED Dig Uno LED Drivers corresponding to 
*     torque and hp calculations
* Date: 7-12-25
* Board Used: ESP32-DevkitC-V4
* Libraries:
*   - Wire.h: https://docs.arduino.cc/language-reference/en/functions/communication/wire/
*       -- I2C communication
*   - VL53L1X.h: https://github.com/pololu/vl53l1x-arduino
*       -- Polulu's sensor driver library for the ToF sensor
* Notes:
*   - if I could split these classes up into their own files I would, but the arduino-cli was getting
*     mad at me when I tried to compile them, something about the build process with the esp32 that I've got to figure out
*   - the ToF sensor's region of interest (ROI) was modified to allow a smaller field of view (FoV) at 
*     the cost of losing sensitivity
*   - ring window is just a circular buffer, i.e. a data structure that behaves like a fixed-size 
*     buffer that wraps around itself, when the buffer is full and a new element is to be added, 
*     it overwrites the oldest element
* ----------------------------------------------
*/

#include <Wire.h>
#include <VL53L1X.h>

#include "App.h"

// ==================== SKETCH ========================
App app;

void setup() {
  app.setup();
}

void loop() {
  app.loopOnce();
  delay(2);  // light pacing, sensor runs in continuous mode
}
