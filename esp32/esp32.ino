/** 
* Interactive: Piston Interactive
* File: engine_readings_w_VL53L1X.ino
* Description: full implementation of engine reading logic with M5Stack's ToF sensor which will be used in the piston interactive
* 
* Author: Isai Sanchez
* Original Contributions: Mike Heaton
* Date: 7-12-25
* Board Used: ESP32-DevkitC-V4
* Notes:
*   - VL53L1X.h: https://github.com/pololu/vl53l1x-arduino (Polulu's sensor driver library for the ToF sensor)
*   - the ToF sensor's region of interest (ROI) was modified to allow a smaller field of view (FoV) at 
*     the cost of losing sensitivity
*   - ring window is just a circular buffer, i.e. a data structure that behaves like a fixed-size 
*     buffer that wraps around itself, when the buffer is full and a new element is to be added, 
*     it overwrites the oldest element
*
* (c) Thanksgiving Point Exhibits Electronics Team — 2025
*/

#include <Wire.h>
#include "src/App.h"

App app;

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();
  delay(100);

  app.setup();
}

void loop() {
  app.loopOnce();
  // delay(15); // not needed, App class handles sensor polling logic
}
