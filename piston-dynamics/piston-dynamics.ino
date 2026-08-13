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
#include <esp_task_wdt.h>

#include "src/App.h"
#include "src/Config.h"
#include "src/Debug.h"

App app;

void setup() {
  Serial.begin(115200);
  delay(100);
  Wire.begin();
  Wire.setTimeOut(config::I2C_TIMEOUT_MS);
  delay(100);

  app.setup();

  // NOTE: the ESP32 core starts the task watchdog at boot but does not subscribe it
  // to the Arduino's loop task, so a hang inside loop() goes undetected. Reconfigure the timeout
  // then subscribe it.
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = config::WDT_TIMEOUT_MS,
    .idle_core_mask = config::WDT_IDLE_CORE_MASK,
    .trigger_panic = true,
  };
  if (esp_task_wdt_reconfigure(&wdt_config) != ESP_OK) {
    DEBUG_PRINTLN("WDT reconfigure failed");
  }
  if (esp_task_wdt_add(NULL) != ESP_OK) {
    DEBUG_PRINTLN("WDT subscribe failed");
  }
}

void loop() {
  esp_task_wdt_reset();
  app.loopOnce();
}
