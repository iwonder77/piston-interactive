/* 
* ----------------------------------------------
* PROJECT NAME: engine_readings_w_VL53L1X
* Description: testing M5Stack's ToF sensor which will be used in the piston interactive
* 
* Original Author: Mike Heaton
* Modified by: Isai Sanchez
* Changes:
*   - More comments, variable renaming and function creation
*   - Error Handling for readings and calculations
*   - Sensor config might be different, was playing around with it for a while
*   - Motion detection logic
*   - Enhanced min/max readings to better calculate crankshaft throw
* Date: 7-12-25
* Board Used: ESP32-DevkitC-V4
* Libraries:
*   - Wire.h: https://docs.arduino.cc/language-reference/en/functions/communication/wire/
*       -- I2C communication
*   - VL53L1X.h: https://github.com/pololu/vl53l1x-arduino
*       -- sensor driver library for the ToF sensor we are using 
*   - RunningAverage.h: https://github.com/RobTillaart/RunningAverage
*       -- circular buffer library used to smoothen out sample readings
* Notes:
*   - the sensors region of interest (ROI) was modified to allow a smaller field of view (FoV) at 
*     the cost of losing sensitivity
*   - circular buffer is a data structure that behaves like a fixed-size buffer that wraps around itself,
*     when the buffer is full and a new element is to be added, it overwrites the oldest element
* ----------------------------------------------
*/

#include <Wire.h>
#include <VL53L1X.h>
#include <RunningAverage.h>

VL53L1X sensor;

// ====================== CONFIG CONSTANTS ==================================
// error handling settings
const int MAX_SENSOR_ERRORS = 5;         // max consecutive errors
const float MIN_VALID_READING = 40.0;    // sensor range is minimum: 4 cm
const float MAX_VALID_READING = 4000.0;  // to maximum: 400cm
const unsigned long ERROR_RECOVERY_INTERVAL = 5000;

// motion detection settings
const float MIN_MOTION_THRESHOLD = 2.0;        // minimum change in position to detect motion (mm)
const unsigned long MOTION_TIMEOUT_MS = 3000;  // reset tracking after this many ms of no motion
const float MIN_THROW_FOR_MOTION = 5.0;        // minimum throw to consider system "in motion" (mm)

// enhanced settings for min/max detection
const float MIN_MAX_HYSTERESIS = 1.0;        // must exceed previous min/max by this amount to update
const float MIN_REASONABLE_THROW = 3.0;      // minimum detected throw that makes sense (mm)
const float MAX_REASONABLE_THROW = 150.0;    // maximum detected throw that makes sense (mm)
const int MIN_SAMPLES_FOR_VALID_THROW = 20;  // we need this many samples before throw is considered valid

// engine calculation settings
const float PISTON_AREA = 50.0;          // WARNING: fixed piston head area, must be different for each installation
const float PRESSURE_MULTIPLIER = 0.5;   // simplified pressure factor
const float TORQUE_SCALE_FACTOR = 0.2;   // scale torque for reasonable LED range
const float MAX_DISPLAY_RPM = 200.0;     // maximum rpm for display scaling
const float MAX_DISPLAY_TORQUE = 700.0;  // maximum torque for display scaling
const float MAX_DISPLAY_HP = 110.0;      // maximum HP for display scaling

// LED PWM pin configuration values
const int TORQUE_LED_PIN = 18;
const int HP_LED_PIN = 19;

// constants for RunningAverage objects (see below)
const int SAMPLE_WINDOW_SIZE = 10;   // number of samples to average for smoothing raw readings
const int MIN_MAX_WINDOW_SIZE = 50;  // window size for tracking min/max values
const int RPM_WINDOW_SIZE = 10;
// ============================================================================

// RunningAverage helps us calculate the running average of a continuous sample of readings
// we use it here as a light, low impact filter-ish object to smooth out/take average of past n
// raw sensor samples
RunningAverage sampleFilter(SAMPLE_WINDOW_SIZE);
RunningAverage minMaxWindow(MIN_MAX_WINDOW_SIZE);
RunningAverage rpmFilter(RPM_WINDOW_SIZE);

// ======================= GLOBAL VARIABLES ===================================
// sensor reading variables (position of piston head)
float currentPosition = 0.0;
float lastPosition = 0.0;
float maxPosition = 0.0;
float minPosition = 1000.0;
float crankshaftThrow = 0.0;

// error handling variables
int sensorErrorCount = 0;             // count of consecutive sensor errors
bool systemHealthy = true;            // overall system health status
unsigned long lastErrorRecovery = 0;  // last time we tried to recover from errors

// timing variable
unsigned long lastSensorRead = 0;

// motion detection variables
bool isMoving = false;
unsigned long lastMotionTime = 0;  // timestamp of last detected motion
float totalMotionDistance = 0.0;   // total distance moved (for debugging)

// enhanced tracking variables
int samplesInCurrentMotion = 0;  // how many samples we've collected during a motion sequence
bool throwIsValid = false;       // true if we have enough samples for a valid throw measurement

// engine calculation variables
float rpm = 0.0;
float torque = 0.0;
float horsepower = 0.0;
bool lastDirectionUp = true;         // direction of last motion (for RPM calculation)
unsigned long lastZeroCrossing = 0;  // time of last direction change
unsigned long cyclePeriod = 0;       // time for complete cycle (ms)
unsigned long lastRPMUpdate = 0;     // last time RPM was calculated
// ===========================================================================



// ========== MAIN SETUP ==========
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(TORQUE_LED_PIN, OUTPUT);
  pinMode(HP_LED_PIN, OUTPUT);

  analogWrite(TORQUE_LED_PIN, 0);
  analogWrite(HP_LED_PIN, 0);

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect sensor, pls fix nerd!!");
    systemHealthy = false;
  } else {
    configureSensor();
  }

  // clear out our filters
  sampleFilter.clear();
  minMaxWindow.clear();
  rpmFilter.clear();

  // initialize motion detection
  lastMotionTime = millis();

  Serial.println("Position,Min,Max,Throw,RPM,Torque,HP");
}

// ========== SENSOR CONFIGURATION FUNCTION ==========
// see Polulu's VL53L1X github repo for more inforamtion on these config methods
void configureSensor() {
  sensor.setROISize(6, 6);                   // set the region of interest (ROI) to 4x4 pixels (smaller ROI = narrower FoV = better accuracy, less noise)
  sensor.setROICenter(199);                  // set the center of the sensor's ROI
  sensor.setDistanceMode(VL53L1X::Short);    // set the distance mode to short (available are Short, Medium, Long)
  sensor.setMeasurementTimingBudget(15000);  // measurement timing budget
  sensor.startContinuous(5);                 // the specified inter-measurement period in milliseconds determines how often the sensor takes a measurement,
}

// ========== MAIN LOOP ==========
void loop() {
  if (!systemHealthy) {
    attemptSystemRecovery();
    delay(100);
    return;
  }
  if (sensor.dataReady()) {
    if (readAndProcessSensor()) {
      // check for motion timeout after successful reading
      checkMotionTimeout();

      // calculate engine parameters and update LEDs
      calculateEngineParameters();
      updateLEDs();

      //outputData();
    }
  }
  // small delay to prevent overwhelming system
  delay(10);
}

// ========== SENSOR READING FUNCTION ==========
bool readAndProcessSensor() {
  float rawReading = sensor.read();  // latest raw sensor reading (mm)

  // validate the sensor reading
  if (!isValidSensorReading(rawReading)) {
    handleSensorError();
    return false;  // Reading failed
  }

  if (sensorErrorCount > 0) {
    sensorErrorCount = 0;
    Serial.println("Sensor errors cleared - readings restored");
  }

  // filter sample (mm)
  sampleFilter.add(rawReading);                 // add latest sample to the sliding window of 5 samples
  currentPosition = sampleFilter.getAverage();  // get average to smoothen out noise (mm)

  // detect motion before updating min/max
  detectMotion();

  // update min/max tracking
  updateMinMaxValues();

  calculateValidatedThrow();

  // record when we last read the sensor
  lastSensorRead = millis();

  return true;
}

// ========== MOTION DETECTION FUNCTION ==========
void detectMotion() {
  // calculate how much the position has changed
  float positionChange = abs(currentPosition - lastPosition);

  // check if change is significant enough to be considered motion
  if (positionChange > MIN_MOTION_THRESHOLD) {
    // if so, motion detected!
    if (!isMoving) {
      Serial.println("Motion detected - starting tracking");
      isMoving = true;
      samplesInCurrentMotion = 0;  // reset sample counter
      throwIsValid = false;        // reset validity flag
    }

    // update motion tracking
    lastMotionTime = millis();
    totalMotionDistance += positionChange;
    samplesInCurrentMotion++;

    // additional check: only consider it "real motion" if we have sufficient samples to calculate throw
    if (samplesInCurrentMotion >= MIN_SAMPLES_FOR_VALID_THROW) {
      throwIsValid = true;
    }

    // calculate RPMs from direction changes
    calculateRPMs();
  }

  // store current position for next comparison
  lastPosition = currentPosition;
}

// ========== RPM CALCULATION FUNCTION ==========
void calculateRPMs() {
  // determine current direction of motion
  bool currentDirectionUp = (currentPosition > lastPosition);

  // calculate center position (midpoint between min and max)
  float centerPosition = (maxPosition + minPosition) / 2.0;
  bool crossedCenter = (lastPosition - centerPosition) * (currentPosition - centerPosition) < 0;

  // check if we've crossed the center of the motion to calculate RPMs
  if (crossedCenter && crankshaftThrow > MIN_THROW_FOR_MOTION) {
    // calculate cycle period (two zero crossings = one complete cycle)
    unsigned long currentTime = millis();
    if (lastZeroCrossing > 0) {
      unsigned long halfCycle = currentTime - lastZeroCrossing;
      cyclePeriod = halfCycle * 2;

      // calculate RPM from cycle period (ensure cycle period is reasonable first)
      if (cyclePeriod > 0 && cyclePeriod < 10000) {
        float rawRPM = 60000.0 / cyclePeriod;

        // filter RPM for stability and only accept reasonable values
        if (rawRPM > 5 && rawRPM < MAX_DISPLAY_RPM) {
          rpmFilter.add(rawRPM);
          rpm = rpmFilter.getAverage();
          lastRPMUpdate = currentTime;  // record when we last updated RPM
        }
      }
    }
    lastZeroCrossing = currentTime;
  }
  lastDirectionUp = currentDirectionUp;
}


// ========== MIN/MAX TRACKING FUNCTION ==========
void updateMinMaxValues() {
  // only update min/max if we're actually moving
  // this prevents noise from affecting our measurements when stationary
  if (!isMoving) {
    return;
  }

  minMaxWindow.add(currentPosition);

  // Get min and max from the tracking window
  float windowMax = minMaxWindow.getMaxInBuffer();
  float windowMin = minMaxWindow.getMinInBuffer();

  // apply hysteresis - new max must be significantly higher than current max
  if (windowMax > maxPosition + MIN_MAX_HYSTERESIS) {
    maxPosition = windowMax;
    Serial.print("New MAX detected: ");
    Serial.println(maxPosition);
  }

  // apply hysteresis - new min must be significantly lower than current min
  if (windowMin < minPosition - MIN_MAX_HYSTERESIS) {
    minPosition = windowMin;
    Serial.print("New MIN detected: ");
    Serial.println(minPosition);
  }
}

// ========== VALIDATED THROW CALCULATION FUNCTION ==========
void calculateValidatedThrow() {
  float rawThrow = (maxPosition - minPosition) / 2.0;

  // validate throw is within reasonable bounds
  if (rawThrow < MIN_REASONABLE_THROW) {
    // too small, probably just noise or very small motion
    crankshaftThrow = 0.0;
    throwIsValid = false;
  } else if (rawThrow > MAX_REASONABLE_THROW) {
    Serial.println("WARNING: Throw too large, resetting tracking");
    resetMinMaxTracking();
    crankshaftThrow = 0.0;
    throwIsValid = false;
  } else {
    // reasonable throw value
    crankshaftThrow = rawThrow;
  }
}

// ========== ENGINE PARAMETER CALCULATIONS ==========
void calculateEngineParameters() {
  if (!isMoving || !throwIsValid || crankshaftThrow < MIN_THROW_FOR_MOTION) {
    // no valid motion, set everything to "zero"
    rpm = 0.0;
    torque = 0.0;
    horsepower = 0.0;
    rpmFilter.clear();
    return;
  }

  // apply RPM decay if we haven't had a recent RPM update (means cranking is slowing down)
  unsigned long timeSinceRPMUpdate = millis() - lastRPMUpdate;
  if (timeSinceRPMUpdate > 1000) {
    // gradually decay RPM to reflect slowing down
    float decayFactor = 0.99;
    rpm = rpm * decayFactor;

    // clear RPM filter if RPM gets very low
    if (rpm < 10) {
      rpm = 0.0;
      rpmFilter.clear();
    }
  }

  // calculate torque estimate
  // simplified formula: Torque ∝ throw × piston_area × "pressure"
  // we'll use RPM² as a proxy for pressure (higher RPM = higher forces)
  float pressureEstimate = (rpm * rpm) / 10000.0;  // Scale down RPM²
  torque = crankshaftThrow * PISTON_AREA * pressureEstimate * TORQUE_SCALE_FACTOR;

  // calculate horsepower estimate
  // standard formula: HP = (TORQUE * RPM) / 5252
  // we'll use a simplified version scaled for our display
  if (rpm > 0) {
    horsepower = (torque * rpm) / 1000.0;
  } else {
    horsepower = 0.0;
  }

  // constrain values to reasonable display ranges
  torque = constrain(torque, 0, MAX_DISPLAY_TORQUE);
  horsepower = constrain(horsepower, 0, MAX_DISPLAY_HP);
}

// ========== LED UPDATE FUNCTION ==========
void updateLEDs() {
  // Scale torque and HP to 0–255 for PWM
  int torquePWM = (int)(255.0 * torque / MAX_DISPLAY_TORQUE);
  int hpPWM = (int)(255.0 * horsepower / MAX_DISPLAY_HP);

  // clamp values to not go above or below optimal PWM strength
  torquePWM = constrain(torquePWM, 0, 255);
  hpPWM = constrain(hpPWM, 0, 255);

  Serial.print(torquePWM);
  Serial.print(",");
  Serial.print(hpPWM);
  Serial.println();

  // Send to PWM pins
  analogWrite(TORQUE_LED_PIN, torquePWM);
  analogWrite(HP_LED_PIN, hpPWM);
}

// ========== MOTION TIMEOUT CHECKING ==========
void checkMotionTimeout() {
  // if no motion is detected for "MOTION_TIMOUT_MS" ms, we assume the user stopped cranking and
  // reset all variables
  if (isMoving && (millis() - lastMotionTime > MOTION_TIMEOUT_MS)) {
    Serial.println("Motion timeout - resetting tracking...");
    stopMotionTracking();
  }
}

// ============= RESET MOTION TRACKING FUNCTION =============
void stopMotionTracking() {
  isMoving = false;
  totalMotionDistance = 0.0;
  samplesInCurrentMotion = 0;  // NEW: Reset sample counter
  throwIsValid = false;        // NEW: Reset validity

  rpm = 0.0;
  torque = 0.0;
  horsepower = 0.0;
  rpmFilter.clear();
  cyclePeriod = 0;
  lastZeroCrossing = 0;
  lastRPMUpdate = 0;

  resetMinMaxTracking();

  Serial.println("Enhanced motion tracking reset - ready for new motion");
}

// ============= RESET MIN/MAX TRACKING =============
void resetMinMaxTracking() {
  // Reset min/max values to current position
  maxPosition = currentPosition;
  minPosition = currentPosition;
  crankshaftThrow = 0.0;

  // Clear the min/max tracking window
  minMaxWindow.clear();

  // Add current position to start fresh
  minMaxWindow.add(currentPosition);
}


// ========== SENSOR READING VALIDATION FUNCTION ==========
bool isValidSensorReading(float reading) {
  // Check if reading is within reasonable bounds
  if (reading < MIN_VALID_READING) {
    Serial.print("Error: Reading too low -> ");
    Serial.println(reading);
    return false;
  }

  if (reading > MAX_VALID_READING) {
    Serial.print("Error: Reading too high -> ");
    Serial.println(reading);
    return false;
  }

  // Check for sensor timeout or communication error
  if (reading == 65535 || reading == 0) {
    Serial.println("ERROR: Sensor timeout or communication error");
    return false;
  }

  return true;  // Reading is valid
}

// ========== ERROR HANDLING FUNCTION ==========
void handleSensorError() {
  sensorErrorCount++;

  Serial.print("Sensor error #");
  Serial.print(sensorErrorCount);
  Serial.print(" of ");
  Serial.println(MAX_SENSOR_ERRORS);

  // If we've had too many errors, mark system as unhealthy
  if (sensorErrorCount >= MAX_SENSOR_ERRORS) {
    systemHealthy = false;
    Serial.println("CRITICAL: Too many sensor errors - system marked unhealthy");
  }
}

// ========== SYSTEM RECOVERY FUNCTION ==========
void attemptSystemRecovery() {
  // Only try recovery every few seconds
  if (millis() - lastErrorRecovery < ERROR_RECOVERY_INTERVAL) {
    return;
  }

  Serial.println("Attempting system recovery...");

  // Try to reinitialize the sensor
  sensor.setTimeout(500);
  if (sensor.init()) {
    configureSensor();
    systemHealthy = true;
    sensorErrorCount = 0;
    Serial.println("SUCCESS: System recovery successful!");
  } else {
    Serial.println("FAILED: System recovery failed, will try again");
  }

  lastErrorRecovery = millis();
}

// ========== DATA OUTPUT FUNCTION ==========
void outputData() {
  // view readings in Serial Plotter
  // Serial.print(currentPosition);
  // Serial.print(",");
  // Serial.print(minPosition);
  // Serial.print(",");
  // Serial.print(maxPosition);
  // Serial.print(",");
  // Serial.print(crankshaftThrow);
  // Serial.print(",");
  // engine parameters
  Serial.print(rpm);
  Serial.print(",");
  Serial.print(torque);
  Serial.print(",");
  Serial.print(horsepower);
  Serial.println();
}
