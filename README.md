# Piston Interactive

## Project Overview
This document provides a comprehensive breakdown of the Arduino sketch flow for the Piston Interactive system, including the main ESP32 code for sensor readings and engine parameter calculations, and the QuinLED driver code for LED bar visualization.

## Hardware Components
- Microcontroller: ESP32-DevKitC-V4
- ToF Sensor: M5Stack's VL53L1X ToF Distance Unit. Found [here](https://shop.m5stack.com/products/time-of-flight-distance-unit-vl53l1x?srsltid=AfmBOoprDGgPCZlY4ets509p4m7cXj-nKKdMHRDG5hY20O5jZdbu7gsj)

---


## ToF Sensor Initialization
```cpp
sensor.setTimeout(500);
if (!sensor.init()) {
    systemHealthy = false;         // Mark system as unhealthy
} else {
    configureSensor();             // Apply sensor configuration
}
```

**Sensor Configuration Applied:**
- ROI Size: 6x6 pixels (narrower field of view for accuracy)
- ROI Center: 199 (sensor center point)
- Distance Mode: Short (4cm to 130cm range)
- Timing Budget: 15ms per measurement
- Continuous Mode: 5ms intervals

### 1.4 Filter Initialization
- **Sample Filter**: 10-sample window for noise reduction
- **Min/Max Window**: 50-sample window for extremum tracking  
- **RPM Filter**: 10-sample window for RPM smoothing
- All filters cleared to start with clean state

### 1.5 Motion System Initialization
```cpp
lastMotionTime = millis();         // Initialize motion timeout tracking
```

---

## 2. MAIN LOOP EXECUTION

### 2.1 System Health Check
```
IF system is NOT healthy:
    → Attempt recovery every 5 seconds
    → Try to reinitialize sensor
    → If successful: mark healthy, clear errors
    → If failed: continue trying
    → Return to start of loop
```

### 2.2 Sensor Data Acquisition
```
IF sensor.dataReady():
    → Proceed to reading
ELSE:
    → Wait 10ms and check again
```

---

## 3. DATA PROCESSING PIPELINE

### 3.1 Raw Reading & Validation
```cpp
float rawReading = sensor.read();
```

**Validation Checks:**
1. **Range Check**: 40mm ≤ reading ≤ 4000mm
2. **Error Values**: Reject 65535 or 0 (sensor errors)
3. **Timeout Check**: Detect communication failures

**Error Handling:**
- Increment error counter on invalid reading
- Mark system unhealthy after 5 consecutive errors
- Continue to next loop iteration on error

### 3.2 Signal Filtering
```cpp
sampleFilter.add(rawReading);
currentPosition = sampleFilter.getAverage();
```
- **Purpose**: Reduce sensor noise and jitter
- **Method**: 10-sample rolling average
- **Output**: Smoothed position value in mm

---

## 4. MOTION DETECTION SYSTEM

### 4.1 Motion Change Detection
```cpp
float positionChange = abs(currentPosition - lastPosition);
if (positionChange > MIN_MOTION_THRESHOLD) {  // 2.0mm threshold
    // Motion detected
}
```

### 4.2 Motion State Management
**First Motion Detection:**
```cpp
if (!isMoving) {
    isMoving = true;
    samplesInCurrentMotion = 0;    // Reset sample counter
    throwIsValid = false;          // Reset validity flag
}
```

**Ongoing Motion Tracking:**
```cpp
lastMotionTime = millis();                    // Update motion timestamp
totalMotionDistance += positionChange;        // Accumulate total distance
samplesInCurrentMotion++;                     // Increment sample count

if (samplesInCurrentMotion >= 20) {           // Minimum samples for validity
    throwIsValid = true;
}
```

### 4.3 Motion Timeout Handling
```cpp
if (isMoving && (millis() - lastMotionTime > 3000)) {  // 3 second timeout
    stopMotionTracking();          // Reset all motion variables
}
```

---

## 5. MIN/MAX TRACKING SYSTEM

### 5.1 Window-Based Tracking
```cpp
minMaxWindow.add(currentPosition);           // Add to tracking window
float windowMax = minMaxWindow.getMaxInBuffer();
float windowMin = minMaxWindow.getMinInBuffer();
```

### 5.2 Hysteresis Application
**Maximum Update:**
```cpp
if (windowMax > maxPosition + 1.0) {         // 1mm hysteresis
    maxPosition = windowMax;
    Serial.println("New MAX detected");
}
```

**Minimum Update:**
```cpp
if (windowMin < minPosition - 1.0) {         // 1mm hysteresis  
    minPosition = windowMin;
    Serial.println("New MIN detected");
}
```

**Purpose of Hysteresis:**
- Prevents oscillation around boundary values
- Requires significant change to update min/max
- Reduces noise impact on throw calculations

---

## 6. RPM CALCULATION SYSTEM

### 6.1 Zero-Crossing Detection
```cpp
float centerPosition = (maxPosition + minPosition) / 2.0;
bool crossedCenter = (lastPosition - centerPosition) * (currentPosition - centerPosition) < 0;
```

### 6.2 Cycle Timing
```cpp
if (crossedCenter && crankshaftThrow > 5.0) {        // Only if sufficient throw
    unsigned long currentTime = millis();
    if (lastZeroCrossing > 0) {
        unsigned long halfCycle = currentTime - lastZeroCrossing;
        cyclePeriod = halfCycle * 2;                  // Full cycle = 2 half-cycles
        
        if (cyclePeriod > 0 && cyclePeriod < 10000) { // Reasonable cycle time
            float rawRPM = 60000.0 / cyclePeriod;    // Convert ms to RPM
            
            if (rawRPM > 5 && rawRPM < 200) {        // Reasonable RPM range
                rpmFilter.add(rawRPM);
                rpm = rpmFilter.getAverage();
            }
        }
    }
    lastZeroCrossing = currentTime;
}
```

### 6.3 RPM Decay System
```cpp
unsigned long timeSinceRPMUpdate = millis() - lastRPMUpdate;
if (timeSinceRPMUpdate > 1000) {                     // 1 second since last update
    rpm = rpm * 0.99;                                // 1% decay per cycle
    
    if (rpm < 10) {                                  // Clear if very low
        rpm = 0.0;
        rpmFilter.clear();
    }
}
```

---

## 7. THROW VALIDATION SYSTEM

### 7.1 Throw Calculation
```cpp
float rawThrow = (maxPosition - minPosition) / 2.0;  // Half of total oscillation
```

### 7.2 Validation Logic
```cpp
if (rawThrow < 3.0) {                    // Too small - likely noise
    crankshaftThrow = 0.0;
    throwIsValid = false;
} else if (rawThrow > 150.0) {           // Too large - likely error
    resetMinMaxTracking();                // Reset and start over
    crankshaftThrow = 0.0;
    throwIsValid = false;  
} else {
    crankshaftThrow = rawThrow;          // Valid throw
}
```

---

## 8. ENGINE PARAMETER CALCULATIONS

### 8.1 Calculation Prerequisites
```cpp
if (!isMoving || !throwIsValid || crankshaftThrow < 5.0) {
    // Set all parameters to zero
    rpm = torque = horsepower = 0.0;
    return;
}
```

### 8.2 Torque Calculation
```cpp
float pressureEstimate = (rpm * rpm) / 10000.0;     // RPM² as pressure proxy
torque = crankshaftThrow * PISTON_AREA * pressureEstimate * TORQUE_SCALE_FACTOR;
torque = constrain(torque, 0, MAX_DISPLAY_TORQUE);  // Clamp to display range
```

**Formula Breakdown:**
- **Throw**: Crankshaft throw distance (mm)
- **Piston Area**: Fixed at 50mm² (configurable constant)
- **Pressure Estimate**: RPM² scaled down (higher RPM = higher forces)
- **Scale Factor**: 0.2 (adjusts output to reasonable LED range)

### 8.3 Horsepower Calculation
```cpp
if (rpm > 0) {
    horsepower = (torque * rpm) / 1000.0;           // Simplified HP formula
} else {
    horsepower = 0.0;
}
horsepower = constrain(horsepower, 0, MAX_DISPLAY_HP);
```

---

## 9. LED OUTPUT SYSTEM

### 9.1 Value Mapping
```cpp
int torquePWM = map(torque, 0, MAX_DISPLAY_TORQUE, 0, 255);    // 0-1200 → 0-255
int hpPWM = map(horsepower, 0, MAX_DISPLAY_HP, 0, 255);        // 0-200 → 0-255
```

### 9.2 PWM Constraining
```cpp
torquePWM = constrain(torquePWM, 20, 255);         // Minimum 20 for visibility
hpPWM = constrain(hpPWM, 20, 255);                 // Maximum 255 for full brightness
```

### 9.3 Hardware Output
```cpp
ledcWrite(TORQUE_LED_PIN, torquePWM);              // Pin 18
ledcWrite(HP_LED_PIN, hpPWM);                      // Pin 19
```

---

## 10. DATA OUTPUT & TIMING

### 10.1 Serial Output Format
```
CSV Header: "Position,Min,Max,Throw,RPM,Torque,HP"
Data Line:  "crankshaftThrow,rpm,torque,horsepower"
```

### 10.2 Loop Timing
- **Main Loop Delay**: 10ms (100Hz execution rate)
- **Sensor Continuous Mode**: 5ms measurement intervals
- **Motion Timeout**: 3000ms of inactivity
- **Error Recovery**: Attempt every 5000ms when unhealthy

---

## 11. ERROR HANDLING & RECOVERY

### 11.1 Sensor Error Escalation
```
1. Invalid reading detected
2. Increment error counter (max 5)
3. Continue operation if under threshold
4. Mark system unhealthy if threshold exceeded
5. Attempt recovery every 5 seconds
6. Reinitialize sensor and reconfigure
7. Clear error counter on successful recovery
```

### 11.2 Motion System Resets
**Triggers for Reset:**
- Motion timeout (3 seconds of inactivity)  
- Throw value exceeds reasonable bounds (>150mm)
- System marked unhealthy

**Reset Actions:**
- Clear all motion flags and counters
- Reset min/max positions to current position
- Clear all filters (sample, min/max, RPM)
- Zero all engine parameters
- Reset timing variables

---

## 12. KEY DESIGN PATTERNS IN ORIGINAL CODE

### 12.1 State Management
- **Global Variables**: All state stored in global scope
- **Flag-Based Logic**: Boolean flags for system states
- **Counter-Based Validation**: Sample counting for throw validity

### 12.2 Error Resilience  
- **Graceful Degradation**: System continues with reduced functionality
- **Automatic Recovery**: Self-healing sensor communication
- **Timeout Protection**: Prevents stuck states

### 12.3 Signal Processing
- **Multi-Stage Filtering**: Raw → Sample Filter → Min/Max Window
- **Hysteresis**: Prevents boundary oscillation
- **Validation Gates**: Multiple checks before accepting data

### 12.4 Performance Considerations
- **Non-Blocking Delays**: Uses millis() for timing
- **Efficient Filters**: RunningAverage class for O(1) operations  
- **Constrained Calculations**: Prevents overflow/underflow

This comprehensive flow ensures robust operation in the interactive children's environment while providing accurate piston simulation feedback.
