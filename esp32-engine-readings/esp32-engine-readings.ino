/* 
* ----------------------------------------------
* PROJECT NAME: Piston Interactive
* File Name: engine_readings_w_VL53L1X.ino
* Description: full implementation of engine reading logic with M5Stack's ToF sensor which will be used in the piston interactive
* 
* Author: Isai Sanchez
* Original contributions: Mike Heaton
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

// ===================== CONFIG CONSTANTS =======================================
namespace cfg {
// sensor
constexpr uint16_t SENSOR_TIMEOUT_MS = 500;  // VL53L1X timeout
constexpr uint8_t ROI_W = 6;                 // 4..16 typical
constexpr uint8_t ROI_H = 6;
constexpr uint8_t ROI_CENTER = 199;           // depends on module; 199 is common for M5Stack unit
constexpr uint16_t TIMING_BUDGET_US = 15000;  // 15 ms budget
constexpr uint16_t INTER_MEAS_MS = 5;         // continuous period

// valid sensor reading window (mm)
constexpr float MIN_VALID_MM = 20.0f;    // 4 cm is lowest reading recommended from sensor docs, but let's stick with 2 cm
constexpr float MAX_VALID_MM = 4000.0f;  // 4 m

// motion detection / windows
constexpr float MIN_MOTION_DELTA_MM = 3.0f;        // minimum change in position to detect motion (mm)
constexpr uint32_t MOTION_TIMEOUT_MS = 3000;       // reset tracking after this many ms of no motion
constexpr float MIN_REASONABLE_THROW = 2.0f;       // minimum detected throw that makes sense (mm)
constexpr float MAX_REASONABLE_THROW = 150.0f;     // maximum detected throw that makes sense (mm)
constexpr float MIN_THROW_FOR_MOTION_MM = 2.0f;    // minimum throw to consider system "in motion" (mm)
constexpr float MIN_MAX_HYST_MM = 1.0f;            // must exceed previous min/max by this amount to update
constexpr uint16_t PEAK_MIN_MAX_WINDOW_SIZE = 64;  // max ring buffer size for min/max window
constexpr uint16_t PEAK_PERIOD_WINDOW_SIZE = 6;    // max ring buffer size for period count window
constexpr float INIT_SEED_RPM = 10.0f;             // early feedback RPM before first full cycle

// engine model
constexpr float PISTON_AREA_CM2 = 50.0f;  // WARNING: fixed piston head area, must be different for each installation
constexpr float MIN_DETECTED_RPM = 2.0f;
constexpr float MAX_DETECTED_RPM = 200.0f;
constexpr float MAX_DISPLAY_TORQUE = 1200.0f;
constexpr float MAX_DISPLAY_HP = 200.0f;
constexpr float PRESSURE_MULTIPLIER = 0.5f;  // simplification factor
constexpr float TORQUE_SCALE_FACTOR = 0.2f;  // scale torque for reasonable LED range

// EMA filter constants (0...1) smaller = heavier smoothing
constexpr float EMA_POS_ALPHA = 0.4f;  // position smoothing
constexpr float EMA_RPM_ALPHA = 0.3f;  // rpm smoothing

// LEDC (PWM)
constexpr int TORQUE_LED_PIN = 18;
constexpr int HP_LED_PIN = 19;
constexpr uint32_t PWM_FREQ_HZ = 5000;  // 5kHz PWM frequency
constexpr uint8_t PWM_RES_BITS = 8;     // 8-bit resolution (0...255)
constexpr int PWM_MIN_DUTY = 20;        // allow small readings for torque and hp to pass through
//
// error handling
constexpr uint8_t MAX_SENSOR_ERRORS = 5;
constexpr uint32_t ERROR_RECOVERY_PERIOD = 5000;
}

// ===================== UTILS =====================
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi
                                  : x;
}

// lightweight ring buffer helper
class RingWindow {
public:
  explicit RingWindow(size_t cap) : cap_(cap), buf_(new float[cap]), filled_(0), head_(0) {
    for (size_t i = 0; i < cap_; ++i) buf_[i] = NAN;
  }
  ~RingWindow() {
    delete[] buf_;
  }

  // reset buffer to empty state (mark every slot as NAN)
  void clear() {
    filled_ = 0;
    head_ = 0;
    for (size_t i = 0; i < cap_; ++i) buf_[i] = NAN;
  }

  // write new sample to buffer
  void push(float v) {
    buf_[head_] = v;                // write sample to head
    head_ = (head_ + 1) % cap_;     // advance head (modulo % ensures we circle back to 0 when we hit cap)
    if (filled_ < cap_) ++filled_;  // increment filled until buffer is full (then filled equals cap)
  }

  // checks if any samples have been written to buffer
  bool valid() const {
    return filled_ > 0;
  }

  // return min
  float min() const {
    // skip NAN entries, scan through only first filled items
    float m = INFINITY;
    for (size_t i = 0; i < filled_; ++i)
      if (!isnan(buf_[i]) && buf_[i] < m) m = buf_[i];
    return (m == INFINITY) ? NAN : m;
  }

  // return max
  float max() const {
    // skip NAN entries, scan through only first filled items
    float m = -INFINITY;
    for (size_t i = 0; i < filled_; ++i)
      if (!isnan(buf_[i]) && buf_[i] > m) m = buf_[i];
    return (m == -INFINITY) ? NAN : m;
  }

  // return average
  float average() const {
    if (filled_ == 0) return 0.0f;
    float sum = 0.0f;
    for (size_t i = 0; i < filled_; ++i) sum += buf_[i];
    return sum / float(filled_);
  }

private:
  size_t cap_;     // fixed ring buffer capacity
  float* buf_;     // dynamically allocated float array (of length cap) to store samples
  size_t filled_;  // slots in buffer currently filled with valid data
  size_t head_;    // index where the NEXT write goes to
};
// =================================================

// ==================== SENSOR ====================
class ToFSensor {
public:
  // sensor configuration
  bool configure() {
    Wire.begin();

    // see Pololu's VL53L1X github repo for more inforamtion on these config methods
    tof.setTimeout(cfg::SENSOR_TIMEOUT_MS);                 // set timeout
    if (!tof.init()) return false;                          // ensure init
    tof.setROISize(cfg::ROI_W, cfg::ROI_H);                 // set the region of interest (ROI) to 6x6 pixels (smaller ROI = narrower FoV = better accuracy, less noise)
    tof.setROICenter(cfg::ROI_CENTER);                      // set the center of the sensor's ROI
    tof.setDistanceMode(VL53L1X::Short);                    // set the distance mode to short (available are Short, Medium, Long)
    tof.setMeasurementTimingBudget(cfg::TIMING_BUDGET_US);  // measurement timing budget
    tof.startContinuous(cfg::INTER_MEAS_MS);                // the specified inter-measurement period in milliseconds determines how often the sensor takes a measurement
    return true;
  }

  // we must check Pololu's dataReady() before reading sensor data
  bool ready() {
    return tof.dataReady();
  }

  // Returns true when reading was successful, and modifies position variable that was passed by reference
  bool read(float& pos) {
    uint16_t rawReading = tof.read();                          // Pololu read() clears data ready flag, and returns a uint16_t type
    if (rawReading == 0 || rawReading == 65535) return false;  // timeout or comm error
    float f = static_cast<float>(rawReading);
    if (f < cfg::MIN_VALID_MM || f > cfg::MAX_VALID_MM) return false;

    // EMA smoothing
    if (firstReading) {
      currentPosition = f;
      firstReading = false;
    } else {
      currentPosition = currentPosition + cfg::EMA_POS_ALPHA * (f - currentPosition);
    }
    pos = currentPosition;
    lastSensorRead = millis();
    return true;
  }

  // get timestamp of last sensor read, allows us to detect stale sensors
  uint32_t getLastSensorRead() const {
    return lastSensorRead;
  }

private:
  VL53L1X tof;
  bool firstReading = true;
  float currentPosition = 1.0f;
  uint32_t lastSensorRead = 0;
};
// ================================================

// ================ MOTION TRACKER ================
class MotionTracker {
public:
  MotionTracker() : window(cfg::PEAK_MIN_MAX_WINDOW_SIZE), periodWindow(cfg::PEAK_PERIOD_WINDOW_SIZE) {}

  void reset(float pos) {
    periodWindow.clear();
    window.clear();
    moving = false;
    lastPos = pos;
    maxPos = pos;
    minPos = pos;
    crankshaftThrow = 0.0f;
    lastZeroCrossMs = 0;
    lastRPMUpdateMs = 0;
    rpm_init = false;
    rpm = 0.0f;
    lastEdge = Edge::UNKNOWN;
  }

  void update(float pos) {
    uint32_t now = millis();
    float d = fabsf(pos - lastPos);

    if (d > cfg::MIN_MOTION_DELTA_MM) {
      if (!moving) {
        moving = true;
        Serial.println("Motion detected — tracking");
      }
      lastMotionMs = now;

      // update ring window and hysteretic min/max
      window.push(pos);
      float wMax = window.max();
      float wMin = window.min();
      // hysteretic min/max updates
      if (!isnan(wMax) && wMax > maxPos + cfg::MIN_MAX_HYST_MM) { maxPos = wMax; }
      if (!isnan(wMin) && wMin < minPos - cfg::MIN_MAX_HYST_MM) { minPos = wMin; }

      // throw (radius) ~ (max-min)/2
      float rawThrow = (maxPos - minPos) * 0.5f;
      if (rawThrow < cfg::MIN_REASONABLE_THROW) {
        crankshaftThrow = 0.0f;
        throwValid = false;
      } else if (rawThrow > cfg::MAX_REASONABLE_THROW) {
        // likely bogus — reset window around current pos
        maxPos = pos;
        minPos = pos;
        window.clear();
        crankshaftThrow = 0;
        throwValid = false;
      } else {
        crankshaftThrow = rawThrow;
        throwValid = (crankshaftThrow >= cfg::MIN_THROW_FOR_MOTION_MM);
      }

      // center crossing RPM estimator (two crossings = full cycle) with hysteresis band
      float center = (maxPos + minPos) * 0.5f;
      const float hysteresisBand = cfg::MIN_MAX_HYST_MM * 0.5f;
      Edge currEdge = Edge::UNKNOWN;
      if (pos > center + hysteresisBand) {
        currEdge = Edge::ABOVE;
      } else if (pos < center - hysteresisBand) {
        currEdge = Edge::BELOW;
      }
      // if we're still inside band, keep previous edge (no new crossing)
      // now check if a cross happened with the following condition
      if (currEdge != Edge::UNKNOWN && currEdge != lastEdge && throwValid) {
        if (lastZeroCrossMs != 0) {
          // a half cycle occurs when we hit the second center cross timestamp, calculate
          // that time interval here
          uint32_t half = now - lastZeroCrossMs;
          // reject unreasonable calcs
          if (half > 10 && half < 10000) {
            uint32_t fullPeriod = half * 2;
            periodWindow.push(fullPeriod);
            float avgPeriod = periodWindow.average();
            if (avgPeriod > 0.0f) {
              // calculate raw RPM value from period, sanity check, and apply EMA filtering
              float rawRPM = 60000.0f / avgPeriod;
              if (rawRPM > cfg::MIN_DETECTED_RPM && rawRPM < cfg::MAX_DETECTED_RPM) {
                if (!rpm_init) {
                  rpm = rawRPM;
                  rpm_init = true;
                } else {
                  rpm = rpm + cfg::EMA_RPM_ALPHA * (rawRPM - rpm);
                }
                lastRPMUpdateMs = now;
              }
            }
          }
        }
        lastZeroCrossMs = now;
        lastEdge = currEdge;
      }
    }

    // Motion timeout → reset
    if (moving && (now - lastMotionMs > cfg::MOTION_TIMEOUT_MS)) {
      Serial.println("Motion timeout — reset");
      reset(pos);
    }

    lastPos = pos;
  }

  bool isMoving() const {
    return moving && throwValid && crankshaftThrow >= cfg::MIN_THROW_FOR_MOTION_MM;
  }
  float getCrankshaftThrowMM() const {
    return crankshaftThrow;
  }
  float getRPMs() const {
    return rpm;
  }
  uint32_t lastRPMms() const {
    return lastRPMUpdateMs;
  }

  void decayRPM() {
    // gentle decay if no fresh update (kid slows down spinning)
    uint32_t now = millis();
    if (rpm > 0 && (now - lastRPMUpdateMs > 1000)) {
      rpm *= 0.99f;
      if (rpm < 10.0f) rpm = 0.0f;
    }
  }

private:
  enum class Edge : uint8_t { UNKNOWN,
                              ABOVE,
                              BELOW };

  // --- period buffer for averaging ---
  RingWindow periodWindow;

  RingWindow window;
  bool moving = false;
  bool throwValid = false;
  float lastPos = 0;
  float maxPos = 0;
  float minPos = 0;
  float crankshaftThrow = 0;
  uint32_t lastMotionMs = 0;
  uint32_t lastZeroCrossMs = 0;
  uint32_t lastRPMUpdateMs = 0;
  float rpm = 0;
  bool rpm_init = false;
  Edge lastEdge = Edge::UNKNOWN;
};
// ====================================================

// =================== ENGINE MODEL ===================
struct EngineReadout {
  float rpm = 0;
  float torque = 0;
  float hp = 0;
};

class EngineModel {
public:
  EngineReadout compute(float throwMM, float rpm) {
    EngineReadout r;
    r.rpm = clampf(rpm, 0.0f, cfg::MAX_DETECTED_RPM);
    if (r.rpm <= 0 || throwMM < cfg::MIN_THROW_FOR_MOTION_MM) {
      r.torque = 0;
      r.hp = 0;
      return r;
    }

    // simplified pressure estimate ∝ RPM^2
    float pressure = (r.rpm * r.rpm) / 10000.0f * cfg::PRESSURE_MULTIPLIER;
    float torque = throwMM * cfg::PISTON_AREA_CM2 * pressure * cfg::TORQUE_SCALE_FACTOR;
    torque = clampf(torque, 0.0f, cfg::MAX_DISPLAY_TORQUE);
    float hp = (torque * r.rpm) / 1000.0f;
    hp = clampf(hp, 0.0f, cfg::MAX_DISPLAY_HP);

    r.torque = torque;
    r.hp = hp;
    return r;
  }
};
// ====================================================

// ==================== LED OUTPUT ====================
class LedOutput {
public:
  bool beginPWM() {
    pinMode(cfg::TORQUE_LED_PIN, OUTPUT);
    pinMode(cfg::HP_LED_PIN, OUTPUT);
    bool torqueSuccess = ledcAttach(cfg::TORQUE_LED_PIN, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS);
    bool hpSuccess = ledcAttach(cfg::HP_LED_PIN, cfg::PWM_FREQ_HZ, cfg::PWM_RES_BITS);
    return torqueSuccess && hpSuccess;
  }

  void show(float torque, float hp) {
    uint32_t maxDuty = (1u << cfg::PWM_RES_BITS) - 1u;  // 255
    int t = mapFloatToDuty(torque, cfg::MAX_DISPLAY_TORQUE, cfg::PWM_MIN_DUTY, maxDuty);
    int h = mapFloatToDuty(hp, cfg::MAX_DISPLAY_HP, cfg::PWM_MIN_DUTY, maxDuty);
    ledcWrite(cfg::TORQUE_LED_PIN, t);
    ledcWrite(cfg::HP_LED_PIN, h);
  }

private:
  static int mapFloatToDuty(float v, float vmax, uint32_t minDuty, uint32_t maxDuty) {
    v = clampf(v, 0.0f, vmax);
    if (v <= 0.0f) return 0;

    // Map 0..vmax → PWM_MIN_DUTY..maxDuty
    int duty = int((v / vmax) * float(maxDuty - cfg::PWM_MIN_DUTY) + cfg::PWM_MIN_DUTY + 0.5f);

    return duty;
  }
};
// ====================================================

// ======= HEALTH / Finite-State Machine (FSM) ========
enum class AppState : uint8_t { INIT,
                                IDLE,
                                TRACKING,
                                ERROR_RECOVERY };
// lighteweight WATCHDOG
class Health {
public:
  void onBadRead() {
    if (errCount < 255) ++errCount;
    if (errCount >= cfg::MAX_SENSOR_ERRORS) unhealthy = true;
  }
  void onGoodRead() {
    errCount = 0;
  }

  bool healthy() const {
    return !unhealthy;
  }
  void markHealthy() {
    unhealthy = false;
    errCount = 0;
  }

  bool allowRecovery() {
    uint32_t now = millis();
    if (now - lastRecoveryTime >= cfg::ERROR_RECOVERY_PERIOD) {
      lastRecoveryTime = now;
      return true;
    }
    return false;
  }

private:
  uint8_t errCount = 0;
  bool unhealthy = false;
  uint32_t lastRecoveryTime = 0;
};
// ====================================================

// ======================= APP ========================
class App {
public:
  void setup() {
    Serial.begin(115200);
    Serial.println("Pos(mm),Throw(mm),RPM,Torque,HP");

    if (!sensor.configure()) {
      Serial.println("Sensor init failed");
      state = AppState::ERROR_RECOVERY;
      return;
    }
    if (!led.beginPWM()) {
      Serial.println("LED PWM init failed");
      state = AppState::ERROR_RECOVERY;
      return;
    }

    // prime the motion tracker with a first reading if available
    float pos;
    if (sensor.read(pos)) {
      tracker.reset(pos);
    } else {
      tracker.reset(0);
    }
    state = AppState::IDLE;
  }

  // runs in main loop (FINITE STATE MACHINE)
  void loopOnce() {
    switch (state) {
      case AppState::IDLE:
      case AppState::TRACKING:
        run();
        break;
      case AppState::ERROR_RECOVERY:
        recover();
        break;
      default:
        break;
    }
  }

private:
  void run() {
    // temporary "scratchpad" for fresh position reading
    float pos;
    bool ok = false;

    // read sensor only if data is ready
    if (sensor.ready()) {
      ok = sensor.read(pos);
      // notify watchdog of reading success/failure
      if (!ok) {
        health.onBadRead();
      } else {
        health.onGoodRead();
      }
    }

    // bail immediately into recovery state if health check fails
    if (!health.healthy()) {
      state = AppState::ERROR_RECOVERY;
      return;
    }

    if (ok) {
      // now that the position was read successfully, update motion + RPM estimator
      tracker.update(pos);
      tracker.decayRPM();

      // compute outputs
      EngineReadout r = engine.compute(tracker.getCrankshaftThrowMM(), tracker.getRPMs());
      led.show(r.torque, r.hp);

      // CSV for Serial Plotter
      Serial.print(pos, 2);
      Serial.print(',');
      Serial.print(tracker.getCrankshaftThrowMM(), 2);
      Serial.print(',');
      Serial.print(r.rpm, 2);
      Serial.print(',');
      Serial.print(r.torque, 2);
      Serial.print(',');
      Serial.println(r.hp, 2);

      state = tracker.isMoving() ? AppState::TRACKING : AppState::IDLE;
    }
  }

  void recover() {
    if (!health.allowRecovery()) return;  // wait between attempts
    Serial.println("Attempting recovery...");

    // Re-init sensor
    ToFSensor newSensor;
    if (newSensor.configure()) {
      sensor = newSensor;
      health.markHealthy();
      // reseed tracker
      float pos;
      if (sensor.read(pos)) tracker.reset(pos);
      else tracker.reset(0);
      Serial.println("Recovery OK");
      state = AppState::IDLE;
    } else {
      Serial.println("Recovery failed");
    }
  }

  // Components
  ToFSensor sensor;
  MotionTracker tracker;
  EngineModel engine;
  LedOutput led;
  Health health;
  AppState state = AppState::INIT;
};
// ====================================================

// ==================== SKETCH ========================
App app;

void setup() {
  app.setup();
}

void loop() {
  app.loopOnce();
  delay(2);  // light pacing, sensor runs in continuous mode
}
