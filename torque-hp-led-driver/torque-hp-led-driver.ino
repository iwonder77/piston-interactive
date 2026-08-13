/* 
* ----------------------------------------------
* PROJECT NAME: Piston Interactive DigUno LED Driver code
* Description: QuinLED Dig Uno receives PWM signal whose strength is proportional to 
*              the torque or horsepower produced by spinning crankshaft, lights LED strip accordingly
* 
* Author: Isai Sanchez
* Date: 7-16-25
* Board Used: QuinLED Dig Uno
* Notes:
* ----------------------------------------------
*/

#include <FastLED.h>
#include <RunningAverage.h>
#include <esp_task_wdt.h>

#include "src/Config.h"
#include "src/Debug.h"

// ----- TIMER INTERRUPT VARIABLES FOR PWM CALC -----
volatile unsigned long pulse_start_time = 0;
volatile unsigned long pulse_width = 0;
volatile unsigned long last_period_start = 0;
volatile unsigned long period_length = 0;
volatile bool new_data_available = false;

// ----- LED SETUP -----
CRGB leds[config::NUM_LEDS];

// ----- RUNNING AVG FILTER -----
RunningAverage dutyCycleFilter(config::AVERAGE_SAMPLES);

// ----- IDLE ANIMATION VARIABLES -----
unsigned long lastActivityTime = 0;
float wavePhase = 0;

// ----- CORE FUNC VARIABLES -----
int currentTargetLEDs = 0;  // The base LED count from PWM calc

// ========== INTERRUPT FUNCTION ==========
void IRAM_ATTR pwmInterrupt() {
  unsigned long current_time = micros();
  int pinState = digitalRead(config::PWM_INPUT_PIN);

  switch (pinState) {
    // rising edge (start of a new pulse)
    case HIGH:
      // calculate period (time between consecutive rising edges)
      if (last_period_start != 0) {
        period_length = current_time - last_period_start;
      }
      last_period_start = current_time;
      pulse_start_time = current_time;
      break;
    // falling edge, record pulse width time
    case LOW:
      pulse_width = current_time - pulse_start_time;
      new_data_available = true;  // signal main loop that data is ready
      break;
  };
}

// ========== LED STRIP UPDATE FUNCTION ==========
void updateDisplay() {
  unsigned long currentTime = millis();
  bool isIdling = (currentTime - lastActivityTime) > config::IDLE_TIMEOUT_MS;

  int displayLEDs = currentTargetLEDs;

  if (isIdling && currentTargetLEDs > 0) {
    // Create breathing/wave effect during idle
    wavePhase += config::WAVE_PHASE_STEP;
    if (wavePhase > TWO_PI) wavePhase = 0;

    // Create wave that oscillates ±25% around the target
    float waveMultiplier = 1.0 + (sin(wavePhase) * config::WAVE_AMPLITUDE);
    displayLEDs = (int)(config::NUM_ANIMATION_LEDS * waveMultiplier);
    displayLEDs = constrain(displayLEDs, 1, config::NUM_LEDS);

    // Add subtle randomness for more organic feel
    if (random(config::PERCENT_MAX) < config::JITTER_CHANCE_PCT) {
      displayLEDs += random(-1, 2);  // ±1 LED jitter
      displayLEDs = constrain(displayLEDs, 1, config::NUM_LEDS);
    }
  }

  // Clear all LEDs
  fill_solid(leds, config::NUM_LEDS, CRGB::Black);

  // Choose color based on state
  CRGB ledColor = isIdling ? CRGB(config::IDLE_COLOR_R, config::IDLE_COLOR_G,
                                  config::IDLE_COLOR_B)
                           : CRGB::Green;

  // Create the wave effect you want
  for (int i = 0; i < displayLEDs && i < config::NUM_LEDS; i++) {
    if (isIdling) {
      // During idle: create a wave intensity that fades toward the edges
      float distanceFromCenter = abs(i - (displayLEDs / 2.0));
      float maxDistance = displayLEDs / 2.0;
      float intensity = 1.0 - (distanceFromCenter / maxDistance * config::EDGE_FADE);

      // Add wave motion along the strip
      float waveIntensity = sin(wavePhase + (i * config::WAVE_SPATIAL_FREQ)) *
                                config::WAVE_INTENSITY_SCALE +
                            config::WAVE_INTENSITY_OFFSET;
      intensity *= waveIntensity;

      leds[i] = CRGB(
        (int)(ledColor.r * intensity),
        (int)(ledColor.g * intensity),
        (int)(ledColor.b * intensity));
    } else {
      // Normal operation: solid color
      leds[i] = ledColor;
    }
  }

  FastLED.show();
}


void setup() {
  Serial.begin(115200);
  pinMode(config::PWM_INPUT_PIN, INPUT_PULLUP);
  FastLED.addLeds<WS2815, config::LED_DATA_PIN, RGB>(leds, config::NUM_LEDS);
  FastLED.setBrightness(config::LED_BRIGHTNESS);

  dutyCycleFilter.clear();
  lastActivityTime = millis();

  attachInterrupt(digitalPinToInterrupt(config::PWM_INPUT_PIN), pwmInterrupt,
                  CHANGE);

  currentTargetLEDs = config::INITIAL_TARGET_LEDS;

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
  // make sure high time reading is valid
  if (new_data_available) {
    noInterrupts();
    unsigned long safe_pulse_width = pulse_width;
    unsigned long safe_period = period_length;
    new_data_available = false;
    interrupts();
    if (safe_period >= config::MIN_VALID_PERIOD_US) {
      float duty_cycle =
          (float)safe_pulse_width / safe_period * config::PERCENT_MAX;
      duty_cycle = constrain(duty_cycle, 0, config::PERCENT_MAX);
      dutyCycleFilter.addValue(duty_cycle);
      float smoothedDutyCycle = dutyCycleFilter.getAverage();
      int targetLEDs = map(smoothedDutyCycle, 0, config::PERCENT_MAX, 0,
                           config::NUM_LEDS - 1);
      targetLEDs = constrain(targetLEDs, 0, config::NUM_LEDS - 1);

      // check if target LEDs changed significantly
      if (abs(targetLEDs - currentTargetLEDs) >=
          config::LED_HYSTERESIS_THRESHOLD) {
        currentTargetLEDs = targetLEDs;
        lastActivityTime = millis();  // Reset idle timer
      }
    }
  }
  // always update display (either normal or with idle animation)
  updateDisplay();

  // TODO: replace with non-blocking frame timing (styleguide 6) when the
  // animation moves into its own class
  delay(config::FRAME_INTERVAL_MS);
}
