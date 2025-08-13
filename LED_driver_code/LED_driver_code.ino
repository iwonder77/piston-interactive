/* 
* ----------------------------------------------
* PROJECT NAME: auto_shop_piston_interactive_LED_driver_code
* Description: QuinLED Dig Uno receives PWM signal whose strength is proportional to 
*              the torque or horsepower produced by spinning crankshaft
* 
* Author: Isai Sanchez
* Date: 7-16-25
* Board Used: QuinLED Dig Uno
* Notes:
* ----------------------------------------------
*/

#include <FastLED.h>
#include <RunningAverage.h>

#define NUM_LEDS 47        // 47 LEDs on our strip
#define LED_DATA_PIN 3     // LED strip data pin
#define LED_BRIGHTNESS 20  // LED strip data pin
#define PWM_INPUT_PIN 12   // Pin to read PWM signal from main ESP32
#define PWM_MEASUREMENT_NUM 3

CRGB leds[NUM_LEDS];
const int AVERAGE_SAMPLES = 5;
RunningAverage pwmAverage(AVERAGE_SAMPLES);

// Simple idle animation variables
unsigned long lastActivityTime = 0;
const unsigned long IDLE_TIMEOUT = 3000;  // Start idling after 3 seconds
float wavePhase = 0;

// Core functionality variables
const int LED_HYSTERESIS_THRESHOLD = 2;
int lastNumLEDs = 0;
int currentTargetLEDs = 0;  // The base LED count from potentiometer

void setup() {
  Serial.begin(115200);
  pinMode(PWM_INPUT_PIN, INPUT);
  FastLED.addLeds<WS2815, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255 * LED_BRIGHTNESS / 100);

  pwmAverage.clear();
  lastActivityTime = millis();
}

void loop() {
  unsigned long highTime = pulseIn(PWM_INPUT_PIN, HIGH, 25000);

  if (highTime > 0 && highTime <= 250) {
    pwmAverage.addValue(highTime);
    float avgHighTime = pwmAverage.getAverage();

    // Convert pulse width to LED count
    int pwmValue = map(avgHighTime, 0, 200, 0, 255);
    pwmValue = constrain(pwmValue, 0, 255);
    int targetLEDs = map(pwmValue, 0, 255, 0, NUM_LEDS);
    targetLEDs = constrain(targetLEDs, 0, NUM_LEDS);

    // Check if potentiometer changed significantly
    if (abs(targetLEDs - currentTargetLEDs) >= LED_HYSTERESIS_THRESHOLD) {
      currentTargetLEDs = targetLEDs;
      lastActivityTime = millis();  // Reset idle timer

      Serial.print("Pot changed - Target LEDs: ");
      Serial.println(currentTargetLEDs);
    }
  }

  // Always update display (either normal or with idle animation)
  updateDisplay();

  delay(50);  // Smooth animation timing
}

void updateDisplay() {
  unsigned long currentTime = millis();
  bool isIdling = (currentTime - lastActivityTime) > IDLE_TIMEOUT;

  int displayLEDs = currentTargetLEDs;

  if (isIdling && currentTargetLEDs > 0) {
    // Create breathing/wave effect during idle
    wavePhase += 0.15;  // Speed of breathing animation
    if (wavePhase > TWO_PI) wavePhase = 0;

    // Create wave that oscillates ±15% around the target
    float waveMultiplier = 1.0 + (sin(wavePhase) * 0.15);  // 0.85 to 1.15
    displayLEDs = (int)(currentTargetLEDs * waveMultiplier);
    displayLEDs = constrain(displayLEDs, 1, NUM_LEDS);

    // Add subtle randomness for more organic feel
    if (random(100) < 10) {          // 10% chance each frame
      displayLEDs += random(-1, 2);  // ±1 LED jitter
      displayLEDs = constrain(displayLEDs, 1, NUM_LEDS);
    }
  }

  // Clear all LEDs
  fill_solid(leds, NUM_LEDS, CRGB::Black);

  // Choose color based on state
  CRGB ledColor = isIdling ? CRGB(255, 80, 0) : CRGB::Green;  // Orange for idle, Green for active

  // Create the wave effect you want
  for (int i = 0; i < displayLEDs && i < NUM_LEDS; i++) {
    if (isIdling) {
      // During idle: create a wave intensity that fades toward the edges
      float distanceFromCenter = abs(i - (displayLEDs / 2.0));
      float maxDistance = displayLEDs / 2.0;
      float intensity = 1.0 - (distanceFromCenter / maxDistance * 0.5);  // 50% fade to edges

      // Add wave motion along the strip
      float waveIntensity = sin(wavePhase + (i * 0.3)) * 0.3 + 0.7;  // 0.4 to 1.0
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

  // Debug output
  static unsigned long lastDebug = 0;
  if (currentTime - lastDebug > 500) {  // Every 500ms
    Serial.print(isIdling ? "IDLE - " : "ACTIVE - ");
    Serial.print("Target: ");
    Serial.print(currentTargetLEDs);
    Serial.print(", Display: ");
    Serial.print(displayLEDs);
    if (isIdling) {
      Serial.print(", Wave: ");
      Serial.print(wavePhase, 2);
    }
    Serial.println();
    lastDebug = currentTime;
  }
}
