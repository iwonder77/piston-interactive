#include "BarDisplay.h"

bool BarDisplay::init() {
  FastLED.addLeds<WS2815, config::LED_DATA_PIN, RGB>(leds_, config::NUM_LEDS);
  FastLED.setBrightness(config::LED_BRIGHTNESS);
  fill_solid(leds_, config::NUM_LEDS, CRGB::Black);
  FastLED.show();
  return true;
}

void BarDisplay::update(uint16_t count, bool is_idling) {
  if (is_idling) {
    // NOTE: advance the phase here, once per frame. Keeping it inside the
    // render means a caller cannot double-step or freeze the animation by
    // getting the call order wrong.
    wave_phase_ += config::WAVE_PHASE_STEP;
    if (wave_phase_ > config::WAVE_PHASE_WRAP) {
      wave_phase_ = 0.0f;
    }
    renderIdle();
  } else {
    renderSolid(count);
  }
  FastLED.show();
}

void BarDisplay::renderSolid(uint16_t count) {
  if (count > config::NUM_LEDS) {
    count = config::NUM_LEDS;
  }
  const CRGB color(config::ACTIVE_COLOR_R, config::ACTIVE_COLOR_G,
                   config::ACTIVE_COLOR_B);
  for (uint16_t i = 0; i < count; ++i) {
    leds_[i] = color;
  }
  for (uint16_t i = count; i < config::NUM_LEDS; ++i) {
    leds_[i] = CRGB::Black;
  }
}

void BarDisplay::renderIdle() {
  // global breath: 1.0 at the peak, (1 - 2 * WAVE_AMPLITUDE) at the trough.
  // NOTE: hoisted out of the loop - it does not depend on the LED index, so
  // computing it per pixel would repeat the same sinf() NUM_LEDS times a frame.
  const float breath = 1.0f - config::WAVE_AMPLITUDE +
                       sinf(wave_phase_) * config::WAVE_AMPLITUDE;

  for (uint16_t i = 0; i < config::NUM_LEDS; ++i) {
    const float intensity = intensityAt(i, breath);
    leds_[i] = CRGB(static_cast<uint8_t>(config::IDLE_COLOR_R * intensity),
                    static_cast<uint8_t>(config::IDLE_COLOR_G * intensity),
                    static_cast<uint8_t>(config::IDLE_COLOR_B * intensity));
  }
}

float BarDisplay::intensityAt(uint16_t index, float breath) const {
  // fade toward both ends of the strip, brightest in the middle
  const float distance_from_center = fabsf(index - config::HALF_STRIP_LEDS);
  const float edge = 1.0f - (distance_from_center / config::HALF_STRIP_LEDS *
                             config::EDGE_FADE);

  // wave travelling along the strip
  const float wave = sinf(wave_phase_ + (index * config::WAVE_SPATIAL_FREQ)) *
                         config::WAVE_INTENSITY_SCALE +
                     config::WAVE_INTENSITY_OFFSET;

  // NOTE: clamp because the three terms multiply - tuning the constants above
  // can push the product past 1.0, which would wrap the uint8_t cast
  return config::clampf(edge * wave * breath, 0.0f, 1.0f);
}
