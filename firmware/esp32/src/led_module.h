#pragma once
// =============================================================================
// LED Module — WS2812B NeoPixel control via FastLED
// =============================================================================
// Ported from pi_nodes/nodes/led_node.py + pi_nodes/hardware/led_driver.py
// Subscribes: samurai/{robot_id}/led/command
// Publishes:  samurai/{robot_id}/led/state (retained, on change)

#include <FastLED.h>
#include <ArduinoJson.h>
#include "config.h"

// Animation mode enum
enum LedMode {
    LED_OFF, LED_SOLID, LED_BLINK, LED_PULSE, LED_RAINBOW, LED_POLICE
};

// Named colors (matches Python COLORS dict)
struct NamedColor { const char* name; CRGB color; };
static const NamedColor NAMED_COLORS[] = {
    {"red",     CRGB(255, 0, 0)},
    {"green",   CRGB(0, 255, 0)},
    {"blue",    CRGB(0, 0, 255)},
    {"yellow",  CRGB(255, 255, 0)},
    {"cyan",    CRGB(0, 255, 255)},
    {"magenta", CRGB(255, 0, 255)},
    {"white",   CRGB(255, 255, 255)},
    {"orange",  CRGB(255, 80, 0)},
    {"off",     CRGB(0, 0, 0)},
};
#define NUM_NAMED_COLORS (sizeof(NAMED_COLORS) / sizeof(NAMED_COLORS[0]))

class LedModule {
public:
    LedMode mode;
    const char* mode_name;
    const char* color_name;
    bool  stateChanged;

    LedModule()
        : mode(LED_OFF), mode_name("off"), color_name("off"),
          stateChanged(true), _color(CRGB::Black),
          _anim_step(0), _anim_phase(false), _anim_ms(0) {}

    void begin() {
        FastLED.addLeds<LED_TYPE, PIN_LED_DATA, LED_COLOR_ORDER>(
            _leds, LED_COUNT);
        FastLED.setBrightness(LED_BRIGHTNESS);
        _allOff();
        FastLED.show();
        Serial.printf("[LED] %d WS2812B on GPIO%d\n", LED_COUNT, PIN_LED_DATA);
    }

    // Parse MQTT command (same format as Python led_node)
    void command(JsonDocument &doc) {
        const char* m = "off";
        const char* c = "";

        if (doc.is<const char*>()) {
            m = doc.as<const char*>();
        } else if (doc.is<JsonObject>()) {
            m = doc["mode"] | "off";
            c = doc["color"] | "";
        }

        // Check if mode is actually a color name shortcut
        CRGB resolved = CRGB::White;
        const char* resolved_name = (strlen(c) > 0) ? c : "white";
        bool mode_is_color = false;

        for (size_t i = 0; i < NUM_NAMED_COLORS; i++) {
            if (strcasecmp(m, NAMED_COLORS[i].name) == 0) {
                resolved = NAMED_COLORS[i].color;
                resolved_name = NAMED_COLORS[i].name;
                mode_is_color = true;
                break;
            }
        }
        // Resolve color from 'color' field
        if (!mode_is_color) {
            for (size_t i = 0; i < NUM_NAMED_COLORS; i++) {
                if (strcasecmp(c, NAMED_COLORS[i].name) == 0) {
                    resolved = NAMED_COLORS[i].color;
                    resolved_name = NAMED_COLORS[i].name;
                    break;
                }
            }
        }

        _color = resolved;
        color_name = resolved_name;
        _anim_step = 0;
        _anim_phase = false;
        _anim_ms = millis();

        if (mode_is_color) {
            mode = LED_SOLID;
            mode_name = "solid";
        } else if (strcasecmp(m, "off") == 0) {
            mode = LED_OFF; mode_name = "off";
            _allOff(); FastLED.show();
        } else if (strcasecmp(m, "solid") == 0) {
            mode = LED_SOLID; mode_name = "solid";
            _fill(resolved); FastLED.show();
        } else if (strcasecmp(m, "blink") == 0) {
            mode = LED_BLINK; mode_name = "blink";
        } else if (strcasecmp(m, "pulse") == 0) {
            mode = LED_PULSE; mode_name = "pulse";
        } else if (strcasecmp(m, "rainbow") == 0) {
            mode = LED_RAINBOW; mode_name = "rainbow";
            color_name = "rainbow";
        } else if (strcasecmp(m, "police") == 0) {
            mode = LED_POLICE; mode_name = "police";
            color_name = "police";
        } else {
            mode = LED_OFF; mode_name = "off";
            _allOff(); FastLED.show();
        }

        stateChanged = true;
    }

    // Call at ~50 Hz for animation frames
    void update() {
        unsigned long now = millis();

        switch (mode) {
        case LED_OFF:
            break;

        case LED_SOLID:
            if (_anim_step == 0) {
                _fill(_color); FastLED.show();
                _anim_step = 1;
            }
            break;

        case LED_BLINK:
            if (now - _anim_ms >= 300) {
                _anim_ms = now;
                _anim_phase = !_anim_phase;
                if (_anim_phase) _fill(_color);
                else _allOff();
                FastLED.show();
            }
            break;

        case LED_PULSE: {
            if (now - _anim_ms >= 30) {
                _anim_ms = now;
                // 0..25 = fade in, 26..51 = fade out, then pause
                if (_anim_step <= 25) {
                    float k = _anim_step / 25.0f;
                    CRGB c(_color.r * k, _color.g * k, _color.b * k);
                    _fill(c); FastLED.show();
                } else if (_anim_step <= 51) {
                    float k = (51 - _anim_step) / 25.0f;
                    CRGB c(_color.r * k, _color.g * k, _color.b * k);
                    _fill(c); FastLED.show();
                }
                _anim_step++;
                if (_anim_step > 55) _anim_step = 0;  // ~100ms pause
            }
            break;
        }

        case LED_RAINBOW:
            if (now - _anim_ms >= 20) {
                _anim_ms = now;
                for (int i = 0; i < LED_COUNT; i++) {
                    uint8_t hue = (i * 256 / LED_COUNT + _anim_step) & 255;
                    _leds[i] = CHSV(hue, 255, 255);
                }
                FastLED.show();
                _anim_step = (_anim_step + 1) & 255;
            }
            break;

        case LED_POLICE: {
            if (now - _anim_ms >= 150) {
                _anim_ms = now;
                int half = LED_COUNT / 2;
                switch (_anim_step % 4) {
                case 0:
                    for (int i = 0; i < LED_COUNT; i++)
                        _leds[i] = (i < half) ? CRGB::Red : CRGB::Black;
                    break;
                case 1: _allOff(); break;
                case 2:
                    for (int i = 0; i < LED_COUNT; i++)
                        _leds[i] = (i >= half) ? CRGB::Blue : CRGB::Black;
                    break;
                case 3: _allOff(); break;
                }
                FastLED.show();
                _anim_step++;
            }
            break;
        }
        } // switch
    }

    void buildState(JsonDocument &doc) {
        doc["mode"]  = mode_name;
        doc["color"] = color_name;
        doc["count"] = LED_COUNT;
        stateChanged = false;
    }

    void off() {
        mode = LED_OFF;
        mode_name = "off";
        _allOff();
        FastLED.show();
    }

private:
    CRGB  _leds[LED_COUNT];
    CRGB  _color;
    int   _anim_step;
    bool  _anim_phase;
    unsigned long _anim_ms;

    void _fill(CRGB c) {
        for (int i = 0; i < LED_COUNT; i++) _leds[i] = c;
    }
    void _allOff() { _fill(CRGB::Black); }
};
