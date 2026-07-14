#pragma once

#include <FastLED.h>

#define LED_PIN     13
#define NUM_LEDS    45
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
#define LED_FRAMEMS 10

extern CRGB leds[NUM_LEDS];
extern CRGBPalette16 currentPalette;
extern TBlendType currentBlending;

void updateLEDstatus();
void updateLEDs();
void setLEDsToTime();
void turnOffLEDs(int startIndex, int endIndex);
