// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.
//
// Takes over the LED ring completely: cycles the whole ring through 10 hue steps (full
// saturation/brightness) so you can see the LEDs are alive and correctly colored, independent
// of MINUTES/SECONDS/HOURS, which black out specific LED groups on top of that cycling color to
// test the segments used to encode the time (see LEDDisplay.cpp's setLEDsToTime()).

#include "DebugLedTest.h"
#include "LEDDisplay.h"
#include "WiFiOTA.h"

#define HUE_STEP_MS     1000
#define HUE_STEP_COUNT  10

static uint8_t currentHue = 0;
static unsigned long lastHueStepMillis = 0;

// MINUTES/SECONDS: simple on/off toggle - false means the whole group is lit, true means every
// other LED in the group is blacked out.
static bool minutesHalfOn = false;
static bool secondsHalfOn = false;

// HOURS: cycles through 4 states applied identically to all three 7-LED hour groups
// (0-6, 7-13, 14-20): all on -> 0,2,4,6 -> 1,4 -> all off -> (back to all on).
static int hoursPatternState = 0;
static const bool hoursOnMask[4][7] = {
  { true,  true,  true,  true,  true,  true,  true  },
  { true,  false, true,  false, true,  false, true  },
  { false, true,  false, false, true,  false, false },
  { false, false, false, false, false, false, false },
};
static const int hoursGroupBases[3] = { 0, 7, 14 };

void debugLedTestEnter()
{
  currentHue = 0;
  lastHueStepMillis = millis();
  minutesHalfOn = false;
  secondsHalfOn = false;
  hoursPatternState = 0;
  logPrintln("Entered LED test - full ring cycling through hues.");
  logPrintln("Commands: MINUTES, SECONDS, HOURS (each toggles/cycles its LED group), EXIT.");
}

void debugLedTestExit()
{
  logPrintln("Exiting LED test.");
}

bool debugLedTestHandleCommand(const String &cmd)
{
  if(cmd == "MINUTES")
  {
    minutesHalfOn = !minutesHalfOn;
    logPrintln(minutesHalfOn ? "Minutes: half on/half off." : "Minutes: all on.");
    return true;
  }
  if(cmd == "SECONDS")
  {
    secondsHalfOn = !secondsHalfOn;
    logPrintln(secondsHalfOn ? "Seconds: half on/half off." : "Seconds: all on.");
    return true;
  }
  if(cmd == "HOURS")
  {
    hoursPatternState = (hoursPatternState + 1) % 4;
    logPrint("Hours pattern state: ");
    logPrintln(hoursPatternState);
    return true;
  }
  return false;
}

void debugLedTestTick()
{
  if(millis() - lastHueStepMillis >= HUE_STEP_MS)
  {
    lastHueStepMillis = millis();
    currentHue += (256 / HUE_STEP_COUNT);
  }

  for(int i = 0; i < NUM_LEDS; i++)
    leds[i] = CHSV(currentHue, 255, 255);

  if(minutesHalfOn)
  {
    for(int i = 21; i <= 32; i += 2)
      leds[i] = CRGB::Black;
  }
  if(secondsHalfOn)
  {
    for(int i = 33; i <= 44; i += 2)
      leds[i] = CRGB::Black;
  }
  for(int g = 0; g < 3; g++)
  {
    int base = hoursGroupBases[g];
    for(int offset = 0; offset < 7; offset++)
    {
      if(!hoursOnMask[hoursPatternState][offset])
        leds[base + offset] = CRGB::Black;
    }
  }

  FastLED.show();
}
