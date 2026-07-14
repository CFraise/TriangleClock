// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.
//
// Takes over how hours_now/minutes_now/seconds_now advance: instead of the normal 1-second-per-
// second ticker, this replaces it with a controllable one (default still 1/sec, override with
// FAST-N) and lets you jam the clock straight to a given hh:mm:ss. Everything else - the
// orientation-driven motor moves and setLEDsToTime()/updateLEDs() - keeps running normally off
// the same hours_now/minutes_now/seconds_now globals, since main.cpp's loop() only swaps out the
// ticker itself while this mode is active.

#include "DebugTimeTest.h"
#include "WiFiOTA.h"

extern int hours_now;
extern int minutes_now;
extern int seconds_now;

static int secondsPerTick = 1;
static unsigned long lastTickMillis = 0;

void debugTimeTestEnter()
{
  secondsPerTick = 1;
  lastTickMillis = millis();
  logPrintln("Entered manual time test - NTP/RTC time updates are paused.");
  logPrintln("Commands: hh:mm:ss to jump to a time, FAST-N to advance N seconds/sec, EXIT.");
}

void debugTimeTestExit()
{
  logPrintln("Exiting manual time test, resyncing from RTC...");
  setupRTC();
}

bool debugTimeTestHandleCommand(const String &cmd)
{
  if(cmd.startsWith("FAST-"))
  {
    int n = cmd.substring(5).toInt();
    if(n > 0)
    {
      secondsPerTick = n;
      logPrint("Now advancing ");
      logPrint(n);
      logPrintln(" seconds per real second.");
      return true;
    }
    return false;
  }

  int firstColon = cmd.indexOf(':');
  int secondColon = cmd.indexOf(':', firstColon + 1);
  if(firstColon > 0 && secondColon > firstColon)
  {
    int h = cmd.substring(0, firstColon).toInt();
    int m = cmd.substring(firstColon + 1, secondColon).toInt();
    int s = cmd.substring(secondColon + 1).toInt();
    if(h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60)
    {
      hours_now = h;
      minutes_now = m;
      seconds_now = s;
      logPrintln("Time set to " + cmd);
      return true;
    }
  }
  return false;
}

// Mirrors the normal ticker in main.cpp's loop(), but advances secondsPerTick seconds at a time
// instead of always 1, with the same minute/hour carry and 24h wraparound.
void debugTimeTestTick()
{
  if(millis() < lastTickMillis + 1000)
    return;
  lastTickMillis = millis();

  int totalSeconds = seconds_now + secondsPerTick;
  seconds_now = totalSeconds % 60;
  int carryMinutes = totalSeconds / 60;

  int totalMinutes = minutes_now + carryMinutes;
  minutes_now = totalMinutes % 60;
  int carryHours = totalMinutes / 60;

  hours_now = (hours_now + carryHours) % 24;
}
