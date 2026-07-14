// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.

#include "DebugConsole.h"
#include "DebugLedTest.h"
#include "DebugTimeTest.h"
#include "DebugAccelTest.h"
#include "WiFiOTA.h"

DebugMode debugMode = DEBUG_NONE;

static String serialLineBuffer;

void debugConsolePollSerial()
{
  while(Serial.available())
  {
    char c = Serial.read();
    if(c == '\n' || c == '\r')
    {
      if(serialLineBuffer.length() > 0)
      {
        handleDebugCommand(serialLineBuffer);
        serialLineBuffer = "";
      }
    }
    else
    {
      serialLineBuffer += c;
    }
  }
}

static void exitCurrentMode()
{
  switch(debugMode)
  {
    case DEBUG_LED:   debugLedTestExit();   break;
    case DEBUG_TIME:  debugTimeTestExit();  break;
    case DEBUG_ACCEL: debugAccelTestExit(); break;
    default: break;
  }
  debugMode = DEBUG_NONE;
  logPrintln("Back to normal operation.");
}

void handleDebugCommand(const String &rawCmd)
{
  String cmd = rawCmd;
  cmd.trim();
  if(cmd.length() == 0)
    return;

  String upper = cmd;
  upper.toUpperCase();

  if(upper == "EXIT" || upper == "NORMAL")
  {
    exitCurrentMode();
    return;
  }

  if(debugMode == DEBUG_NONE)
  {
    if(upper == "LEDTEST")   { debugMode = DEBUG_LED;   debugLedTestEnter();   return; }
    if(upper == "TIMETEST")  { debugMode = DEBUG_TIME;  debugTimeTestEnter();  return; }
    if(upper == "ACCELTEST") { debugMode = DEBUG_ACCEL; debugAccelTestEnter(); return; }
    logPrintln("Unknown command. Try LEDTEST, TIMETEST, or ACCELTEST.");
    return;
  }

  bool handled = false;
  switch(debugMode)
  {
    case DEBUG_LED:   handled = debugLedTestHandleCommand(upper);   break;
    case DEBUG_TIME:  handled = debugTimeTestHandleCommand(upper);  break;
    case DEBUG_ACCEL: handled = debugAccelTestHandleCommand(upper); break;
    default: break;
  }
  if(!handled)
    logPrintln("Unrecognized command for the current test mode.");
}
