#pragma once

// DEBUG TOOL - temporary, intended to be stripped out once the LED/time/accel tuning work is
// done. To remove: delete this file, DebugConsole.cpp, DebugLedTest.*, DebugTimeTest.*,
// DebugAccelTest.*, and the "DEBUG TOOL HOOK" lines in main.cpp/WiFiOTA.cpp.

#include <Arduino.h>

enum DebugMode { DEBUG_NONE, DEBUG_LED, DEBUG_TIME, DEBUG_ACCEL };
extern DebugMode debugMode;

// Reads any pending Serial bytes, assembles them into lines, and dispatches complete lines
// to handleDebugCommand(). Call once per loop() iteration.
void debugConsolePollSerial();

// Parses one command line (from Serial or WebSerial) and routes it to the right place:
// mode-entry commands (LEDTEST/TIMETEST/ACCELTEST) when idle, EXIT/NORMAL to leave a mode,
// or the active mode's own command handler otherwise.
void handleDebugCommand(const String &cmd);
