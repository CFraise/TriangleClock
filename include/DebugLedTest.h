#pragma once

// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.

#include <Arduino.h>

void debugLedTestEnter();
void debugLedTestExit();
void debugLedTestTick();
bool debugLedTestHandleCommand(const String &cmd);
