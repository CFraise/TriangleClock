#pragma once

// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.

#include <Arduino.h>

void debugTimeTestEnter();
void debugTimeTestExit();
void debugTimeTestTick();
bool debugTimeTestHandleCommand(const String &cmd);
