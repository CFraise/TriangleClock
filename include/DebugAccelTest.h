#pragma once

// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.

#include <Arduino.h>

void debugAccelTestEnter();
void debugAccelTestExit();
void debugAccelTestTick();
bool debugAccelTestHandleCommand(const String &cmd);
