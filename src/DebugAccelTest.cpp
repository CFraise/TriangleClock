// DEBUG TOOL - temporary, see DebugConsole.h for removal notes.
//
// No LED or time control - just accelerometer + motor, to help tune the actual pointAccelDown()
// self-leveling algorithm. Reports instantaneous accel plus a rolling ~5s average while idle,
// and accepts +N/-N to step the motor N steps at a time, printing a reading after every single
// step so you can watch how the accelerometer responds during motion (tagged [MOVING]) versus
// once it's stopped ([STILL]).
//
// The +N/-N move is paced one microstep per debugAccelTestTick() call (via millis(), not delay()),
// the same way main.cpp's loop() steps the orientation motor one microstep per loop() iteration
// rather than blocking through the whole move - so ArduinoOTA/WebSerial/Serial keep getting
// serviced (and you see [MOVING] lines stream in live) while a move is in progress.

#include "DebugAccelTest.h"
#include "Motor.h"
#include "WiFiOTA.h"

#define SAMPLE_INTERVAL_MS  100
#define REPORT_INTERVAL_MS  500
#define SAMPLE_COUNT        50  // SAMPLE_INTERVAL_MS * SAMPLE_COUNT = 5s of rolling history
#define STEP_DELAY_MS       20  // deliberately slow, so movement is easy to watch/measure

static float sampleBuffer[SAMPLE_COUNT];
static int sampleIndex = 0;
static int samplesFilled = 0;
static unsigned long lastSampleMillis = 0;
static unsigned long lastReportMillis = 0;

static int moveStepsRemaining = 0;
static int moveDirection = FORWARD;
static unsigned long lastMoveStepMillis = 0;

void debugAccelTestEnter()
{
  sampleIndex = 0;
  samplesFilled = 0;
  lastSampleMillis = millis();
  lastReportMillis = millis();
  moveStepsRemaining = 0;
  logPrintln("Entered acceleration test - LEDs/time are untouched, motor+accel only.");
  logPrintln("Commands: +N / -N to step the motor N steps, EXIT.");
}

void debugAccelTestExit()
{
  moveStepsRemaining = 0;
  myMotor->release();
  logPrintln("Exiting acceleration test.");
}

bool debugAccelTestHandleCommand(const String &cmd)
{
  if(cmd.length() < 2 || (cmd[0] != '+' && cmd[0] != '-'))
    return false;

  int steps = cmd.substring(1).toInt();
  if(steps <= 0)
    return false;

  if(moveStepsRemaining > 0)
  {
    logPrintln("Already moving - wait for the current move to finish.");
    return true;
  }

  moveDirection = (cmd[0] == '+') ? FORWARD : BACKWARD;
  moveStepsRemaining = steps;
  lastMoveStepMillis = millis();
  logPrint("Moving ");
  logPrint(steps);
  logPrintln(moveDirection == FORWARD ? " steps forward (non-blocking)..." : " steps backward (non-blocking)...");

  return true;
}

void debugAccelTestTick()
{
  unsigned long now = millis();

  if(moveStepsRemaining > 0)
  {
    if(now - lastMoveStepMillis < STEP_DELAY_MS)
      return;
    lastMoveStepMillis = now;

    myMotor->onestep(moveDirection, MICROSTEP);
    moveStepsRemaining--;
    float instant = readAccel(1);
    logPrint("[MOVING] steps remaining ");
    logPrint(moveStepsRemaining);
    logPrint(" accel=");
    logPrintln(instant);

    if(moveStepsRemaining == 0)
    {
      myMotor->release();
      logPrintln("Move complete.");
    }
    return; // skip idle sampling/reporting below while a move is in progress
  }

  if(now - lastSampleMillis < SAMPLE_INTERVAL_MS)
    return;
  lastSampleMillis = now;

  float instant = readAccel(1);
  sampleBuffer[sampleIndex] = instant;
  sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
  if(samplesFilled < SAMPLE_COUNT)
    samplesFilled++;

  if(now - lastReportMillis < REPORT_INTERVAL_MS)
    return;
  lastReportMillis = now;

  float average = 0;
  for(int i = 0; i < samplesFilled; i++)
    average += sampleBuffer[i];
  average /= samplesFilled;

  logPrint("[STILL] instant=");
  logPrint(instant);
  logPrint(" avg5s=");
  logPrintln(average);
}
