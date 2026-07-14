#include <Arduino.h>
#include "WiFiOTA.h"
#include "Motor.h"
#include "LEDDisplay.h"
#include <Wire.h>
#include <S5851A.h>
#include <time.h>

int hours_now;
int minutes_now;
int seconds_now;

S5851A sensor(0x4E);

#define BUTTON_A  0
#define BUTTON_B  16
#define BUTTON_C  2
#define SW_TOG    14
#define RTCINT_PIN  12

int statusWifi = 0;
int statusAccel = 0;
int statusTemp = 0;
int statusMotor = 0;
int statusRTC = 0;

int setupTempSensor();
float readTemp();

void setup() {
  Serial.begin(115200);

  pinMode(SW_TOG, INPUT);

  logPrintln("Testing LEDs");
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(1);
  //fade in lights to yellow pre test

  for(int i=21; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB::Yellow;
  }
  FastLED.show();
  for(int i=2; i<=BRIGHTNESS;i++)
  {
    FastLED.setBrightness(i);
    FastLED.show();
  }

  logPrintln("Connecting...");
  beginWiFiConnection();

  statusMotor   = setupMotor();
  statusTemp    = setupTempSensor();
  statusAccel   = setupAccel();
  statusRTC     = setupRTC();
  updateLEDstatus();
  statusWifi    = setupOTA();
  updateLEDstatus();
  pointAccelDown();

  if(statusWifi && statusRTC) {
    updateTime();
  }

  if(statusMotor)
    initialMove();

  myMotor->release();
  //MoveToStartPosition with initial animation

}

int tempLastMillis = 0;
int lastMillisLEDloop = 0;

void loop() {
  ArduinoOTA.handle();
  if(webSerialReady)
    WebSerial.loop();
  //nunchuck.readData();
  yield();
  if( millis() >= tempLastMillis + 1000)
  {
    tempLastMillis = millis();
    seconds_now++;
    if(seconds_now == 60)
    {
      seconds_now = 0;
      minutes_now++;
      if(minutes_now == 60)
      {
        minutes_now = 0;
        hours_now++;
        if(hours_now==24)
          hours_now = 0;
      }
    }
  }
  if(orientationStepCounter > 0)
  {
    myMotor->step(1,orientationDirectionMove, MICROSTEP);
    orientationStepCounter--;
    if(orientationStepCounter == 0)
      myMotor->release();
  }


  //Add a GPIO interrupt to update the time
  //Add a regular animation update with a function at the end to disable sections based on the time
  //Probably need to add something to set the framerate of the LEDs
  if(millis() > lastMillisLEDloop + LED_FRAMEMS)
  {
    lastMillisLEDloop = millis();
    updateLEDs();
  }
}

// Set the S-5851A sensor up in shutdown mode so it only uses power on the occasional time we decide to measure temperature
// Return: 1 if connection succeeded or 2 if it failed
int setupTempSensor()
{
  // Reset all connected S-5851A sensor by I2C General Call Address.
  // Refer to "8. General call" on datasheet.
  if (!sensor.resetByGeneralCall()) {
    logPrintln("Temp sensor failed: reset by general call");
    return -1;
  }

  // Shutdown
  // Refer to "4.1 Shutdown mode (SD)" on datasheet.
  if (sensor.shutdown()) {
    logPrintln("S-5851A in SD mode");
  } else {
    logPrintln("S-5851A can't reinstall address.");
    return -1;
  }

  return 1;
}

float readTemp()
{
  if (sensor.requestOneshot()) {
    unsigned long startMillis = millis();
    int counter;
    for (counter = 0; counter < 500; counter++) {
      if (sensor.checkOneshotFinished()) {
        break;
      }
      delay(10);
    }
    if (counter < 500) {
      logPrint("oneshot finished: ");
      logPrint(millis() - startMillis);
      logPrintln(" ms");
      if (sensor.update()) {
        logPrint("temperature: ");
        logPrintln(sensor.getTempC());
      } else {
        logPrintln("update() failed");
      }
    } else {
      logPrintln("Can't oneshot finished");
    }
  } else {
    logPrintln("requestOneshot() failed.");
  }
  return 1;
}
