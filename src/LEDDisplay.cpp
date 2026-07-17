#include "LEDDisplay.h"
#include "Motor.h"
#include "WiFiOTA.h"

extern int statusWifi;
extern int statusAccel;
extern int statusTemp;
extern int statusMotor;
extern int statusRTC;

extern int hours_now;
extern int minutes_now;
extern int seconds_now;

CRGB leds[NUM_LEDS];
TBlendType currentBlending;

DEFINE_GRADIENT_PALETTE( passFail_gp ) {
    0, 255,   0,  0,    //red
  100, 255, 255,  0,    //yellow
  200,   0, 255,  0,    //green
  255,   255, 0,  0, }; //red

CRGBPalette16 currentPalette = passFail_gp;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

// Function for updating the start up status LEDs to their field
void updateLEDstatus()
{
  float lastX = 0;
  float lastY = 0;
  if(statusWifi == 0) // Updating the non wifi LED statuses
  {
    for(int i=1; i<=100; i++)
    {
      for(int j=21; j<=24; j++) //Temp Status LEDs
      {
        leds[j] = ColorFromPalette(currentPalette, 100 + statusTemp*i);
      }
      for(int j=25; j<=26; j++) //RTC Status LEDs
      {
        leds[j] = ColorFromPalette(currentPalette, 100 + statusRTC*i);
        leds[j+18] = ColorFromPalette(currentPalette, 100 + statusRTC*i);
      }
      for(int j=39; j<=42; j++) //Motor Status LEDs
      {
        leds[j] = ColorFromPalette(currentPalette, 100 + statusMotor*i);
        lastX = 127 + statusMotor*i;
      }
      for(int j=31; j<=34; j++) //Accel Status LEDs
      {
        leds[j] = ColorFromPalette(currentPalette, 100 + statusAccel*i);
        lastY = 127 + statusAccel*i;
      }
      FastLED.show();
      delay(3);
    }
  } else { // Updating the wifi LED status
    for(int i=1; i<=100; i++)
    {
      for(int j=27; j<=30; j++) //Temp Status LEDs
      {
        leds[j] = ColorFromPalette(currentPalette, 100 + statusWifi*i);
        leds[j+8] = ColorFromPalette(currentPalette, 100 + statusWifi*i);
      }
      FastLED.show();
      delay(3);
    }


  }
}

// Fills LEDs through a palette, and then disables them to properly indicate the time.
void updateLEDs()
{
  // This runs every LED_FRAMEMS (10ms) - too fast to usefully log, so throttle it to ~1/sec.
  // Built from hours_now/minutes_now/seconds_now (the locally-ticking clock main.cpp advances
  // every second) rather than rtc.stringTime(), which only reflects the one RTC chip read done
  // at boot in updateTime() and would otherwise print that same stale value forever.
  static unsigned long lastTimeLog = 0;
  if(millis() - lastTimeLog >= 1000)
  {
    lastTimeLog = millis();
    char timeBuf[9];
    snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", hours_now, minutes_now, seconds_now);
    Serial.println(timeBuf);
    if(webSerialReady)
      WebSerial.println(timeBuf);
  }
  for(int i=0; i < NUM_LEDS; i++)
  {
    leds[i] = CHSV(195,255,255);
  }

  setLEDsToTime();
  FastLED.show();
}

// Blacks out the segments needed to tell the time
void setLEDsToTime()
{
  int offsetHours;
  //3 chunks of day go from (bottom left) 3:00am - 10:59am, (top) 11:00am - 6:59pm, (bottom right) 7:00pm - 2:59am. Morning, midday, night
  if(hours_now >= 3 && hours_now < 11) //morning
  {
    if(orientationNow != MORNING && orientationStepCounter == 0) // Need to update position, but wait until any previous move is finished
    {
      if(orientationNow == MIDDAY)
      {
        orientationStepCounter += NUMOFMOTORSTEPS;
        orientationDirectionMove = FORWARD;
      }
      else if(orientationNow == NIGHT)
      {
        orientationStepCounter += NUMOFMOTORSTEPS;
        orientationDirectionMove = BACKWARD;
      }
      orientationNow = MORNING;
    }
    offsetHours = hours_now - 3;
  } else if( hours_now >= 11 && hours_now < 19) { //midday
    if(orientationNow != MIDDAY && orientationStepCounter == 0) // Need to update position, but wait until any previous move is finished
    {
      if(orientationNow == MORNING)
      {
        orientationStepCounter += NUMOFMOTORSTEPS;
        orientationDirectionMove = BACKWARD;
      }
      else if(orientationNow == NIGHT)
      {
        orientationStepCounter += NUMOFMOTORSTEPS;
        orientationDirectionMove = FORWARD;
      }
      orientationNow = MIDDAY;
    }
    offsetHours = hours_now - 11;
  } else {//night
    if(orientationNow != NIGHT && orientationStepCounter == 0) // Need to update position, but wait until any previous move is finished
    {
      if(orientationNow == MORNING)
      {
        orientationStepCounter += NUMOFMOTORSTEPS;
        orientationDirectionMove = FORWARD;
      }
      else if(orientationNow == MIDDAY)
      {
        orientationStepCounter += NUMOFMOTORSTEPS;
        orientationDirectionMove = BACKWARD;
      }
      orientationNow = NIGHT;
    }
    if(hours_now >= 19)
      offsetHours = hours_now - 19;
    else
      offsetHours = hours_now + 5;
  }

  if( ((orientationNow == MIDDAY) && !(0x01 & offsetHours)) || ((orientationNow == MORNING) && !((0x04 & offsetHours)>>2)) || ((orientationNow == NIGHT) && !((0x02 & offsetHours)>>1))  )
  {
    turnOffLEDs(7,13);
  }
  if( ((orientationNow == MIDDAY) && !((0x02 & offsetHours)>>1)) || ((orientationNow == MORNING) && !(0x01 & offsetHours)) || ((orientationNow == NIGHT) && !((0x04 & offsetHours)>>2))  )
  {
    turnOffLEDs(14,20);
  }
  if( ((orientationNow == MIDDAY) && !((0x04 & offsetHours)>>2)) || ((orientationNow == MORNING) && !((0x02 & offsetHours)>>1)) || ((orientationNow == NIGHT) && !(0x01 & offsetHours))  )
  {
    turnOffLEDs(0,6);
  }
  //Minutes
  if( ((orientationNow == MIDDAY) && !(0x01 & minutes_now)) || ((orientationNow == MORNING) && !((0x08 & seconds_now)>>3)) || ((orientationNow == NIGHT) && !((0x10 & minutes_now)>>4))  )
  {
    turnOffLEDs(31,32);
  }
  if( ((orientationNow == MIDDAY) && !((0x02 & minutes_now)>>1)) || ((orientationNow == MORNING) && !((0x04 & seconds_now)>>2)) || ((orientationNow == NIGHT) && !((0x20 & minutes_now)>>5))  )
  {
    turnOffLEDs(29,30);
  }
  if( ((orientationNow == MIDDAY) && !((0x04 & minutes_now)>>2)) || ((orientationNow == MORNING) && !((0x02 & seconds_now)>>1)) || ((orientationNow == NIGHT) && !((0x20 & seconds_now)>>5))  )
  {
    turnOffLEDs(27,28);
  }
  if( ((orientationNow == MIDDAY) && !((0x08 & minutes_now)>>3)) || ((orientationNow == MORNING) && !(0x01 & seconds_now)) || ((orientationNow == NIGHT) && !((0x10 & seconds_now)>>4))  )
  {
    turnOffLEDs(21,22);
  }
  if( ((orientationNow == MIDDAY) && !((0x10 & minutes_now)>>4)) || ((orientationNow == MORNING) && !(0x01 & minutes_now)) || ((orientationNow == NIGHT) && !((0x08 & seconds_now)>>3))  )
  {
    turnOffLEDs(23,24);
  }
  if( ((orientationNow == MIDDAY) && !((0x20 & minutes_now)>>5)) || ((orientationNow == MORNING) && !((0x02 & minutes_now)>>1)) || ((orientationNow == NIGHT) && !((0x04 & seconds_now)>>2))  )
  {
    turnOffLEDs(25,26);
  }
  //Seconds
  if( ((orientationNow == MIDDAY) && !(0x01 & seconds_now)) || ((orientationNow == MORNING) && !((0x10 & seconds_now)>>4)) || ((orientationNow == NIGHT) && !((0x08 & minutes_now)>>3))  )
  {
    turnOffLEDs(33,34);
  }
  if( ((orientationNow == MIDDAY) && !((0x02 & seconds_now)>>1)) || ((orientationNow == MORNING) && !((0x20 & seconds_now)>>5)) || ((orientationNow == NIGHT) && !((0x04 & minutes_now)>>2))  )
  {
    turnOffLEDs(35,36);
  }
  if( ((orientationNow == MIDDAY) && !((0x04 & seconds_now)>>2)) || ((orientationNow == MORNING) && !((0x20 & minutes_now)>>5)) || ((orientationNow == NIGHT) && !((0x02 & minutes_now)>>1))  )
  {
    turnOffLEDs(37,38);
  }
  if( ((orientationNow == MIDDAY) && !((0x08 & seconds_now)>>3)) || ((orientationNow == MORNING) && !((0x10 & minutes_now)>>4)) || ((orientationNow == NIGHT) && !(0x01 & minutes_now))  )
  {
    turnOffLEDs(39,40);
  }
  if( ((orientationNow == MIDDAY) && !((0x10 & seconds_now)>>4)) || ((orientationNow == MORNING) && !((0x08 & minutes_now)>>3)) || ((orientationNow == NIGHT) && !(0x01 & seconds_now))  )
  {
    turnOffLEDs(41,42);
  }
  if( ((orientationNow == MIDDAY) && !((0x20 & seconds_now)>>5)) || ((orientationNow == MORNING) && !((0x04 & minutes_now)>>2)) || ((orientationNow == NIGHT) && !((0x02 & seconds_now)>>1))  )
  {
    turnOffLEDs(43,44);
  }
}

//Helper function to set all LEDs between start and endIndex (inclusive) to black
void turnOffLEDs(int startIndex, int endIndex)
{
  for(int i=startIndex; i<=endIndex; i++) {
    leds[i] = CRGB::Black;
  }
}
