#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiiChuck.h>
#include <Adafruit_MotorShield.h>
#include <S5851A.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SparkFun_RV8803.h> 
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <FastLED.h>
#include <time.h>

#define LED_PIN     13
#define NUM_LEDS    45
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define RPM_START   20

#define DAYLIGHTSAVINGS 0
int hours_now;
int minutes_now;
int seconds_now;

TBlendType    currentBlending;

DEFINE_GRADIENT_PALETTE( passFail_gp ) {
    0, 255,   0,  0,    //red
  100, 255, 255,  0,    //yellow
  200,   0, 255,  0,    //green
  255,   255, 0,  0, }; //green

CRGBPalette16 currentPalette = passFail_gp;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
RV8803 rtc;
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Accessory nunchuck;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x42);
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);
S5851A sensor(0x4E);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

const char* ssid = "BecauseFi";
const char* password = "idontknow";
const char* ssid2 = "JAMES15";
const char* password2 = "604osensa";

#define BUTTON_A    0
#define BUTTON_B    16
#define BUTTON_C    2
#define SW_TOG      14
#define RTCINT_PIN  12

bool microstepFlag = false;
int microstepCounter = 0;

int statusWifi = 0;
int statusAccel = 0;
int statusTemp = 0;
int statusMotor = 0;
int statusRTC = 0;

void setupDisplayNunchuck();
int setupTempSensor();
int setupAccel();
int setupRTC();
int setupMotor();
int setupOTA();
void updateLEDstatus();
void pointAccelDown();
float readAccel(int numOfReads);
void updateTime();
void updateLEDs();
void setLEDsToTime();


void setup() {
  Serial.begin(115200);

  pinMode(SW_TOG, INPUT);
  pinMode(RTCINT_PIN, INPUT);
  setupDisplayNunchuck();

  display.println("Testing LEDs"); display.display();
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

  display.println("Connecting..."); display.display();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  statusMotor   = setupMotor();
  statusTemp    = setupTempSensor();
  statusAccel   = setupAccel();
  statusRTC     = setupRTC();
  updateLEDstatus();
  statusWifi    = setupOTA();
  updateLEDstatus();

  // myMotor->step(150,FORWARD,DOUBLE);
  // delay(2000);
  // myMotor->step(75,BACKWARD,MICROSTEP);
  // delay(2000);
  // myMotor->step(30,FORWARD,DOUBLE);
  // delay(2000);

  pointAccelDown();
  if(statusWifi && statusRTC) {
    updateTime();
  }
  //MoveToStartPosition with initial animation
}

int tempLastMillis = 0;

void loop() {
  ArduinoOTA.handle();
  yield();
  //nunchuck.readData();
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
  
  
  //Add a GPIO interrupt to update the time
  //Add a regular animation update with a function at the end to disable sections based on the time
  updateLEDs();
}

// Initialize the optional R&D addons for the nunchuck controller and the OLED 128x64 screen
void setupDisplayNunchuck()
{
  nunchuck.begin();
  if (nunchuck.type == Unknown) {
    nunchuck.type = NUNCHUCK;
  }

  display.begin(0x3C, true); // Address 0x3C default
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.setRotation(3);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
}

// Set the S-5851A sensor up in shutdown mode so it only uses power on the occasional time we decide to measure temperature
// Return: 1 if connection succeeded or 2 if it failed
int setupTempSensor()
{
  // Reset all connected S-5851A sensor by I2C General Call Address.
  // Refer to "8. General call" on datasheet.
  if (!sensor.resetByGeneralCall()) {
    display.println("Temp sensor failed: reset by general call"); display.display();
    return -1;
  }

  // Shutdown
  // Refer to "4.1 Shutdown mode (SD)" on datasheet.
  if (sensor.shutdown()) {
    display.println("S-5851A in SD mode"); display.display();
  } else {
    display.println("S-5851A can't reinstall address."); display.display();
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
      display.print("oneshot finished: ");
      display.print(millis() - startMillis);
      display.println(" ms");
      if (sensor.update()) {
        display.print("temperature: ");
        display.println(sensor.getTempC());
      } else {
        display.println("update() failed");
      }
    } else {
      display.println("Can't oneshot finished");
    }
  } else {
    display.println("requestOneshot() failed.");
  }
  return 1;
}

// Sets up the ADXL343BCCZ which sits on the triangleClock LED board. Set to 2G since we are only expecting gravity as a force
// Return: 1 if connection succeeded or -1 if it failed
int setupAccel()
{
  display.println("Accelerometer Test"); display.display();

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    display.println("FAIL no ADXL343 found"); display.display();
    return -1;
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_2_G);
  return 1;
}

// Setup the RTC and establish communication
// Return: 1 if connection succeeded or -1 if it failed
int setupRTC()
{
  if (rtc.begin() == false)
  {
    display.println("RTC not found"); display.display();
    return -1;
  }
  display.println("RTC online!"); display.display();
  rtc.set24Hour(); 
  hours_now = rtc.getHours();
  minutes_now = rtc.getMinutes();
  seconds_now = rtc.getSeconds();
  
  rtc.disableAllInterrupts();
  rtc.clearAllInterruptFlags();//Clear all flags in case any interrupts have occurred.
  rtc.setPeriodicTimeUpdateFrequency(TIME_UPDATE_1_SECOND); //Can also use TIME_UPDATE_1_MINUTE (TIME_UPDATE_1_SECOND = false, TIME_UPDATE_1_MINUTE = true)
  rtc.enableHardwareInterrupt(UPDATE_INTERRUPT); //The update interrupt needs to have the hardware interrupt enabled to function

  return 1;
}

// Initialize the motor based on the adafruit shiel code
// Return: 1 if connection succeeded or -1 if it failed
int setupMotor()
{
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    display.println("Could not find Motor Shield. Check wiring."); display.display();
    return -1;
  }
  display.println("Motor Shield found."); display.display();
  myMotor->setSpeed(RPM_START);  // 10 rpm

  return 1;
}

// Initialize the WiFi and prepare board for OTA updates
// Return: 1 if connection succeeded or -1 if it failed
int setupOTA()
{ 
  if(WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    display.println("Connection Failed! Trying work credentials");  display.display();
    WiFi.begin(ssid2, password2);
  }
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
    display.println("Connection Failed! Rebooting...");  display.display();
    return -1;
    // delay(5000);
    // ESP.restart();
  }
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  display.println("Ready");
  display.print("IP address: ");
  display.println(WiFi.localIP());
  display.display();

  return 1;
}

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

#define HISPEED     150
#define MEDSPEED    75
#define SLOWSPEED   30
#define STARTSPEED  75
#define DELAYREAD   500
#define MICRODELAYREAD  30
#define DERIVSPACE  3
#define FRAMERATEMS   20

//Uses the accelerometer and motors to position the clock upright, flashing the lights to indicate the direction it is going
void pointAccelDown()
{
  bool finished = false;
  int moveStepperFlag = 0;
  int checkAccelFlag = 1;
  int stateMachine = 0;

  int direction = FORWARD;
  int moveType = DOUBLE;
  int numOfReads = 3;
  int speed = 1;
  float accelRead = 0; //Latest accelerometer reading
  float prevAccelRead = 0; //Previous Accelerometer reading
  float prev2AccelRead = 0; //The reading before the previous accelerometer reading

  float calcBright = 0;
  unsigned long int startMillis = millis();
  unsigned long int lastMotorFinish = 0;
  unsigned long int lastDisplayUpdate = 0;
  unsigned long int currentMillis = millis();
  int currentDelay = DELAYREAD;

  int numOfReadsFinal = 2*STARTSPEED;
  int currentFinalRead = 0;
  float finalAccelArray[numOfReadsFinal];
  float finalAccelArrayAve[numOfReadsFinal];
  float finalAccelArrayDeriv[numOfReadsFinal];

  if(!statusMotor || !statusAccel) //no motor, or no accel is no bueno. Just blink for a few seconds to let someone manually move it
  {
    checkAccelFlag = 0;
    startMillis -= 50000;
  }

  while(!finished && currentMillis <= (startMillis+60000))
  {
    currentMillis = millis();
    if(checkAccelFlag && moveStepperFlag == 0 && currentMillis > (lastMotorFinish + currentDelay))
    {
      accelRead = readAccel(numOfReads);
      if(accelRead < 5)       speed = HISPEED;
      else if(accelRead < 9)  speed = MEDSPEED;
      else                    speed = SLOWSPEED;

      switch(stateMachine)
      {
        case 0: // Read accel and try moving forward
          Serial.print("State 0 - Try moving somewhere\nStarting Acceleration: ");
          Serial.println(accelRead);
          prevAccelRead = accelRead;
          moveStepperFlag = STARTSPEED;
          checkAccelFlag = 0;
          stateMachine++;
          break;
        case 1: // Finished test direction, see if it was a good choice
          Serial.print("State 1 - Finished test move, was the direction correct?\nAcceleration: ");
          Serial.println(accelRead);
          if(accelRead > prevAccelRead) //Going the right way
          {
            Serial.println("It was correct, continuing in this direction...");
            stateMachine = 3;
            moveStepperFlag = speed;
            checkAccelFlag = 0;
          } else {
            Serial.println("Accel went down, trying the other direction...");
            stateMachine = 2;
            direction = BACKWARD;
            moveStepperFlag = STARTSPEED;
            checkAccelFlag = 2;
          }
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 2:
          Serial.println("State 2 - Test that reversing direction increases accel value (otherwise board is horizontal or pointing down already and we are done");
          if(checkAccelFlag == 2)
          {
            Serial.print("1st read: ");
            Serial.println(accelRead);
            moveStepperFlag = STARTSPEED;
          } else {
            Serial.print("2nd read: ");
            Serial.println(accelRead);
            if(accelRead > prevAccelRead && prevAccelRead > prev2AccelRead)
            {
              Serial.println("Now moving in the correction direction, moving to state 3...");
              stateMachine = 3;
              moveStepperFlag = speed;
            } else if(prevAccelRead > accelRead && prevAccelRead > prev2AccelRead) {
              Serial.println("Starting spot was pointing down, entering final tuning state 4...");
              moveStepperFlag = 1;
              stateMachine = 4;
              direction = FORWARD;
              moveType = MICROSTEP;
              currentDelay = MICRODELAYREAD;
              numOfReads = 5;
            } else {
              Serial.println("accel reads don't make sense, the board is probably horizontal or something, exiting this function...");
              finished = true;
            }
          }
          checkAccelFlag = 0;
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 3:
          if(accelRead > prevAccelRead)
          {
            Serial.print("State 3 - Keep on moving...\nAccel: ");
            Serial.println(accelRead);
            moveStepperFlag = speed;
          } else {
            Serial.print("State 3 - Moved past down, time to fine tune...\nAccel: ");
            Serial.println(accelRead);
            // Reverse direction
            if(direction == FORWARD)  direction = BACKWARD;
            else                      direction = FORWARD;
            moveStepperFlag = 1;
            stateMachine = 4;
            numOfReadsFinal = 2*speed;
            moveType = MICROSTEP;
            currentDelay = MICRODELAYREAD;
            numOfReads = 5;
          }
          checkAccelFlag = 0;
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 4:
          if(currentFinalRead == 0)
            Serial.print("State 4 - Move 1 step at a time until you've found the minimum value...\nAccel: ");
          Serial.println(accelRead);
          finalAccelArray[currentFinalRead] = accelRead;
          currentFinalRead++;
          if(currentFinalRead >= numOfReadsFinal) //finished your final collection swatch, time to do some math
          {
            int numToDivide = 0;
            for(int i=0; i<numOfReadsFinal; i++) //Do a rolling average going 5 units before and after
            {
              numToDivide = 0;
              for(int j=i-5; j <= i+5; j++)
              {
                if(j >= 0 && j < numOfReadsFinal) {
                  finalAccelArrayAve[i] += finalAccelArray[j];
                  numToDivide++;
                }
              }

              finalAccelArrayAve[i]  /= numToDivide;
            }
            for(int i=0; i<numOfReadsFinal; i++) // Use the rolling average values to do a centred derivative
            {
              float derivValue = 0;
              if(i==0)
                derivValue = (finalAccelArrayAve[DERIVSPACE] - finalAccelArrayAve[i]) / DERIVSPACE;
              else if(i==numOfReadsFinal-1)
                derivValue = (finalAccelArrayAve[i] - finalAccelArrayAve[i-3]) / DERIVSPACE;
              else
              {
                int lower = i - 3;
                int higher = i + 3;
                if(lower < 0)
                  lower = 0;
                if(higher >= numOfReadsFinal)
                  higher = numOfReadsFinal-1;

                derivValue = 0.5*(((finalAccelArrayAve[i] - finalAccelArrayAve[lower])/(i-lower)) + ((finalAccelArrayAve[higher] - finalAccelArrayAve[i])/(higher-i)));
              }
              
              finalAccelArrayDeriv[i] = derivValue;
            }
            int minValue = abs(finalAccelArrayDeriv[0]);
            int minIndex = 0;
            for(int i=1; i<numOfReadsFinal; i++) //Looking for a derivative of 0, find absolute value minimum
            {
              if(abs(finalAccelArrayDeriv[i]) < minValue)
              {
                minIndex = i;
                minValue = abs(finalAccelArrayDeriv[i]);
              }
            }
            // Reverse direction
            if(direction == FORWARD)  direction = BACKWARD;
            else                      direction = FORWARD;

            checkAccelFlag = 0;
            moveStepperFlag = numOfReadsFinal-(minIndex+1);
            stateMachine = 5;
          } else {
            moveStepperFlag = 1;
            checkAccelFlag = 0;
          }
          break;
        case 5:
          Serial.print("Youre done!\nAccel: ");
          Serial.print(accelRead);
          myMotor->release();
          finished = true;
          break;
      }
    }

    if(moveStepperFlag > 0) {
      myMotor->step(1, direction, moveType);
      moveStepperFlag--;
      if(moveStepperFlag == 0)
      {
        lastMotorFinish = millis();
        if(checkAccelFlag != 2)
          checkAccelFlag = 1;
      }
    }
    // if(moveStepperFlag > 0) {
    //   myMotor->step(moveStepperFlag, direction, moveType);
    //   moveStepperFlag = 0;
    //   lastMotorFinish = millis();
    //   if(checkAccelFlag != 2)
    //     checkAccelFlag = 1;
    // }


    if(currentMillis > (lastDisplayUpdate+FRAMERATEMS) ) {
      lastDisplayUpdate = currentMillis;
      for(int i=0; i < 7; i++)
      {
        calcBright = beatsin8(20,0,255, 0, -13*i)*max(1.0, ((i+4)/9.0));
        if(i == 6) {
          calcBright += 140;
          if( calcBright > 255)
            calcBright = 255;

        }
        leds[7+i] = CHSV( 0 ,255,(int) calcBright);
        leds[20-i] = leds[7+i];
      }
      FastLED.show();
    }

    yield();
    ArduinoOTA.handle();
  }
}

// readAccel will read the accelerometer x amount of times and average the result of the y value
// input: numOfReads is an int saying how many reads to average across
// output: the averaged value in m/s^2 (negative of it so that gravity is higher)
float readAccel(int numOfReads)
{
  sensors_event_t event;
  accel.getEvent(&event);

  float accelValueY = 0;

  for(int i=0; i<numOfReads; i++)
  {
    accel.getEvent(&event);
    accelValueY += event.acceleration.y;
  }
  accelValueY /= numOfReads;

  return accelValueY;
}

// Does an NTP read of the time and updates the RTC
void updateTime()
{
  timeClient.begin(); 
  timeClient.setTimeOffset((-8*60*60) + (DAYLIGHTSAVINGS*60*60));

  timeClient.update();
  Serial.println("NTP Time:");
  Serial.println(timeClient.getFormattedTime());

  time_t rawtime = timeClient.getEpochTime();
  struct tm * ti;
  ti = localtime(&rawtime);

  hours_now = timeClient.getHours();
  minutes_now = timeClient.getMinutes();
  seconds_now = timeClient.getSeconds();

  if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    //String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
    String currentTime = rtc.stringTime(); //Get the time
    Serial.println("RTC Date (before update):");
    Serial.println(currentDate);
    Serial.println("RTC Time (before update):");
    Serial.println(currentTime);
  }
  else
  {
    display.print("RTC read failed - Can't update time");
  }
  rtc.setTime(seconds_now, minutes_now, hours_now, timeClient.getDay(), ti->tm_mday, ti->tm_mon + 1, ti->tm_year + 1900);
}

// Fills LEDs through a palette, and then disables them to properly indicate the time.
void updateLEDs()
{
  display.clearDisplay();
  display.setCursor(0,0);
  display.print(rtc.stringTime());
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
    //something to check the position
    offsetHours = hours_now - 3;
    

  } else if( hours_now >= 11 && hours_now < 19) { //midday
    //something to check the position
    offsetHours = hours_now - 11;

  } else {//night
  //something to check the position
    if(hours_now >= 19)
      offsetHours = hours_now - 19;
    else
      offsetHours = hours_now + 5;
  }
  if(!(0x01 & hours_now))
  {
    for(int i=7; i<=13; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x02 & hours_now)>>1))
  {
    for(int i=14; i<=20; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x04 & hours_now)>>2))
  {
    for(int i=0; i<=6; i++) {
      leds[i] = CRGB::Black;
    }
  }
  //Minutes
  if(!(0x01 & minutes_now))
  {
    for(int i=31; i<=32; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x02 & minutes_now)>>1))
  {
    for(int i=29; i<=30; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x04 & minutes_now)>>2))
  {
    for(int i=27; i<=28; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x08 & minutes_now)>>3))
  {
    for(int i=21; i<=22; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x10 & minutes_now)>>4))
  {
    for(int i=23; i<=24; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x20 & minutes_now)>>5))
  {
    for(int i=25; i<=26; i++) {
      leds[i] = CRGB::Black;
    }
  }
  //Seconds
  if(!(0x01 & seconds_now))
  {
    for(int i=33; i<=34; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x02 & seconds_now)>>1))
  {
    for(int i=35; i<=36; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x04 & seconds_now)>>2))
  {
    for(int i=37; i<=38; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x08 & seconds_now)>>3))
  {
    for(int i=39; i<=40; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x10 & seconds_now)>>4))
  {
    for(int i=41; i<=42; i++) {
      leds[i] = CRGB::Black;
    }
  }
  if(!((0x20 & seconds_now)>>5))
  {
    for(int i=43; i<=44; i++) {
      leds[i] = CRGB::Black;
    }
  }
}