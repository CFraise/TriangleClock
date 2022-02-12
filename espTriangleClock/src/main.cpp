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

#define LED_PIN     13
#define NUM_LEDS    45
#define BRIGHTNESS  255
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

#define RPM_START   20

TBlendType    currentBlending;

DEFINE_GRADIENT_PALETTE( passFail_gp ) {
    0, 255,   0,  0,    //red
  100, 255, 255,  0,    //yellow
  200,   0, 255,  0,    //green
  255,   255, 0,  0, }; //red

CRGBPalette16 currentPalette = passFail_gp;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;

/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);
RV8803 rtc;
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x42);
Adafruit_StepperMotor *myMotor = AFMS.getStepper(1024, 1);
S5851A sensor(0x4E);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

const char* ssid2 = "BecauseFi";
const char* password2 = "idontknow";
const char* ssid = "JAMES15";
const char* password = "604osensa";

#define BUTTON_A  0
#define BUTTON_B  16
#define BUTTON_C  2
#define SW_TOG    14

bool microstepFlag = false;
int microstepCounter = 0;

int statusWifi = 0;
int statusAccel = 0;
int statusTemp = 0;
int statusMotor = 0;
int statusRTC = 0;

int setupTempSensor();
int setupAccel();
int setupRTC();
int setupMotor();
int setupOTA();
void updateLEDstatus();
void pointAccelDown();
float readAccel(int numOfReads);



void setup() {
  Serial.begin(115200);

  pinMode(SW_TOG, INPUT);

  Serial.println("Testing LEDs");
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

  Serial.println("Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  statusMotor   = setupMotor();
  statusTemp    = setupTempSensor();
  statusAccel   = setupAccel();
  statusRTC     = setupRTC();
  updateLEDstatus();
  statusWifi    = setupOTA();
  updateLEDstatus();

  // Serial.println("Accelerometer noise test -> 1 read/time");
  // for(int i=0;i<10;i++) {
  // Serial.println(readAccel(1));
  // delay(100);
  // }
  // delay(2000);
  // Serial.println("Accelerometer noise test -> 3 read/time");
  // for(int i=0;i<10;i++) {
  // Serial.println(readAccel(3));
  // delay(100);
  // }
  // delay(2000);

  // Serial.println("Accelerometer noise test -> 5 read/time");
  // for(int i=0;i<10;i++) {
  // Serial.println(readAccel(5));
  // delay(100);
  // }
  // delay(2000);

  // Serial.println("Accelerometer noise test -> 10 read/time");
  // for(int i=0;i<10;i++) {
  // Serial.println(readAccel(10));
  // delay(100);
  // }

  // delay(2000);

  pointAccelDown();


  //NTP
  timeClient.begin(); 
  timeClient.setTimeOffset(-8*60*60);
  
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
}

void loop() {
  ArduinoOTA.handle();
  //nunchuck.readData();
  yield();
  delay(500);
  Serial.println("again");
  FastLED.show();
}

// Set the S-5851A sensor up in shutdown mode so it only uses power on the occasional time we decide to measure temperature
// Return: 1 if connection succeeded or 2 if it failed
int setupTempSensor()
{
  // Reset all connected S-5851A sensor by I2C General Call Address.
  // Refer to "8. General call" on datasheet.
  if (!sensor.resetByGeneralCall()) {
    Serial.println("Temp sensor failed: reset by general call");
    return -1;
  }

  // Shutdown
  // Refer to "4.1 Shutdown mode (SD)" on datasheet.
  if (sensor.shutdown()) {
    Serial.println("S-5851A in SD mode");
  } else {
    Serial.println("S-5851A can't reinstall address.");
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
      Serial.print("oneshot finished: ");
      Serial.print(millis() - startMillis);
      Serial.println(" ms");
      if (sensor.update()) {
        Serial.print("temperature: ");
        Serial.println(sensor.getTempC());
      } else {
        Serial.println("update() failed");
      }
    } else {
      Serial.println("Can't oneshot finished");
    }
  } else {
    Serial.println("requestOneshot() failed.");
  }
  return 1;
}

// Sets up the ADXL343BCCZ which sits on the triangleClock LED board. Set to 2G since we are only expecting gravity as a force
// Return: 1 if connection succeeded or -1 if it failed
int setupAccel()
{
  Serial.println("Accelerometer Test");

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("FAIL no ADXL343 found");
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
    Serial.println("RTC not found");
    return -1;
  }
  Serial.println("RTC online!");
  rtc.set24Hour(); 
  return 1;
}

// Initialize the motor based on the adafruit shiel code
// Return: 1 if connection succeeded or -1 if it failed
int setupMotor()
{
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    return -1;
  }
  Serial.println("Motor Shield found.");
  myMotor->setSpeed(RPM_START);  // 10 rpm

  return 1;
}

// Initialize the WiFi and prepare board for OTA updates
// Return: 1 if connection succeeded or -1 if it failed
int setupOTA()
{ 
  if(WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Trying home credentials");
    WiFi.begin(ssid2, password2);
  }
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
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

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

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

#define HISPEED     500
#define MEDSPEED    200
#define SLOWSPEED   100
#define STARTSPEED  200

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

  while(!finished && millis() <= (startMillis+60000))
  {
    if(checkAccelFlag && moveStepperFlag == 0)
    {
      delay(1000);
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
              stateMachine = 4;
              direction = FORWARD;
              moveStepperFlag = 1;
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
            moveType = MICROSTEP;
            numOfReads = 50;
          }
          checkAccelFlag = 0;
          prev2AccelRead = prevAccelRead;
          prevAccelRead = accelRead;
          break;
        case 4:
          Serial.print("State 4 - Move 1 step at a time until you've found the minimum value...\nAccel: ");
          Serial.print(accelRead);
          if(accelRead > prevAccelRead)
          { //keep inching
            moveStepperFlag = 1;
          } else {
            Serial.println("Previous direction was the max acceleration, move back there and finish...");
            // Reverse direction
            if(direction == FORWARD)  direction = BACKWARD;
            else                      direction = FORWARD;
            moveStepperFlag = 1;
            stateMachine = 5;
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
      myMotor->onestep(direction, moveType);
      moveStepperFlag--;
      if(moveStepperFlag == 0 && checkAccelFlag != 2)
        checkAccelFlag = 1;
    }


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