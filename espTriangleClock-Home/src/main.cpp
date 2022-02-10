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

void setupDisplayNunchuck();
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

//Uses the accelerometer and motors to position the clock upright, flashing the lights to indicate the direction it is going
void pointAccelDown()
{
  float calcBright = 0;
  int direction = FORWARD;
  int flashIndex = 0;
  bool finished = false;
  unsigned long int startMillis = millis();

  int moveStepperFlag = 0;
  int checkAccelFlag = 1;
  int stateMachine = 1;

  float accelRead = 0; //Latest accelerometer reading
  float prevAccelRead = 0; //Previous Accelerometer reading
  float prev2AccelRead = 0; //The reading before the previous accelerometer reading

  while(!finished && millis() <= (startMillis+30000))
  {
    if(checkAccelFlag)
    {
    accelRead = readAccel(3);
    switch(stateMachine)
    {
      case 1:
        test;
        break;
      case 2:
        test;
        break;
      case 3:
        test;
        break;
      case 4:
        test;
        break;
      case 5:
        test;
        break;
    }
    prev2AccelRead = prevAccelRead;
    prevAccelRead = accelRead;
    checkAccelFlag = 0;
    }

    if(moveStepperFlag > 0) {
      myMotor->onestep(direction, DOUBLE);
      moveStepperFlag--;
      if(moveStepperFlag == 0)
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