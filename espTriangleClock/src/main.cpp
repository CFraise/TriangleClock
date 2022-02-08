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

//Uses the accelerometer and motors to position the clock upright, flashing the lights to indicate the direction it is going
void pointAccelDown()
{
  while(true)
  {


    for(int i=0; i < 7; i++)
    {
      leds[7+i] = CSHV(255,255,255);
      leds[20-i] = same;
      FastLED.show();
    }
    
  }

}
