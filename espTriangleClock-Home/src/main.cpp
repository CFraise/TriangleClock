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
#define BRIGHTNESS  128
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;



/* Assign a unique ID to this sensor at the same time */
/* Uncomment following line for default Wire bus      */
Adafruit_ADXL343 accel = Adafruit_ADXL343(12345);

RV8803 rtc;

//The below variables control what the date and time will be set to
int sec = 2;
int minute = 47;
int hour = 14; //Set things in 24 hour mode
int date = 2;
int month = 3;
int year = 2020;
int weekday = 2;

#define neko_width 128
#define neko_height 64

const uint8_t PROGMEM neko_data[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0x9E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x07, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0E, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x98, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0C, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x00, 0x00, 0x01, 0x0C, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x1C, 0x00, 0x80, 0x00, 0x00, 0x00, 0x02, 0x0C, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x38, 0x30, 0x80, 0x1F, 0xC0, 0x00, 0x04, 0x06, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x38, 0x78, 0xDF, 0xF0, 0xFF, 0x00, 0x0C, 0x06, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x70, 0xC8, 0x7E, 0x00, 0x00, 0x3F, 0x31, 0xC6, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x70, 0x98, 0x00, 0x00, 0x00, 0x01, 0xE3, 0xE3, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x70, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x03, 0x23, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x60, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x33, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xF1, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x61, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x7F, 0x80, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x20, 0x00, 0x00, 0x1F, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x05, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x0F, 0xFF, 0x00, 0x00, 0x07, 0x80, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x6F, 0xC0, 0x00, 0x7C, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0xE1, 0xC1, 0xF0, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x03, 0x80, 0x00, 0xFF, 0x01, 0xC3, 0xC8, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x1F, 0xFF, 0xC1, 0x41, 0xFF, 0xF0, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC0, 0x78, 0x0F, 0x01, 0xC0, 0xFF, 0xFC, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x01, 0xC3, 0x80, 0x3F, 0xC0, 0xC0, 0x00, 0x1E, 0x00, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xE0, 0x01, 0xFF, 0x00, 0x01, 0xF8, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0xE0, 0x03, 0xF0, 0x00, 0x00, 0xFC, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x70, 0x0F, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x03, 0x80, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x78, 0x1C, 0x00, 0x00, 0x00, 0x03, 0xE0, 0x03, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x38, 0x30, 0x00, 0x00, 0x00, 0x00, 0x70, 0x07, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x1C, 0x40, 0x00, 0x10, 0x03, 0x00, 0x08, 0x0E, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x0F, 0xDE, 0x00, 0x08, 0x1C, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x03, 0xF8, 0x00, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x01, 0xE0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x80, 0x00, 0x00, 0x00, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xF0, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x03, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xFC, 0x0F, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2F, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


const char* ssid = "BecauseFi";
const char* password = "idontknow";
const char* ssid2 = "JAMES15";
const char* password2 = "604osensa";

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);
Accessory nunchuck;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x42);
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 1);

S5851A sensor(0x4E);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);


#define BUTTON_A  0
#define BUTTON_B  16
#define BUTTON_C  2
#define SW_TOG    14

void updateScreen();
void scroll_cat();
void SetupPurpleAndGreenPalette();
void SetupBlackAndWhiteStripedPalette();
void SetupTotallyRandomPalette();
void ChangePalettePeriodically();
void FillLEDsFromPaletteColors( uint8_t colorIndex);

bool microstepFlag = false;
int microstepCounter = 0;


void setup() {
  Serial.begin(115200);

  nunchuck.begin();
  if (nunchuck.type == Unknown) {
    nunchuck.type = NUNCHUCK;
  }

  display.begin(0x3C, true); // Address 0x3C default

  // Clear the buffer.
  display.clearDisplay();

  display.setRotation(3);

  pinMode(SW_TOG, INPUT);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Connecting..."); display.display();
  display.drawBitmap(32, 0, neko_data, 128, 64, 1);
  display.setRotation(2);
  display.display();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  FillLEDsFromPaletteColors( 0);
  //scroll_cat();
  display.setRotation(3);
  if(WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    display.println("Connection Failed! Trying work credentials");  display.display();
    WiFi.begin(ssid2, password2);
  }
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    display.println("Connection Failed! Rebooting...");  display.display();
    delay(5000);
    ESP.restart();
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
  display.setRotation(3);
  display.println("Ready");
  display.print("IP address: ");
  display.println(WiFi.localIP());
  display.display();

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    display.println("Could not find Motor Shield. Check wiring."); display.display();
    while (1);
  }
  display.println("Motor Shield found."); display.display();
  myMotor->setSpeed(20);  // 10 rpm
  delay(1700);
  // Reset all connected S-5851A sensor by I2C General Call Address.
  // Refer to "8. General call" on datasheet.
  if (!sensor.resetByGeneralCall()) {
    Serial.println("failed: reset by general call");
    while(1) {
      delay(100);
    };
  }
  delay(100);

  // Shutdown
  // Refer to "4.1 Shutdown mode (SD)" on datasheet.
  if (sensor.shutdown()) {
    Serial.println("Shutdown. Check that temperature value is not updated even if the surrounding environment changes.");
  } else {
    Serial.println("can't reinstall address.");
  }

  timeClient.begin(); 
  timeClient.setTimeOffset(-8*60*60);

  if (rtc.begin() == false)
  {
    display.println("Device not found. Please check wiring. Freezing.");
    while(1);
  }
  display.println("RTC online!");


  //Uncomment the below code to set the RTC to your own time
  if (rtc.setTime(sec, minute, hour, weekday, date, month, year) == false) {
    display.println("Something went wrong setting the time");
  }
  rtc.set24Hour(); //Uncomment this line if you'd like to set the RTC to 24 hour mode
  display.display();

  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Accelerometer Test"); display.display();

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    display.println("Ooops, no ADXL343 detected ... Check your wiring!"); display.display();
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL343_RANGE_2_G);

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(  BRIGHTNESS );
    
  currentPalette = RainbowColors_p;
  currentBlending = LINEARBLEND;
}

void loop() {
  ArduinoOTA.handle();
  nunchuck.readData();
  updateScreen();
  yield();
  delay(20);
  if(microstepFlag == true)
  {
    myMotor->onestep(BACKWARD, MICROSTEP);
    microstepCounter++;
    if(microstepCounter >= MICROSTEPS*200) {
      microstepFlag = false;
      microstepCounter = 0;
    }
     

  }
  //myMotor->step(200, FORWARD, DOUBLE);
  Serial.println("again");

  ChangePalettePeriodically();
    
  static uint8_t startIndex = 0;
  startIndex = startIndex + 1; /* motion speed */
    
  FillLEDsFromPaletteColors( startIndex);
    
  FastLED.show();
  // display.clearDisplay();
  // display.setCursor(0,0);
  // display.println("----- Check the time until oneshot update fished -----"); display.display();
  // if (sensor.requestOneshot()) {
  //   unsigned long startMillis = millis();
  //   int counter;
  //   for (counter = 0; counter < 500; counter++) {
  //     if (sensor.checkOneshotFinished()) {
  //       break;
  //     }
  //     delay(10);
  //   }
  //   if (counter < 500) {
  //     display.print("oneshot finished: ");
  //     display.print(millis() - startMillis);
  //     display.println(" ms");
  //     if (sensor.update()) {
  //       display.print("temperature: ");
  //       display.println(sensor.getTempC());
  //     } else {
  //       display.println("update() failed");
  //     }
  //   } else {
  //     display.println("Can't oneshot finished");
  //   }
  // } else {
  //   display.println("requestOneshot() failed.");
  // }
  // display.display();

  // delay(5000);
  // display.clearDisplay();
  // display.setCursor(0,0);

  // display.println("----- Test waitForOneshotFinished() method -----");display.display();
  // if (sensor.requestOneshot()) {
  //   if (sensor.waitForOneshotFinished(1000)) {
  //     if (sensor.update()) {
  //       display.print("temperature: ");
  //       display.println(sensor.getTempC());
  //     } else {
  //       display.println("update() failed");
  //     }
  //   } else {
  //     display.println("Timeout or can't read sensor register.");
  //   }
  // } else {
  //   display.println("requestOneshot() failed.");
  // }
  // display.display();

  // delay(5000);
  // display.clearDisplay();
  // display.setCursor(0,0);

  // display.println("----- Check really shutdown mode -----");display.display();
  // for (int i = 0; i < 10; i++) {
  //   if (sensor.update()) {
  //     display.print("temperature: ");
  //     display.println(sensor.getTempC());
  //   } else {
  //     display.println("update() failed");
  //   }
  //   display.display();
  //   delay(1000);
  // }

  // delay(5000);
  // display.clearDisplay();
  // display.setCursor(0,0);
}

void updateScreen() {
  timeClient.update();
  

  display.setCursor(0,0);
  display.clearDisplay();
  display.setTextSize(1);

  display.println(timeClient.getFormattedTime());

  if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    //String currentDate = rtc.stringDate(); //Get the current date in dd/mm/yyyy format
    String currentTime = rtc.stringTime(); //Get the time
    display.print(currentDate);
    display.print(" ");
    display.println(currentTime);
  }
  else
  {
    display.print("RTC read failed");
  }

  // display.print("X: "); display.print(nunchuck.getAccelX());
  // display.print(" Y: "); display.print(nunchuck.getAccelY()); 
  // display.print(" Z: "); display.println(nunchuck.getAccelZ()); 

  sensors_event_t event;
  accel.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  display.print("X: "); display.print(event.acceleration.x); display.print("  ");
  display.print("Y: "); display.print(event.acceleration.y); display.print("  ");
  display.print("Z: "); display.print(event.acceleration.z); display.print("  ");Serial.println("m/s^2 ");

  
  display.print("Joy: ("); 
  display.print(nunchuck.getJoyX());
  display.print(", "); 
  display.print(nunchuck.getJoyY());
  display.println(")");


  display.print("Buttons:\n"); 
  // if (nunchuck.getButtonZ()) { display.print("Z "); myMotor->step(200, BACKWARD, DOUBLE); }
  // if (nunchuck.getButtonC()) { display.print("C1 "); myMotor->step(100, BACKWARD, MICROSTEP); myMotor->release(); } //microstepFlag = true;}

  if(!digitalRead(BUTTON_A)) display.print("A ");
  if(!digitalRead(BUTTON_B)) display.print("B ");
  if(!digitalRead(BUTTON_C)) display.print("C ");
  if(digitalRead(SW_TOG)) display.print("TOG ");
  display.print("\n");

  display.println(WiFi.localIP());
  display.display();  
}



void scroll_cat() {
  for(int i=0;i>-30;i--) {
    display.setRotation(3);
    display.clearDisplay();
    display.drawBitmap(i, 0, neko_data, neko_width, neko_height, 1);
    display.setRotation(2);
    display.display();
  }             
}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
    uint8_t brightness = 255;
    
    for( int i = 0; i < NUM_LEDS; ++i) {
        leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
        colorIndex += 3;
    }
}


// There are several different palettes of colors demonstrated here.
//
// FastLED provides several 'preset' palettes: RainbowColors_p, RainbowStripeColors_p,
// OceanColors_p, CloudColors_p, LavaColors_p, ForestColors_p, and PartyColors_p.
//
// Additionally, you can manually define your own color palettes, or you can write
// code that creates color palettes on the fly.  All are shown here.

void ChangePalettePeriodically()
{
    uint8_t secondHand = (millis() / 1000) % 60;
    static uint8_t lastSecond = 99;
    
    if( lastSecond != secondHand) {
        lastSecond = secondHand;
        if( secondHand ==  0)  { currentPalette = RainbowColors_p;         currentBlending = LINEARBLEND; }
        if( secondHand == 10)  { currentPalette = RainbowStripeColors_p;   currentBlending = NOBLEND;  }
        if( secondHand == 15)  { currentPalette = RainbowStripeColors_p;   currentBlending = LINEARBLEND; }
        if( secondHand == 20)  { SetupPurpleAndGreenPalette();             currentBlending = LINEARBLEND; }
        if( secondHand == 25)  { SetupTotallyRandomPalette();              currentBlending = LINEARBLEND; }
        if( secondHand == 30)  { SetupBlackAndWhiteStripedPalette();       currentBlending = NOBLEND; }
        if( secondHand == 35)  { SetupBlackAndWhiteStripedPalette();       currentBlending = LINEARBLEND; }
        if( secondHand == 40)  { currentPalette = CloudColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 45)  { currentPalette = PartyColors_p;           currentBlending = LINEARBLEND; }
        if( secondHand == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND;  }
        if( secondHand == 55)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = LINEARBLEND; }
    }
}

// This function fills the palette with totally random colors.
void SetupTotallyRandomPalette()
{
    for( int i = 0; i < 16; ++i) {
        currentPalette[i] = CHSV( random8(), 255, random8());
    }
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette()
{
    // 'black out' all 16 palette entries...
    fill_solid( currentPalette, 16, CRGB::Black);
    // and set every fourth one to white.
    currentPalette[0] = CRGB::White;
    currentPalette[4] = CRGB::White;
    currentPalette[8] = CRGB::White;
    currentPalette[12] = CRGB::White;
    
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette()
{
    CRGB purple = CHSV( HUE_PURPLE, 255, 255);
    CRGB green  = CHSV( HUE_GREEN, 255, 255);
    CRGB black  = CRGB::Black;
    
    currentPalette = CRGBPalette16(
                                   green,  green,  black,  black,
                                   purple, purple, black,  black,
                                   green,  green,  black,  black,
                                   purple, purple, black,  black );
}


// This example shows how to set up a static color palette
// which is stored in PROGMEM (flash), which is almost always more
// plentiful than RAM.  A static PROGMEM palette like this
// takes up 64 bytes of flash.
const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM =
{
    CRGB::Red,
    CRGB::Gray, // 'white' is too bright compared to red and blue
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Black,
    
    CRGB::Red,
    CRGB::Red,
    CRGB::Gray,
    CRGB::Gray,
    CRGB::Blue,
    CRGB::Blue,
    CRGB::Black,
    CRGB::Black
};



// Additional notes on FastLED compact palettes:
//
// Normally, in computer graphics, the palette (or "color lookup table")
// has 256 entries, each containing a specific 24-bit RGB color.  You can then
// index into the color palette using a simple 8-bit (one byte) value.
// A 256-entry color palette takes up 768 bytes of RAM, which on Arduino
// is quite possibly "too many" bytes.
//
// FastLED does offer traditional 256-element palettes, for setups that
// can afford the 768-byte cost in RAM.
//
// However, FastLED also offers a compact alternative.  FastLED offers
// palettes that store 16 distinct entries, but can be accessed AS IF
// they actually have 256 entries; this is accomplished by interpolating
// between the 16 explicit entries to create fifteen intermediate palette
// entries between each pair.
//
// So for example, if you set the first two explicit entries of a compact 
// palette to Green (0,255,0) and Blue (0,0,255), and then retrieved 
// the first sixteen entries from the virtual palette (of 256), you'd get
// Green, followed by a smooth gradient from green-to-blue, and then Blue.