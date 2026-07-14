#include "WiFiOTA.h"
#include "wifi_credentials.h"
#include "DebugConsole.h" // DEBUG TOOL - remove this include + the handleDebugCommand() call below when stripped out

extern int hours_now;
extern int minutes_now;
extern int seconds_now;

#define DAYLIGHTSAVINGS 0

AsyncWebServer server(80);
bool webSerialReady = false;

const char* ssid = WIFI_PRIMARY_SSID;
const char* password = WIFI_PRIMARY_PASSWORD;
const char* ssid2 = WIFI_BACKUP_SSID;
const char* password2 = WIFI_BACKUP_PASSWORD;

RV8803 rtc;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void logPrintln()
{
  Serial.println();
  if(webSerialReady)
    WebSerial.println();
}

void logPrintf(const char* fmt, ...)
{
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  if(webSerialReady)
    WebSerial.print(buf);
}

void beginWiFiConnection()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
}

// Setup the RTC and establish communication
// Return: 1 if connection succeeded or -1 if it failed
int setupRTC()
{
  if (rtc.begin() == false)
  {
    logPrintln("RTC not found");
    return -1;
  }
  logPrintln("RTC online!");
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

// Initialize the WiFi and prepare board for OTA updates
// Return: 1 if connection succeeded or -1 if it failed
int setupOTA()
{
  if(WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    logPrintln("Connection Failed! Trying home credentials");
    WiFi.begin(ssid2, password2);
  }
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
    logPrintln("Connection Failed! Rebooting...");
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
    logPrintln("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    logPrintln("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    logPrintf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    logPrintf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) logPrintln("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) logPrintln("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) logPrintln("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) logPrintln("Receive Failed");
    else if (error == OTA_END_ERROR) logPrintln("End Failed");
  });
  ArduinoOTA.begin();

  WebSerial.onMessage(onWebSerialMessage);
  WebSerial.begin(&server);
  server.begin();
  webSerialReady = true;

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  logPrintln("WebSerial ready at http://" + WiFi.localIP().toString() + "/webserial");

  return 1;
}

// Handles messages typed into the WebSerial browser console - just echoes them back for now
void onWebSerialMessage(uint8_t *data, size_t len)
{
  String message;
  for(size_t i = 0; i < len; i++)
    message += char(data[i]);
  logPrintln("Received from WebSerial: " + message);
  handleDebugCommand(message); // DEBUG TOOL - remove when stripped out
}

// Does an NTP read of the time and updates the RTC
void updateTime()
{
  timeClient.begin();
  timeClient.setTimeOffset((-8*60*60) + (DAYLIGHTSAVINGS*60*60));

  timeClient.update();
  logPrintln("NTP Time:");
  logPrintln(timeClient.getFormattedTime());

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
    logPrintln("RTC Date (before update):");
    logPrintln(currentDate);
    logPrintln("RTC Time (before update):");
    logPrintln(currentTime);
  }
  else
  {
    logPrint("RTC read failed - Can't update time");
  }
  rtc.setTime(seconds_now, minutes_now, hours_now, timeClient.getDay(), ti->tm_mday, ti->tm_mon + 1, ti->tm_year + 1900);
}
