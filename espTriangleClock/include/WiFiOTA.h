#pragma once

#include <stdarg.h>

// ESPAsyncWebServer/ESPAsyncTCP-esphome must be included before ESP8266WiFi.h - both define a
// conflicting wl_tcp_state-style enum (CLOSING/LAST_ACK/TIME_WAIT), and ESP8266WiFi.h's copy is
// guarded to back off only if lwip's tcp.h was already seen. See esp8266/Arduino#7249.
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <NTPClient.h>
#include <SparkFun_RV8803.h>

extern bool webSerialReady; // set once WiFi is up and WebSerial.begin() has been called
extern RV8803 rtc;

void beginWiFiConnection();
int setupRTC();
int setupOTA();
void updateTime();
void onWebSerialMessage(uint8_t *data, size_t len);

// Mirrors every log line to both the physical Serial port and the WebSerial browser console
// (https://<device-ip>/webserial) once WiFi is up, so debugging works remotely over OTA too.
template<typename T>
void logPrint(T value)
{
  Serial.print(value);
  if(webSerialReady)
    WebSerial.print(value);
}

template<typename T>
void logPrintln(T value)
{
  Serial.println(value);
  if(webSerialReady)
    WebSerial.println(value);
}

void logPrintln();
void logPrintf(const char* fmt, ...);
