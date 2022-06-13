/*
 * Copyright 2017, 2018, 2020, 2022 John Archbold
*/

/********************************************************
 * EQG Serial WiFi
 *********************************************************/
#pragma once

#include <WiFi.h>
#include <AsyncUDP.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Update.h>
#include <esp_now.h>
#include <ESPmDNS.h>

/**************************************************************
 *	WiFi communications buffers and pointers
 *	WiFi variables
 **************************************************************/

String  APssid;			// char AP_ssid[16] = "EQGWiFi";
String  APpass;			// char AP_pass[16] = "EQG2HBXPcl"
String  STAssid;		// char STA_ssid[16] = "EQGWiFi";
String  STApass;		// char STA_pass[16] = "EQG2HBXPcl"

const char* http_username = "admin";
const char* http_password = "eqmod";
// flag to use from web update to reboot the ESP
bool shouldReboot = false;
bool loginValid = false;

IPAddress ip(192, 168, 88, 1);
IPAddress gateway(192, 168, 88, 1);
IPAddress netmask(255, 255, 255, 0);

/**************************************************************
 *	WiFi WebServer, WebSocket
*/
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");				// access at ws://[esp ip]/ws
AsyncEventSource events("/events");		// event source (Server-Sent events)

/**************************************************************
 *	WiFi UDP
*/
WiFiUDP udp;
IPAddress remoteIp;
const int localUdpPort = 11880;
uint8_t numSsid = 0;

struct __attribute__((packed)) DataStruct {
	char text[250];
	uint8_t len;
};
DataStruct sendWiFi;
DataStruct recvWiFi;
uint8_t sendWiFi8[sizeof(sendWiFi)];
uint8_t recvWiFi8[sizeof(recvWiFi)];

/**************************************************************
 *	WiFi EQMOD virtualization
*/
#define EQxTimeout 10
#define EQxSize ESP_NOW_MAX_DATA_LEN-1

uint8_t RxD;
uint8_t TxD;
uint8_t TxDIndex;

unsigned long RxTimeout;
//unsigned long WiFiTimeout;

unsigned long LastmS;
unsigned long CheckmS = 1000;
bool Connected = false;
bool dataSending = false;
bool waitingForReply = false;

unsigned long TxDuS;
unsigned long AckuS;
unsigned long RxDuS;

unsigned long LastLEDmS;
unsigned long FastLEDmS = 200;
unsigned long SlowLEDmS = 800;
unsigned long BlinkmS = SlowLEDmS;
unsigned long SavedBlinkmS = SlowLEDmS;

bool UDPFlag = true;
bool APFlag = true;
bool STAFlag = false;
bool APSTAFlag = false;
bool SerialFlag = false;

void HBXWiFiCommsSetup();
bool HBXCheckRx();
void HBXCheckTx();

void recvCallBack(const uint8_t*, const uint8_t* , int );
void putRxDataIntoMountInputBuffer(void);
void getTxDataFromMountOutputBuffer(void);

void HBXWiFiServerSetup();