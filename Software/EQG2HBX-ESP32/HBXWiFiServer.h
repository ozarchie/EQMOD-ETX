/*
 * Copyright 2017, 2018 John Archbold
*/


/********************************************************
  EQG Serial WiFi
  ===============
 *********************************************************/
#ifndef HBXWiFiServer
#define HBXWiFiServer

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <esp_now.h>

/**************************************************************
 *	WiFi communications buffers and pointers
 *	WiFi variables
 **************************************************************/

uint8_t smac[] = { 0x5C, 0xCF, 0x7F, 0x88, 0x88, 0x88 };		// Hopefully :) Unique Espressif mac
uint8_t mmac[] = { 0x5C, 0xCF, 0x7F, 0x00, 0x00, 0x00 };		// Master mac address
const uint8_t WIFI_CHANNEL = 4;

// char ssid[64] = "EQMODWiFi";
// char pass[64] = "CShillit0";
String  ssid;
String  pass;

IPAddress ip(192, 168, 88, 1);
IPAddress netmask(255, 255, 255, 0);

/**************************************************************
 *	WiFi WebServer
*/
AsyncWebServer server(80);

/**************************************************************
 *	WiFi ESP_NOW
*/
esp_now_peer_info serial_peer;

/**************************************************************
 *	WiFi UDP
*/
WiFiUDP udp;
IPAddress remoteIp;
const int localUdpPort = 11880;

/**************************************************************
 *	WiFi Data Buffers
*/
struct __attribute__((packed)) DataStruct {
	char text[ESP_NOW_MAX_DATA_LEN];
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
unsigned long WiFiTimeout;
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

bool UDPFlag = false;
bool APFlag = false;
bool SerialFlag = false;

void HBXWiFiSetup();
bool HBXCheckRx();
void HBXCheckTx();
void InitESPNow();
void recvCallBack(const uint8_t*, const uint8_t* , int );
void sendData(const esp_now_peer_info_t* );
void sendCallBack(const uint8_t* , esp_now_send_status_t );
void putRxDataIntoMountInputBuffer(void);
void getTxDataFromMountOutputBuffer(void);

#endif	// HBXWiFiServer

