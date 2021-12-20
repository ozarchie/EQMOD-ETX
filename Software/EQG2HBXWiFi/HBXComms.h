/*
 * Copyright 2017, 2018 John Archbold
*/
#include <Arduino.h>

/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/
#ifndef HBXComms
#define HBXComms

#include <EEPROM.h>

 // Serial port definitions for HBX interface
 // =========================================

#ifdef mESP32
#define dbgSerial Serial
#define EQGSerial Serial2
#endif

// Pin definitions for HBX interface
// =================================

#ifdef  mESP32
#define HDA1            27          // Pin2 on HBX interface
#define HCL1            26          // Pin3 on HBX interface
#define HDA2            14          // Pin4 on HBX interface
#define HCL2            25          // Pin5 on HBX interface
#define HDAX            14          // Pin6 on HBX interface
#define HCLX            25          // Pin7 on HBX interface
#endif

#define CR              0x0d
#define LF              0x0a

#define HBXLEN	16
#define HBXMASK	HBXLEN-1

#define H2XRESETTIME  25             // Reset H2X bus
#define BITTIME       96             // H2X clock ~200uS i.e 100us Low/High

#define STATUSDELAY   50						 // H2X ETX status poll delay (mS)
#define STATEDELAY		6.55           // H2X ETX state poll delay (mS)
#define CMNDTIME      1              // H2X command delay (mS)
#define STARTTIME     50             // H2X startup time for motors
#define CLOCKTIMEOUT  50             // H2X Clock transition timeout (uS) (for monitor mode)
#define MOTORDETECT   500            // H2X Detect Motor controller

unsigned char HBXBitTime = BITTIME;
void TimerDelayuS(unsigned int);

bool HBXSendCommand(unsigned char, unsigned char);
void HBXMotorReset(unsigned char);
bool HBXStartSequence(unsigned char);
void HBXSendByte(unsigned char, unsigned char);
unsigned char HBXGetByte(unsigned char);
void HBXSend2Bytes(unsigned char);
void HBXSend3Bytes(unsigned char);
void HBXGet3Bytes(unsigned char);

void HDAListen(void);
void HDATalk(void);
void HCL1Listen(void);
void HCL1Talk(void);
void HCL2Listen(void);
void HCL2Talk(void);
void HBXReset(void);
//bool ResetMotor(unsigned char);

long TwosComplement(long);

//unsigned long eeprom_crc(void);
//unsigned long get_eeprom_crc(void);
//bool set_eeprom_crc(void);
//bool check_eeprom_crc(void);

// Testing
void HBXTestLoop(void);
void HBXTest(void);
bool HBXGet2Status(void);

#endif

