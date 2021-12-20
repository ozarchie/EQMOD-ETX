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

// Pin definitions for HBX interface
// =================================
#ifdef m2560
#define HDA1            8           // Pin2, 4, 6 on HBX interface
#define HCL1            2           // Pin3 on HBX interface
#define HDA2            10          // Not used
#define HCL2            3           // Pin5 on HBX interface
#endif

#ifdef  mESP32
#define HDA1            25          // Pin2, 4, 6 on HBX interface
#define HCL1            26          // Pin3 on HBX interface
#define HCL2            27          // Pin5 on HBX interface
#endif

#define CR              0x0d
#define LF              0x0a

#define HBXLEN	16
#define HBXMASK	HBXLEN-1

#define H2XRESETTIME  25             // Reset H2X bus
#define BITTIME       120            // H2X clock ~200uS i.e 100us Low/High
#define HIGHTIME      120            // H2X clock ~200uS i.e 100us Low/High
#define LOWTIME       120            // H2X clock ~200uS i.e 100us Low/High
#define DSTABLE       60             // H2X data write delay ~ 50uS
#define DGLITCH       5              // H2X data glitch sample
#define ETXDELAY      6.55           // H2X ETX poll delay (mS)
#define CMNDTIME      1              // H2X command delay (mS)
#define STARTTIME     50             // H2X startup time for motors
#define CLOCKTIMEOUT  50             // H2X Clock transition timeout (uS) (for monitor mode)
#define MOTORDETECT   500            // H2X Detect Motor controller

int8_t HBXBitTime = 128;

void TimerDelayuS(unsigned int);

bool HBXSendCommand(unsigned char, unsigned char);
bool HBXStartSequence(unsigned char);
void HBXSendByte(unsigned char, unsigned char);
unsigned char HBXGetByte(unsigned char);
void HBXSend2Bytes(unsigned char);
void HBXSend3Bytes(unsigned char);
void HBXGet3Bytes(unsigned char);

void HBXTestLoop(void);
void HBXMonitorMode(void);
bool HBXMonitorHCL(unsigned char);
bool HBXMonitorBit(unsigned char);
bool HBXMonitorByte(unsigned char);
void HBXMonitorEnd(unsigned char);
void HBXMonitorMessage(unsigned char);
void HBXSaveState(unsigned char);
void HBXPrintStatus(unsigned char);
void HBXPrintState(unsigned char);

void HDAListen(void);
void HDATalk(void);
void HCL1Listen(void);
void HCL1Talk(void);
void HCL2Listen(void);
void HCL2Talk(void);
void H2XReset(void);
bool ResetMotor(unsigned char);

long TwosComplement(long);

// Monitor
void HBXMonitorLoop(void);

// Testing
void HBXTest(void);
bool HBXGet2Status(void);

#endif
