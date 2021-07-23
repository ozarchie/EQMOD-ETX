/*
 * Copyright 2017, 2018, 2020 John Archbold
*/

/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/
#pragma once

#define CR              0x0d
#define LF              0x0a

#define HBXLEN	16
#define HBXMASK	HBXLEN-1

#define HCLRESETTIME  5             // Reset H2X Clock (mS)
#define H2XRESETTIME  25            // Reset H2X bus
#define BITTIME       100						// H2X clock ~200uS i.e 100us Low/High

#define STATUSDELAY   50						// H2X ETX status poll delay (mS)
#define STATEDELAY		6.55          // H2X ETX state poll delay (mS)
#define CMNDTIME      1             // H2X command delay (mS)
#define STARTTIME     50            // H2X startup time for motors (mS)
#define CLOCKTIMEOUT  50            // H2X Clock transition timeout (uS) (for monitor mode)
#define MOTORDETECT   500           // H2X Detect Motor controller (mS)

uint8_t HDA = HDAAz;								// Default
uint8_t HCL = HCLAz;								// Default

uint8_t pCommand;
uint16_t pAzCount;
uint16_t pAltCount;

unsigned char HBXBitTime = BITTIME;
void TimerDelayuS(unsigned int);
void TimerDelaymS(unsigned long);

bool HBXSendCommand(unsigned char, unsigned char);
void HBXMotorReset(unsigned char);
bool HBXStartSequence(unsigned char);
void HBXSendByte(unsigned char, unsigned char);
unsigned char HBXGetByte(unsigned char);
void HBXSend2Bytes(unsigned char);
void HBXSend3Bytes(unsigned char);
void HBXGet3Bytes(unsigned char);

void HDAListen(uint8_t);
void HDATalk(uint8_t);
bool HBXReset(void);

long TwosComplement(long);

// Testing
void HBXTestLoop(void);
void HBXTest(void);
bool HBXGet2Status(void);
