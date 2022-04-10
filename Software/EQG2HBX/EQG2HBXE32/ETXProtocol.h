/*
 * Copyright 2017, 2018, 2020 John Archbold
*/

/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/
#pragma once

// Function Prototypes
//
void UpdateETX(void);
void CheckETXStatus(unsigned char);
void CheckETXState(unsigned char);

bool HBXGetStatus( unsigned char );
bool ETXState( unsigned char );
void AzInitialise( unsigned char );
void AltInitialise( unsigned char );
void WaitForMotors( void );
void CalibrateLEDs( void);
void HBXPrintStatus( unsigned char );
bool HBXReset( void );

void PrintHbxValues( unsigned char );
void PrintHbxValues( unsigned char );
void dbgRx(void);

#define HBXAux          0x00        // The HBX interface port (Aux)
#define MotorAz		      0x01        // Pin7 on HBX interface
#define MotorAlt	      0x02        // Pin5 on HBX interface

uint8_t AuxHBX   = HBXAux;
uint8_t AzMotor   = MotorAz;
uint8_t AltMotor  = MotorAlt;

// ETX Bit Definitions
// Variable - axis[Motor].MotorControl
// nibble 4
#define StartHBX        0x8000      // Motor start bit
#define StopHBX         0x4000      // Motor stop bit
#define SlewHBX         0x2000      // Move in progress
#define SpeedHBX        0x1000      // Speed change pending
#define GoToHBX         0x0800      // GoTo in process

// ETX Known Commands
#define	SpeedChnge	    0x00		// Update "8.16" 	  speed
#define	SpeedStart	    0x01		// Begin  "8.16"    speed
#define	SetOffset				0x02		// Output "16" 		  correction offset
#define	SetLEDI					0x03		// Output "8" 		  LED current
#define	CalibrateLED		0x04		// None
#define	Stop						0x05		// None
#define	SlewReverse	    0x06		// None
#define	SlewForward	    0x07		// None
#define	GetStatus				0x08		// Input "16.8.1" 	ticks.pwm.error
#define	GetLEDI					0x09		// Input "8"		    LED current
#define	GetMotorType		0x0B		// Input "8" 		    Motor type
#define	SleepHBX				0xE4		// None

#define OffsetMax       0x0020        // Maximum for a SetOffset command
// ETX State Machine
#define ETXIdle           0
#define ETXCheckStartup   1
#define ETXSlewMotor      2
#define ETXStepMotor      3
#define ETXCheckSlowDown  4
#define ETXCheckSpeed     5
#define ETXCheckPosition  6
#define ETXCheckStop      7
#define ETXStopMotor      8
#define ETXMotorEnd       9
// ETX axis State Machine
#define NORMAL						0
#define FLIP							1
#define	FLIPPED			      2
#define UNFLIP			      3
#define FLIPPING		      4
#define UNFLIPPING		    5

const float           ETX60PERIOD     = 152.587891;		// (1/6.5536mS)
const unsigned long		ETX_CENTRE = 0x00800000;			// RA, DEC;

const float   MeadeSidereal   = 6460.0900;    // Refer Andrew Johansen - Roboscope
const float   SiderealArcSecs = 15.041069;    // Sidereal arcsecs/sec (ArcSec360/Sidereal secs)
const float   ArcSecs360      = 1296000;      // Arcsecs / 360


//#define ETXSlew0        .5              // .5  x sidereal (0.125 arc-min/sec or 0.0021°/sec) 
#define ETXSlew1        1               // 1  x sidereal (0.25 arc-min/sec or 0.0042°/sec) 
#define ETXSlew2        2               // 2  x sidereal (0.50 arc-min/sec or 0.0084°/sec)
#define ETXSlew3        8               // 8  x sidereal (   2 arc-min/sec or 0.0334°/sec)
#define ETXSlew4        16              // 16 x sidereal (   4 arc-min/sec or 0.0669°/sec)
#define ETXSlew5        64              // 64 x sidereal (  16 arc-min/sec or 0.2674°/sec)
#define ETXSlew6        120             // 30  arc-min/sec or 0.5°/sec
#define ETXSlew7        240             // 60  arc-min/sec or 1.0°/sec
#define ETXSlew8        600             // 150 arc-min/sec or 2.5°/sec
#define ETXSlew9       1080             // 270 arc-min/sec or 4.5°/sec

#define ETXSLOWPOSN     0x00000800      // Point at which to start slowdown

// bool HBXGetStatus(unsigned char);

/*
bool HBXSetMotorState(unsigned char);
bool HBXCheckTargetStatus(unsigned char);
bool HBXUpdatePosn(void);
bool HBXStartMotor(unsigned char);
bool HBXStopMotor(unsigned char);
void PositionPoll(unsigned char);
*/
