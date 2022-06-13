/*
 * Copyright 2017, 2018, 2020 John Archbold
*/

/********************************************************
  EQG2HBX program definitions
  ===========================
 *********************************************************/
 
#pragma once

#include <FS.h>   // Include the SPIFFS library
#include <SPIFFS.h>
#include <Preferences.h>
#include "Ticker.h"

void CheckETXState( unsigned char );

/************************************************************** 
 *  Common variables
 **************************************************************/
String EQ2HBX_Version = "EQG2HBX V1.12";

Preferences preferences;
unsigned long DelayTime;          // Delay timer
unsigned long StatusTime;         // H2X interval time

unsigned long StatusTimer;        // H2X status delay timer
unsigned long StateTimer;					// H2X state delay timer
bool					StateSelect;
bool					StatusSelect;
int						StatusCount;

Ticker AlarmCheckETX;

long          P1;
long          P2;
float         F1;
float         F2;

time_t        epoch;                  // UTC seconds
time_t        currentTime;            // Local value

/************************************************************** 
 *	EQG protocol communications buffers and pointers
 *	EQG protocol variables
 **************************************************************/

#define EQGLEN  256                   // Communications buffers
#define EQGMASK EQGLEN-1              // Index wraps to 0

unsigned long EQGP1;
unsigned long EQGP2;
float         EQGF1;

unsigned char EQGRxBuffer[EQGLEN];		// Hold data from EQG
unsigned char EQGTxBuffer[EQGLEN];    // Hold responses to EQG
unsigned char EQGRxiPtr = 0;				  // Pointer for input from EQG
unsigned char EQGRxoPtr = 0;				  // Pointer for output from EQG Rx buffer
unsigned char EQGTxiPtr = 0;				  // Pointer for input to EQG buffer
unsigned char EQGTxoPtr = 0;				  // Pointer for output to EQG

unsigned char EQGCmnd = 0;				    // EQG	Command
unsigned char EQGErrorValue;          // EQG  Returned error value
unsigned char EQGDone = 0;				    // EQG	Complete message
unsigned char EQGRADEC = 0;           // EQG  Motor Select (ASCII)
unsigned char EQGMOTOR = 0;           // EQG  Motor Select (binary)
unsigned char EQGRAAutoguide = 0;			// EQG	Autoguide rate
unsigned char EQGDECAutoguide = 0;		// EQG	Autoguide rate
unsigned char EQGRxState = 1;				  // EQG	State 
unsigned char EQGRxChar;					    // EQG	Rx Character
unsigned char EQGRxCount;             // EQG  # parameters
        float f;
unsigned long v;

/************************************************************** 
 *	HBX communications buffers and pointers
 *	HBX variables
 **************************************************************/

unsigned long H2XStart = 0;        // Used to count uS ticks
unsigned long H2XTimer = 0;        // Used to count uS ticks
unsigned char EQGMotorStatus;      // Current State of motor
  
typedef struct {
  unsigned char MotorType;          // Current type of motor
  unsigned char MotorFlag;          // Flag to print motor positions

  unsigned long ETXMotorState;      // ETX Motor State Nachine
  unsigned long ETXMotorStatus;     // Current ETX Motor Status
  unsigned long EQGMotorStatus;     // Current EQG Motor Status
  unsigned long MotorControl;       // Current HBX Motor Control bits

  unsigned char HBXBitCount;        // #bits left to process
  unsigned char Command;            // Current command
	unsigned char Flip;								// Axis flipped - Alt for negative, Az probably never
  unsigned char HBXData;					  // Data byte from HBX Bus
  unsigned char HBXP1;						  // HBX status/data - MSB
  unsigned char HBXP2;						  // HBX status/data - LSB
  unsigned char HBXP3;						  // HBX status/data - PWM % related
  unsigned char HBXP4;              // HBX status/data - single flag bit related to battery alarm ( 0 = OK )
  unsigned char HBXCount;           // HBX valid data count
  unsigned char HBXLEDI;            // LED current value from Motor  
  unsigned long DirnSpeed;          // Speed, Direction for Motor to move
           char HBXGuide;           // Guide speed
           char HBXSnapPort;        // Snap port
					 char	LEDValue;						// Polar LED brightness
           char ETXSpeedCommand;    // Current ETX Speed command
           long EQGSpeed;           // EQG Move speed
					 long ETXSpeed;           // ETX Move speed
           long TargetSpeed;        // Target Move speed
        uint8_t SpeedState;         // Slowdown/speedup state
           long Position;           // Current position
           long Target;             // Current target
           long Increment;          // Change in position for motor speed calcs  
           long SlowDown;           // Point to change to lower speed
           long Offset;             // Current adjustment
           
// MeadeRatio = ((Vanes * 4) * GbxRatio * XferRatio * WormTeeth) / 1,296,000 
          float MeadeRatio;         // Meade Ratio
          float GbxRatio;           // GearBox Ratio
  unsigned long Vanes;              // Number of photocoupler vanes
          float XferRatio;          // Gearbox Transfer Ratio (usually 1)
  unsigned long WormTeeth;          // Number of Worm teeth

// a-Value = (Vanes * 4) * GbxRatio * XferRatio * WormTeeth 
// b-Value = (6460.09 * MeadeRatio * a-Value * 15.041069) / 1,296,000            
  unsigned long aVALUE;             // For rate calculations
  unsigned long bVALUE;             // For rate calculations
  unsigned long OneDegree;          // For slew comparisons

// SIDEREALRATE = 6460.09 * MeadeRatio
// SOLARRATE = (SOLARSECS/SIDEREALSECS) * SIDEREALRATE
// LUNARRATE = (SOLARSECS/SIDEREALSECS) * SIDEREALRATE
// DEGREERATE1 = SLEW7(240) * SIDEREALRATE
// BASERATE = (b * arcsec360) / a

  unsigned long SIDEREALRATE;       // Constants
  unsigned long SOLARRATE;
  unsigned long LUNARRATE;
	unsigned long BASERATE;
	unsigned long DEGREERATE1;

// PEC = a-VALUE / WormTeeth;
  unsigned long PEC;                // PEC period (period of worm tooth)
  
  unsigned char PrintStatus0;       // Force print of no status change
  unsigned long TimeDelta;          // Used in HBX Monitor
} axis_type;

axis_type axis[4];                  // Az, Alt

// Support other scopes with Meade interface
typedef struct {
  unsigned long Vanes;              // Number of photocoupler vanes
  float         GbxRatio;           // GearBox Ratio
  float         XferRatio;          // Gearbox Transfer Ratio (usually 1)
  unsigned long WormTeeth;          // Number of Worm teeth
	char					Telescope[16];			// name of scope
} axis_values;

axis_values ratio[16][2] =																	// 16 scopes, Az, Alt
  {
    {{36, 91.1458333, 1, 94, "ETX60"}, {36, 157.5, 1, 58, "ETX60"}},          // ETX60/70/80
		{{36, 91.1458333, 1, 94, "ETX70"}, {36, 157.5, 1, 58, "ETX70"}},          // ETX60/70/80
		{{36, 91.1458333, 1, 94, "ETX80"}, {36, 157.5, 1, 58, "ETX80"}},          // ETX60/70/80
		{{108, 50, 1, 144, "LXD55"}, {108, 50, 1, 144, "LXD55"}},                 // LXD55/75, LX70-GTS
		{{108, 50, 1, 144, "LXD75"}, {108, 50, 1, 144, "LXD75"}},                 // LXD55/75, LX70-GTS
		{{108, 50, 1, 144, "LXD70"}, {108, 50, 1, 144, "LXD70"}},                 // LXD55/75, LX70-GTS
		{{108, 53.5859375, 1, 154, "LX90"}, {108, 53.5859375, 1, 154, "LX90"}},		// LX90, LT, LX80AltAz
		{{108, 53.5859375, 1, 154, "LT"}, {108, 53.5859375, 1, 154, "LT"}},				// LX90, LT, LX80AltAz
		{{108, 53.5859375, 1, 154, "LX80"}, {108, 53.5859375, 1, 154, "LX80"}},		// LX90, LT, LX80AltAz
		{{256, 50, 1, 350, "LX200"}, {256, 50, 1, 350, "LX200"}},                 // LX200
    {{500, 36, 1, 225, "LX850"}, {500, 36, 1, 225, "LX850"}},                 // LX850
    {{256, 50, 1, 180, "LX400"}, {256, 50, 1, 180, "LX400"}},                 // LX400/500
		{{36, 205.3330000, 1, 144, "DSEXT"}, {36, 205.3330000, 1, 144, "DSEXT"}}, // DS external
		{{36, 410.6660000, 1, 100, "DHEXT"}, {36, 157.5, 1, 58, "DHEXT"}},        // DH external/114EQs/4504s  

		{{36, 205.3330000, 1, 60, "ETXnn"}, {36, 205.3330000, 1, 60, "ETXnn"}},   // ETX-xxx, DS-xxx
		{{36, 91.1458333, 1, 83, "ETX??"}, {36, 144.7362076, 1, 66, "ETX??"}}    // ??
  };

unsigned char scopetype;        // Default telescope	(ETX60)
unsigned char mounttype;        // Default mounttype	(ALTAZ)
unsigned char protocoltype;			// Default protocol		(UDP)
unsigned char modetype;					// Default station		(AP)

unsigned char scopedefault = 0;        // Default telescope	(ETX60)
unsigned char mountdefault = 0;        // Default mounttype	(ALTAZ)
unsigned char protocoldefault = 0;			// Default protocol		(UDP)
unsigned char modedefault = 0;					// Default station		(AP)

String scope;                       // Text is in Meade Telescope array
String mount;
String protocol;
String mode;

char mountDesc[3][8] = {"ALTAZ", "GEM", "FORK"};
char protocolDesc[3][8] = {"UDP", "NOW", "SERIAL"};
char modeDesc[3][16] = {"WIFI_AP", "WIFI_STA", "WIFI_STA_AP"};

char* an0 = (char*)"Bad";
char* an1 = (char*)"Az ";
char* an2 = (char*)"Alt";
char * axis_name[3] = { an0, an1, an2 };

char SpeedStateDesc[4][16] = {"ChangeToStep", "Slowing1", "Slowing2", "Stopped"};


// Testing only

Ticker AlarmDebugPrint;
#define dbgLEN  256                   // Communications buffers
#define dbgMASK dbgLEN-1              // Index wraps to 0
unsigned char dbgRxBuffer[dbgLEN];    // Hold data from EQG
         char dbgCommand[dbgLEN];     // Hold data from EQG
unsigned char dbgRxiPtr = 0;          // Pointer for input from EQG
unsigned char dbgRxoPtr = 0;          // Pointer for output from EQG Rx buffer
unsigned char dbgFlag = 0;            // Received a command
unsigned char dbgIndex = 0;           // Index into command

unsigned char TestCount;
unsigned long TestLoopTime;
unsigned long PreviousTime;              // Used in HBX Monitor, Testing
