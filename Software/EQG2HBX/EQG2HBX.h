/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  EQG2HBX program definitions
  ===========================
 *********************************************************/
 
#ifndef EQG2HBX
#define EQG2HBX

// Real Time Clock Libraries
// Time related libararies
#include <DS1307RTC.h>            //https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
#include <TimeLord.h>             //https://github.com/probonopd/TimeLord
#include <TimeLib.h>              //https://github.com/PaulStoffregen/Time
#include <TimeAlarms.h>           //https://github.com/PaulStoffregen/TimeAlarms


// Pin definitions for LED indicators
// ==================================
#define AzLED           7           // Mega2560 D7
#define AltLED          6           // Mega2560 D6
#define FROMEQG         5           // Mega2560 D5
#define FROMHBX         4           // Mega2560 D4

// Jumpers to run monitor or test
// ==============================
#define TESTHBX         9           // Mega2560 D2
#define MONITORHBX      11          // Mega2560 D3

/************************************************************** 
 *  Common variables
 **************************************************************/

unsigned long DelayTimer = 0;         // Delay timer
unsigned long StatusTimer = 0;        // H2X delay timer
unsigned long StatusTime = 0;         // H2X interval time

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

unsigned char EQGDbgBuffer[EQGLEN];    // Debug
unsigned char EQGDbgPtr = 0;          // Pointer for debug

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
unsigned char EQGDbgCount;            // EQG  # parameters

/************************************************************** 
 *	HBX communications buffers and pointers
 *	HBX variables
 **************************************************************/

unsigned long H2XStart = 0;        // Used to count uS ticks
unsigned long H2XTimer = 0;        // Used to count uS ticks
unsigned char MotorStatus;        // Current State of motor
  
typedef struct {
  unsigned char MotorType;          // Current type of motor
  unsigned char MotorFlag;          // Flag to print motor positions

  unsigned long ETXMotorState;      // ETX Motor State Nachine
  unsigned long HBXMotorStatus;     // Current HBX Motor State
  unsigned long HBXMotorControl;    // Current HBX Motor Control bits

  unsigned char HBXBitCount;        // #bits left to process
  unsigned char HBXCmnd;            // Current command
  unsigned char HBXData;					  // Data byte from HBX Bus
  unsigned char HBXP1;						  // HBX status/data - MSB
  unsigned char HBXP2;						  // HBX status/data
  unsigned char HBXP3;						  // HBX status/data - LSB
  unsigned char HBXP4;              // HBX status/data - encoder error
  unsigned char HBXCount;           // HBX valid data count
  unsigned char HBXLEDI;            // LED current value from Motor  
  unsigned long HBXDirSpeed;        // Speed, Direction for Motor to move
           char HBXGuide;           // Guide speed
           long HBXSpeed;           // Move speed
           long HBXTargetSpeed;     // Target Move speed
           char HBXSpeedState;      // Slowdown/speedup state
           long HBXPosn;            // Current position
           long HBXTarget;          // Current target delta
           long HBXDelta;           // Change in position for motor speed calcs  
           long HBXSlowDown;        // Point to change to lower speed
           long HBXOffset;          // Current adjustment
           long HBX_bVALUE;         // For rate calculations
  unsigned long SIDEREALRATE;       // Constants
  unsigned long SOLARRATE;
  unsigned long LUNARRATE;
  unsigned long DEGREERATE1;
  unsigned long DEGREERATE2;
  unsigned long SIDEREALSPEED;
  unsigned long SOLARSPEED;
  unsigned long LUNARSPEED;
  unsigned long DEGREESPEED1;
  unsigned long DEGREESPEED2; 
  unsigned long STEPSPER360; 
  unsigned long WORM; 
  unsigned long PEC;
  unsigned char PrintStatus0;       // Force print of no status change
  unsigned long TimeDelta;          // Used in HBX Monitor
} axis_type;

axis_type axis[4];                  // Az, Alt


unsigned long PreviousTime;              // Used in HBX Monitor, Testing

// Testing only
unsigned char TestCount;
unsigned long TestLoopTime;

// Monitor only
unsigned char DetectedClock;

#endif

