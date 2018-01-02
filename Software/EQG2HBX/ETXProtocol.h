/*
 * Copyright 2017, 2018 John Archbold
*/


/********************************************************
  EQG Protocol function definitions
  =================================
 *********************************************************/
#ifndef ETXProtocol
#define ETXProtocol

#define MotorAz		      0x01        // Pin3 on HBX interface
#define MotorAlt	      0x02        // Pin5 on HBX interface

#define AzMotor         MotorAz
#define AltMotor        MotorAlt

// ETX Bit Definitions
// Variable - axis[Motor].HBXMotorControl
// nibble 4
#define StartHBX        0x8000      // Motor start bit
#define StopHBX         0x4000      // Motor stop bit
#define MoveHBX         0x2000      // Move pending
#define SpeedHBX        0x1000      // Speed change pending
#define GoToHBX         0x0800      // GoTo in Progress

// ETX Known Commands
#define	RotateSlow	    0x00					// Output "8.16" 	  speed
#define	RotateFast	    0x01					// Output "16.8"    ?	speed
#define	SetOffset		    0x02			    // Output "16" 		  correction offset
#define	SetLEDI		      0x03					// Output "8" 		  LED current
#define	CalibrateLED	  0x04					// None
#define	Stop			      0x05					// None
#define	SlewReverse	    0x06					// None
#define	SlewForward	    0x07					// None
#define	GetStatus	      0x08					// Input "16.8.1" 	ticks.pwm.error
#define	GetLEDI		      0x09					// Input "8"		    LED current
#define	GetMotorType	  0x0B					// Input "8" 		    Motor type
#define	ResetH2X		    0xE4					// None

// ETX State Machine
#define ETXCheckStartup   1
#define ETXSlewMotor      2
#define ETXStepMotor      3
#define ETXCheckSlowDown  4
#define ETXCheckSpeed     5
#define ETXCheckPosition  6
#define ETXCheckStop      7
#define ETXStopMotor      8
#define ETXMotorEnd       9

#define EEPROMAzLEDI    0x01          // EEPROM Storage
#define EEPROMAltLEDI   0x02
#define EEPROMMotor     0x03          // MOTOR TYPE

#define ETX60PERIOD     152.587891    // (1/6.55mS)

#define ETX_CENTRE      0x00800000
#define AzSTEPSPER360   1233750     // See etx60at-gears.ods 
#define AltSTEPSPER360  1315440 
#define AzWORM          94
#define AltWORM         58
#define AzPEC           AzSTEPSPER360/AzWORM
#define AltPEC          AltSTEPSPER360/AltWORM

#define AzSIDEREALRATE  0x001805      // (AzSTEPSPER360/SIDEREALSECS)/ETX60PERIOD
#define AzSOLARRATE     0x0017F5      // (AzSTEPSPER360/SOLARSECS)/ETX60PERIOD
#define AzLUNARRATE     0x00172D      // (AzSTEPSPER360/LUNARSECS)/ETX60PERIOD
#define AzDEGREERATE1   0x1675B1      // (AltSTEPSPER360/360)/ETX60PERIOD
#define AzDEGREERATE2   AzDEGREERATE1 << 1

#define AltSIDEREALRATE 0x00199C      // (AltSTEPSPER360/SIDEREALSECS)/ETX60PERIOD
#define AltSOLARRATE    0x00198B      // (AzSTEPSPER360/SOLARSECS)/ETX60PERIOD
#define AltLUNARRATE    0x0018B6      // (AzSTEPSPER360/LUNARSECS)/ETX60PERIOD
#define AltDEGREERATE1  0x17F265      // (AzSTEPSPER360/360)/ETX60PERIOD
#define AltDEGREERATE2  AltDEGREERATE1 << 1

#define AzSIDEREALSPEED  0.09383854     // (AzSTEPSPER360/SIDEREALSECS)/ETX60PERIOD
#define AzSOLARSPEED     0.09358222     // (AzSTEPSPER360/SOLARSECS)/ETX60PERIOD
#define AzLUNARSPEED     0.09053403     // (AzSTEPSPER360/LUNARSECS)/ETX60PERIOD
#define AzDEGREESPEED1   22.4597        // (AzSTEPSPER360/360)/ETX60PERIOD
#define AzDEGREESPEED2   AzDEGREESPEED1 * 2.0

#define AltSIDEREALSPEED 0.10005185     // (AltSTEPSPER360/SIDEREALSECS)/ETX60PERIOD
#define AltSOLARSPEED    0.09977856     // (AzSTEPSPER360/SOLARSECS)/ETX60PERIOD
#define AltLUNARSPEED    0.09652854     // (AzSTEPSPER360/LUNARSECS)/ETX60PERIOD
#define AltDEGREESPEED1  23.9469        // (AltSTEPSPER360/360)/ETX60PERIOD
#define AltDEGREESPEED2  AltDEGREESPEED1 * 2.0

#define ETXSlew1        1               // 1  x sidereal (0.25 arc-min/sec or 0.0042°/sec) 
#define ETXSlew2        2               // 2  x sidereal (0.50 arc-min/sec or 0.0084°/sec)
#define ETXSlew3        8               // 8  x sidereal (   2 arc-min/sec or 0.0334°/sec)
#define ETXSlew4        16              // 16 x sidereal (   4 arc-min/sec or 0.0669°/sec)
#define ETXSlew5        64              // 64 x sidereal (  16 arc-min/sec or 0.2674°/sec)
#define ETXSlew6        120             // 30  arc-min/sec or 0.5°/sec
#define ETXSlew7        240             // 60  arc-min/sec or 1.0°/sec
#define ETXSlew8        360             // 90  arc-min/sec or 1.5°/sec
#define ETXSlew9       1080             // 270 arc-min/sec or 4.5°/sec

#define ETXSLOWPOSN     0x00000800      // Point at which to start slowdown

#define H2X_INPUTPU     INPUT_PULLUP  // Set pin data input mode
#define H2X_INPUT       INPUT         // Set pin data input mode
#define H2X_OUTPUT		  OUTPUT				// Set pin data output

bool HBXSetMotorState(unsigned char);
bool HBXCheckTargetStatus(unsigned char);
bool HBXUpdatePosn(void);

bool HBXStartMotor(unsigned char);
bool HBXStopMotor(unsigned char);
bool HBXGetStatus(unsigned char);
void HBXPosnPoll(unsigned char);

#endif
