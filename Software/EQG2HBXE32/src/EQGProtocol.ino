/**@file*/
/*
 * Copyright 2017, 2018, 2020 John Archbold
*/
#include "Hardware.h"
#include "ETXProtocol.h"
#include "EQGProtocol.h"
#include "EQG2HBX.h"
#include "HBXComms.h"
#include "HBXFileSystem.h"
#include "HBXWiFiServer.h"
/*
// EQG Protocol description
// Courtesy Andrew Johansen - Yahoo Roboscope Group

// Transmit to EQG
==================
  :CbDDDDDD[0D]
  C = command     ( a..z, A..Z )
  b = controller  ( 1 = Az, 2 = Alt, 3 = both )
  DDDDDD = data ( little endian )   uses 24bit little endian ( unless its a bitmapped command like :G )
//-------------------------------------------------------------------------------------------------

// Receive from EQG
===================
 "=DDDDDD[0D]"                  // Data
 "!E[0D]"                       // Error
  = means success
  DDDDDD = data ( little endian )   uses 24bit little endian ( unless its a bitmapped command like :f, :q )
  ! means error
  E is reason code
  Error E = 0  Invalid Command     // the command doesn't apply to the model
            1  Invalid Paramcount  // a valid command was passed with invalid parameter count
            2  Motor not Stopped   // a valid command failed to run ( e.g. sending :G whilst motor is running )
            3  NonHex Param        // the parameter contains a non uppercase Hex Char  // Note! It doesn't mean an invalid parameter
            4  Not energised       // motor is not energised
            5  Driver Asleep       // controller is in sleep mode
            6
            7
            8  Invalid PPEC model
//-------------------------------------------------------------------------------------------------

Values for Bytes sent
:C  DDDDDD bytes

Command, Bytes Sent, Nibbles Received, Description, // Example

  sent rcvd
a  0    6  Get microsteps per 360deg             // =llhhLL[0D]
b  0    6  Get timer interrupt freq              // =llhhLL[0D]
c  0    6  Get current ":U" value                // =llhhLL[0D]
d  0    6  Get Current Encoder count             // =llhhLL[0D]  default  #x800000  = home  ( this is synched somehow with :j Data )
e  0    6  Get Motor Controller Version          // =llhhMM[0D]  MM = mount type,
                                                 //             x00 = "EQ6Pro"
                                                 //             x01 = "HEQ5"
                                                 //             x02 = "EQ5"
                                                 //             x03 = "EQ3"
                                                 //             x04 = "EQ8"
                                                 //             x05 = "AZEQ6"
                                                 //             x06 = "AZEQ5"
                                                 //             hh.ll = board version    hh=x00..x07 = equatorial
                                                 //                                        =x08..xFF = altaz
f  0    3  Get axis tracking/slewing "status"    // =ABC[0D]
                                                 // A  xxx0      0 means GOTO,    1 means SLEW           ***  these are diff to :G usage
                                                 //              0 means "actually doing" the goto. On stopping, it reverts to Slew Mode
                                                 //    xx0x      0 means  +ve,    1 means  -ve
                                                 //    x0xx      0 means LoSpeed, 1 means HiSpeed        ***
                                                 // B  xxx0      0 means stopped, 1 means moving,
                                                 //    xx0x      0 means OK,      1 means blocked   ( For DC motors only )
                                                 // C  xxx0      1 means axis is Initialised/Energised
                                                 //    xx0x      1 means level switch ON            ( AltAz mounts and DEC only )
g  0    2  Get HiSpeed multiplier                // =llhhLL[0D]  EQ6Pro, AZEQ5, EQ8 = 16   AZEQ6 = 32
h  0    6  Get Current "goto" target             // =llhhLL[0D]  last value as set by    :S or ( :j +/- :H )
i  0    6  Get Current "slew" speed              // =llhhLL[0D]  must use with :f Hi/Lo and :g multiplier for correct value
                                                 //              Note! this value gets changed as a goto is done, ie a goto trumps any prev :I data
                                                 //              AZEQ5 changes as we do a goto, EQ6 only returns one value.
j  0    6  Get Current Axis position.            // =llhhLL[0D]  Cardinal encoder count with #x800000 = 0 as a centre datum
                                                 // DEC             #x800000 = 0,    pointing at West Horizon in Sth Hemi
                                                 // DEC             #xA26C80 = -90,  pointing at Polar South  in Sth Hemi
                                                 // RA              #x800000 = 0,    CW shaft down
k  1    6  :kx0[0D]=llhhLL[0D]   gets running microstep count since last start/reset
           :kx1[0D]=llhhLL[0D]   gets running microstep count since last start/reset  then resets count
           // :k  works on EQ6Pro, but not AZEQ5, AZEQ6, EQ8
l ***
m  0    6  Appears to be ramp up for a goto      // =llhhLL[0D]  ( ie :j +/- :M ) )
                                                 // :J processing for EQ6Pro converts data to :h +/- :c  if above low distance limit
                                                 //                                        to :h         if below low distance limit
n  0    2  Read EEProm Addr                      // =DD[0D]    used with :C  for peek ???   ####
o ***
p ***
q  6    6  :qx000000[0D]=000000[0D]    if axis is CW  from home ( ie -ve ) just after home sensor trip has been reset
                        =FFFFFF[0D]               CCW from home ( ie +ve ) just after home sensor trip has been reset
                        =llhhLL[0D]    if sensor has tripped since reset ( use :W to clear data first )
           // AZEQ5 returns =000080 for Az and Alt
:qx010000[0D]=ABCDEF[0D]  ie the bitmapped nibbles for current status
           // A    8  not defined
           //      4  not defined
           //      2  PPEC ON
           //      1  PPEC training in progress,
           // B    8  supports AZ/EQ
           //      4  has Home Sensors
           //      2  supports PPEC
           //      1  supports dual encoders
           // C    8  has WIFI
           //      4  supports half current tracking          // ref :Wx06....
           //      2  axes slews must start independently     // ie cant use :J3
           //      1  has polar LED
           // D
           // E
           // F
           // EQ6    returns !0
           // AZEQ5          =0B6000  at boot
           // AZEQ6          =0B3000
           // EQ8            =076000
r  0    2  Read Register Addr                    // =DD[0D] or =DDDD or =DDDDDD  used with :A  for peek ???
                  // result appears to vary based on length of last valid data return   ref test data done lower
                  // AZEQ5 returns =[0D]  ie no data if used directly after :A
                  // must use :A then :g then :r   ( ie the :g fake sets the return length to 2 )
s  0    6  Get microsteps per worm rev           // =llhhLL[0D]   Used for wormwheel teeth calcs and PPEC
t ***
u ***
v ***
w ***
x ***
y ***
z  0    0  Set Debug Flag                        // EQ6Pro returns !0[0D],   AZEQ5/AZEQ6/EQ8 returns =[0D]

A  2    0  :AxHH[0D]      Set Register Addr      // used with :R and :r   for Register poke/peek
B ***
C  4    0  :CxLLHH[0D]    Set EEProm   Addr      // used with :N and :n   for EEProm   poke/peek
D  0    6  :Dx[0D]        Get 1x Track Rate      // =llhhLL[0D]  This is the :I rate used to give sidereal tracking
E  6    0  :ExllhhLL[0D]  Reset Axis datum to    // used to synch encoder posn against a known HA/DEC )
F  0    0  :Fx[0D]        Initialise Target Axis ( energises motor )
G  2    0  :GxAB[0D]      Prepare slew parameters using bitmapped nybbles xAB
             // ( Note: ":f" is used to read the "current" actual status )
             // A = '0' high speed GOTO slewing,      doesnt make "bitmapped" sense, but it is as coded by SkyWatcher????? ?????
             //     '1' low  speed slewing mode,      all other bytes use bitmapping ( incl :f ), this doesnt
             //     '2' low  speed GOTO mode,
             //     '3' high speed slewing mode
             // xxx0   0 means AutoGoto, 1 means manual slew or stopped
             // xx0x   0 means HiRate if Goto else LoRate if Slew
             //        speed mode for AutoGoto is ignored for EQ6Pro
             // B = '0'  +CW  and Nth Hemi
             //     '1'  -CCW and Nth Hemi
             //     '2'  +CW  and Sth Hemi
             //     '3'  -CCW and Sth Hemi
             // xxx0   0 means +ve, 1 = -ve  "motor" direction, ie code takes care of whats N/S/E/W etc
             //        +ve speed in RA  is Axle moves CW when viewed from pole
             //        +ve speed in DEC is Axle moves CCW when viewed from above
             // xx0x   0 means Nth Hemi else Sth Hemi ( ST4 guiding related ) ?????
             // Note! when using :S type gotos, the direction bit here "appears" to be ignored

H  6    0  :HxllhhLL[0D]  Set goto target ( as a delta to current ) increment. Direction set by :G,
I  6    0  :IxllhhLL[0D]  Set Manual slewing rate ( read in conjunction with Hi/Lo rate as set by :G )
J  0    0  :Jx[0D]        Start moving
K  0    0  :Kx[0D]        Stop moving normal ( ramp if reqd )
L  0    0  :Lx[0D]        Stop moving Immediately
M  6    0  :MxllhhLL[0D]  Set break point increment  // Doesnt appear to do anything ????? But possibly Ramp UP related
             // EQASCOM uses         if H > 80000 then H - 80000 else H / 2
             // Indi    uses HiSpeed if H > 3200 then 3200 else H/10        based on skywatcher code ( that also sets I )
             //              LoSpeed if H >  200 then  200 else H/10
             // no values of :M appear to affect my EQ6 behaviour
N  2    0  :NxHH[0D]      Set EEProm Value to xHH   // used with :C  for poke??   NOT TESTED
O  1    0  :OxA[0D]       Toggle "Trigger/Snap Port"  A = '1' = On, '0' = Off   // EQ6 fires both at same time via Hbx, ie :O11 :O21
                          // AZEQ5 can fire independently, EQ8 uses :O2x[0D] to fire its only port.
P  1    0  :PxA[0D]       set ST4 guiderate A = 0..4 = 1.0, 0.75, 0.50, 0.25, 0.125
Q ***                     Set Bootloader Mode       // Always uses :Qx55AA[0D] and has no response.   55AA looks like a flash unlock code ????
R  2    0  :RxHH[0D]      Set Register Value to xHH // used with :A  for poke??   NOT TESTED
S  6    0  :SxllhhLL[0D]  Set absolute goto target  // EQ8 also uses :M with this, but :M is absolute in EQ8 ?????
                          // :S appears to ignore direction data set via :G
T  6    0  :TxllhhLL[0D]  Set LSB of speed          // hhLL must be 0000.   ie equivalent to :I = ll0000[0D]  but works in HiSpeed mode ?????
                          // Set Long Goto Step Period  ( as per Synta )
U  6    0  :UxllhhLL[0D]  Set rampdown range        // sets the microsteps from target where the rampdown process begins
V  2    0  :VxHH[0D]      Set Polar LED brightness  // HH := x00 to xFF
W  6    0  :Wx000000[0D]  Start PPEC train
           :Wx010000[0D]  Stop  PPEC train
           :Wx020000[0D]  Turn PPEC ON
           :Wx030000[0D]  Turn PPEC OFF   ( also sent when synch encoder used in EQ6 in 3.36. Not in 3.37 ???
           :Wx040000[0D]  Encoder ON
           :Wx050000[0D]  Encoder OFF
           :Wx060000[0D]  Disable Full Current Low speed
           :Wx060100[0D]  Enable  Full Current Low speed
           :Wx07vvvv[0D]  Set Stride for Slewing                       // need to test
           :Wx080000[0D]  Reset Home Sensor datum
X ***
Y ***
Z ***
//=================================================================================================
When setting "GOTO" data, it appears to require a correct sequence
ie :G then :H then :M   or
   :G then :S then :M for gotos.
Mount must be stopped before sending :G here, or it chucks a fault.
:M appears to have no function anymore????

:U appears to be standalone, and can be set at any time and is totally "sticky"
   Only appears to work so far with the EQ6Pro

When getting data we also need to get current status
:j gets current offset
:f is used first to check for current mode ie slew/goto, Hi/Lo, Fwd/Bwd so we know/can check signs for :h and :m
:h gets current target  ( should be ( :j + :H ) for Fwds, ( :j - :H ) for Bwds )   // ie same as :S
:m gets ???     target  ( should be ( :j + :M ) for Fwds, ( :j - :M ) for Bwds )
:d gets the current quadrature encoder count ( if encoders are fitted ). Result is always true
   ie even if encoders are OFF, :d returns the true count.
// *** WARNING ***  :f always responds correctly to the latest :G
// however, :h and :m do not. The :M and :H/:S must be sent AFTER :G
// if you want to correctly reverse engineer settings from :h, :m

When setting "Slew" data, it also requires a set procedure
Again :G sets direction and speed "range", and must be sent when stopped.
:I is used to set the speed.
The value used is basically the no of timer interrupts per microstep
:I := ( :b * 86164 / :a ) / Speed    ( where Speed is in arcsec/sec )
Speed = g*(b*86164/9024000)/I
If :I is greater than about 10, then the slew will need to use :G = LoSpeed mode
If :I is less than 10, then the slew will need :G = HiRate, and :I := I * :g
In LoSpeed mode, once moving, simply resending a new :I will cause the speed to change.
In HiSpeed mode, you must issue a stop, resend :G, reset :I then restart.
:b = :I * Speed/g * :a / 86164
*/

/********************************************************
  EQG Protocol related functions
  ==============================
 *********************************************************/

// Process received EQG characters
// ===============================
void EQGState(void) {
  while ((EQGRxiPtr != EQGRxoPtr) && (EQGDone == 0)) {
   if (dbgFlag == 1) {
      if (EQGRxBuffer[EQGRxoPtr] == 'j')
        dbgFlag = 0;
    }
    if (EQGRxBuffer[EQGRxoPtr] == ':') {
      dbgSerial.println("");
      dbgSerial.print(">>> ");
      dbgFlag++;
    }
    if (dbgFlag) {
			if ((EQGRxBuffer[EQGRxoPtr] != CR) && (EQGRxBuffer[EQGRxoPtr] != LF))
			dbgSerial.write(EQGRxBuffer[EQGRxoPtr]);
			else dbgSerial.write('.');
//      dbgSerial.write(EQGRxBuffer[EQGRxoPtr]);
    }
          
      EQGRxChar = EQGRxBuffer[EQGRxoPtr++];   // Get a character
 			if ((EQGRxState < EQG_WAITFORCR) && (EQGRxChar < ' ')) {
				EQGRxState = EQG_INTERPRET;	          // Terminate on non-alpha
			}

// Convert hex parameters to binary nibble
// ---------------------------------------
			if ((EQGRxState > 0x03) && (EQGRxState < EQG_WAITFORCR)) {
				EQGRxChar = toupper(EQGRxChar);
				if ((EQGRxChar <= 'F') && (EQGRxChar >= '0')) {
					EQGRxChar -= '0';
					if (EQGRxChar > 9) EQGRxChar -= 0x07;
				}
				else EQGRxState = EQG_INTERPRET;      // terminate on non-hex
			}

// Rx State machine
// ----------------

			switch (EQGRxState) {

        case EQG_WAITFORCR:     // Waiting for CR
          if (EQGRxChar == CR) {
            EQGRxState = EQG_CMNDSTART; 
            EQGDone++;
          }
          break;
        
        case EQG_CMNDSTART:		  // Waiting for ':'
					if (EQGRxChar == ':') {
						EQGRxCount = 1;			// Count for # parameters
						EQGRxState++;
					}
				  break;

				case 0x02:						  // Waiting for command
					EQGCmnd = EQGRxChar;
					EQGRxCount++;
					EQGRxState++;
				  break;

				case 0x03:						  // Which motor?
					EQGRADEC = EQGRxChar;
          EQGMOTOR = EQGRADEC - '0';
					if ((EQGRADEC > '0') && (EQGRADEC < '3')) {
						EQGRxCount++;
            switch (EQGCmnd) {    // Commands that have additional bytes
							case 'q':           // Get mount assets
              case 'A':           // Not used - Set Register Addr
              case 'B':           // Unknown
              case 'C':           // Not done - Set EEPROM Addr
              case 'E':           // Set Current Position
              case 'G':           // Set Move direction and speed multiplier
              case 'H':           // Set GoTo Target Increment
              case 'I':           // Set Speed
              case 'M':           // Set BreakPoint Increment
              case 'N':           // Not done - Set EEPROM
              case 'O':           // Not done - Set trigger (0-off,1-on)
              case 'P':           // Set AutoGuide Speed
              case 'R':           // Not done - Set Register
              case 'S':           // Not done - Set GoTo Target
              case 'T':           // Unknown
              case 'U':           // Not done - Set Break Step
							case 'V':						// Set Polar LED brightness
                EQGRxState++;     // Yes, so next state
                break;
              
              default:
                EQGRxState = EQG_INTERPRET; // No, so command complete
                break;
            }   
					}
					else {
            if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
            else EQGDone++;
            EQGErrorValue = '3';      // Failure - Bad Parameters
					}
				  break;

				case 0x04:                  // First nibble				
				  EQGP1 = EQGRxChar;      // EQGRxChar already converted to binary nibble
					EQGRxCount++;           // Increase character count
					switch (EQGCmnd) {		  // Commands that send one nibble
            case 'B':             // Not done - Unknown
            case 'O':             // Set trigger (0-off,1-on)
            case 'P':             // Set ST4 Guiderate
						EQGRxState = EQG_INTERPRET;
					  break;
					default:
						EQGRxState++;
					break;
					}
				  break;

 				case 0x05:                  // Second nibble - first byte (B1 = N1N2)
						EQGP2 = EQGRxChar;      // EQGRxChar already converted to binary nibble
						EQGP1 = ((EQGP1 << 4) | EQGP2); // First byte
            EQGRxCount++;
						switch (EQGCmnd) {		  // Commands that send one byte
              case 'A':             // Set register address
              case 'G':             // Set direction, range
              case 'N':             // Set EEPROM (:C) to xHH
              case 'R':             // Set Register (:A) to xHH
							case 'V':							// Set Polar LED brightness ro xHH
								EQGRxState = EQG_INTERPRET;
							break;
							default:
								EQGRxState++;       // All the rest send 3 bytes
							break;
						}
				  break;

 				case 0x06:                    // Third nibble - N3N1N2
						EQGP2 = EQGRxChar;        // EQGRxChar already converted to binary nibble
						EQGP1 |= (EQGP2 << 8);
            EQGRxCount++;
            EQGRxState++;
           break;

 				case 0x07:                    // Fourth nibble - N3N4N1N2						
						EQGP1 &= 0xFF;            // Clear all bar low byte
            EQGP1 |= (EQGP2 << 12);   // Get N3 into correct position
						EQGP2 = EQGRxChar;        // EQGRxChar already converted to binary nibble
            EQGP1 |= (EQGP2 << 8);    // Get N4 into correct position
            EQGRxCount++;                       
            switch (EQGCmnd) {      // Commands that send two bytes
              case 'C':             // Set EEPROM address
                EQGRxState = EQG_INTERPRET;
                break;
              default:
                EQGRxState++;       // All the rest send 3 bytes
              break;
            }
				  break;

 				case 0x08:                    // Fifth nibble - N5xxN3N4N1N2
					
						EQGP2 = EQGRxChar;
            EQGP1 |= (EQGP2 << 20);
						EQGRxCount++;
						EQGRxState++;			        // Get next data
				  break;

 				case 0x09:                    // Sixth nibble - N5N6N3N4N1N2

						
						EQGP2 = EQGRxChar;
						EQGP1 |= (EQGP2 << 16);
            EQGRxCount++;
            EQGRxState = EQG_INTERPRET;		// All done
				  break;

 				case EQG_INTERPRET:					
					if ((EQGRxChar == 0x0d) && (EQGRxCount >= 3)) {
						EQGRxState = EQG_CMNDSTART;    // Reset state machine
						switch (EQGCmnd) {		
                                  // Commands that have no data
							case 'a':           // Read steps per rotation
							case 'b':           // Read tracking scale
							case 'c':           // Read Motor Speed Change
							case 'd':           // Read Current Encoder count
							case 'e':           // Read firmware version
							case 'f':           // Read motor status
							case 'g':           // Read speed divisor
							case 'j':           // Read position
              case 'm':           // Read motor slowdown point
              case 'n':           // Read EEPROM data
              case 'r':           // Read register data
							case 's':           // Read steps per arcsec
							case 'D':           // Read track rate
							case 'F':           // Energise motors
							case 'J':           // GoTo position, track
							case 'K':           // Stop movement
								EQGDone++;
							  break;

                                  // Commands that have three bytes
                                  // ==============================
							case 'q':						// Read mount assets
							case 'E':           // Set current position
							case 'H':           // Set target position
							case 'I':           // Set GoTo speed
              case 'M':           // Set motor slowdown position ??
              case 'U':           // Set motor slowdown speed ??
								if (EQGRxCount == (3 + 6)) {
									EQGDone++;
								}
								else {
                  if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                  else EQGDone++;
                  EQGErrorValue = '3'; // Failure - Bad Parameters
								}
							  break;

                                  // Commands that have two bytes
              case 'C':           // Set EEPROM address
              if (EQGRxCount == (3 + 4)) {
                  EQGDone++;
                }
                else {
                  if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                  else EQGDone++;
                  EQGErrorValue = '3'; // Failure - Bad Parameters
                }
                break;

                                  // Commands that have one byte
              case 'A':           // Set register address
              case 'G':           // Set direction, range
              case 'N':           // Set EEPROM (:C) to xHH
              case 'R':           // Set Register (:A) to xHH
							case 'V':           // Set LED Brightness to xHH
              if (EQGRxCount == (3 + 2)) {
									EQGDone++;
								}
								else {
                  if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                  else EQGDone++;
									EQGErrorValue = '3'; // Failure - Bad Parameters
							  }
							  break;

                                  // Commands that have one nibble
              case 'P':           // Set autoguide speed
              case 'O':           // Set Snap Port
								if (EQGRxCount == (3 + 1)) {
									EQGDone++;
								}
								else {
                  if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                  else EQGDone++;
                  EQGErrorValue = '3'; // Failure - Bad Parameters
								}
							  break;
                
							default:
                if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                else EQGDone++;
                EQGErrorValue = '0';  // Failure - Bad Command
							  break;
						}   // End - switch (EQGCmnd)
					}   
					else {
            if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
            else EQGDone++;
            EQGErrorValue = '0';      // Failure - Bad Command
					}
  				break; // End - if (((EQGRxChar == 0x0d) || (EQGRxChar == 0x0a)) && (EQGRxCount >= 3))
	
				default:
					EQGRxState = EQG_CMNDSTART;         
		} // End - switch (EQGRxState)
  } // END - while ((EQGRxiPtr != EQGRxoPtr) && (EQGDone == 0))
}

void EQGError(unsigned char errorbyte) {
  EQGTx('!') ;													// Failure - Bad Parameters
  EQGTx(errorbyte);
  EQGTx(CR);
  EQGDone = 0;													// Process errors
  EQGRxState = EQG_CMNDSTART;
  EQGRxCount = 0;												// Count for # parameters
  EQGErrorValue = 0;
}

// Received a valid command so execute it, if supported
// ====================================================
void EQGAction(void) {

		EQGDone = 0;						            // Reset flag
    EQGRxState = EQG_CMNDSTART;
		EQGTx('=');                         // Answer (with parameters)

		switch (EQGCmnd) {

				case 'a':						          // Request total number of steps per revolution
          EQGTxHex6(axis[EQGMOTOR].aVALUE);
				  break;

				case 'b':                     // Request step interrupt frequency	
          EQGTxHex6(axis[EQGMOTOR].bVALUE);  // Let EQMOD calculate SIDEREAL, etc
				  break;

				case 'c':
				  EQGTxHex6(0x00D800);	      // Return same as EQ6 to keep the speed commands consistent
				  break;

				case 'd':						          // Request Mount center point
//					EQGTxHex6(EQG_CENTRE);      // Return same as EQ6 to keep the speed commands consistent
          EQGTxHex6(axis[EQGMOTOR].Position);
				  break;

				case 'e':                     // Request version
					EQGTxHex6(EQGVERSION);	    //
				  break;

				case 'f':                     // Request motor status  

// EQGMotorStatus bit definitions
// A nibble1
// MOVESLEW        0x0001      // Step(0)        Slew(1)
// MOVEDECR        0x0002      // Increasing(0)  Decreasing(1)
// MOVEHIGH        0x0004      // Low(0)         High(1)
// B nibble2
// MOVEAXIS        0x0010      // Stopped(0)     Moving(1)
// MOVEFACE        0x0020      // Front(0)       Rear(1)
// C nibble3
// MOVEACTIVE      0x0100      // Inactive(0)    Active(1)

            EQGTxHex3(axis[EQGMOTOR].EQGMotorStatus);

        break;

				case 'g':                     // Request speed multiplier
					EQGTxHex2(EQG_gVALUE);      // Return same as EQ6 to keep the speed commands consistent
				  break;

				case 'j':                     // Request axis position    
					EQGTxHex6(axis[EQGMOTOR].Position);
				  break;

				case 'm':																	// GET Point at which to change from fast to slow
					EQGTxHex6(axis[EQGMOTOR].SlowDown);
				  break;

				case 'q':																	// GET mount capabilities
					EQGTxHex6(EQGASSETS);										// Say EQ and AZ
					break;

				case 's':                                 // PEC period
					EQGTxHex6(axis[EQGMOTOR].PEC);
				  break;

				case 'E':							                    // Set current motor position
					if ((EQGP1 == 0x800000) || (EQGP1 == 0x85049c))
						break;
          else axis[EQGMOTOR].Position = EQGP1;
						break;

				case 'F':                                 // Initialize and activate motors
          axis[EQGMOTOR].EQGMotorStatus |= MOVEACTIVE;      
          axis[EQGMOTOR].ETXMotorStatus |= MOVEACTIVE;      
				  break;

				case 'G':																	// EQG 'G' Command      :GxAB[0D]

// See below for A
// ===============
// B nibble
// --------
						// B = '0'  +CW  and Nthn Hemi
            //     '1'  -CCW and Nthn Hemi
            //     '2'  +CW  and Sthn Hemi
            //     '3'  -CCW and Sthn Hemi
            // xxx0   0 means +ve, 1 = -ve  "motor" direction, ie code takes care of whats N/S/E/W etc
            //        +ve speed in RA  is Axis moves CW when viewed from pole
            //        +ve speed in DEC is Axis moves CCW when viewed from above (OTA horizontal, facing E->W ?)
            // xx0x   0 means Nthn Hemi else Sthn Hemi ( ST4 guiding related ) ?????
            // Note! when using :S type gotos, the direction bit here "appears" to be ignored
            // Also note that EQMOD does not appear to send the Hemisphere bit
            // 
            // xxx0      CW(0)        CCW(1)       DIRECTION
            // xx0x      North(0)     South(1)     HEMISPHERE

				axis[EQGMOTOR].DirnSpeed = (int)EQGP1;          // Save the command value
				switch (axis[EQGMOTOR].DirnSpeed & 0x03)	{
					case 0x00:
					case 0x02:
						axis[EQGMOTOR].EQGMotorStatus &= ~MOVEDECR;
dbgSerial.print(" +CW  ");
						break;
					case 0x01:
					case 0x03:
						axis[EQGMOTOR].EQGMotorStatus |= MOVEDECR;
dbgSerial.print(" -CCW ");
						break;
					default:
						break;
				}

//	When setting "Slew" data, it also requires a set procedure
//	G sets direction and speed "range", and must be sent when stopped.
// A nibble
             // A = '0' high speed GOTO slewing,      doesnt make "bitmapped" sense, but it is as coded by SkyWatcher
             //     '1' low  speed slewing mode,      all other bytes use bitmapping ( incl :f ), this doesnt
             //     '2' low  speed GOTO mode,
             //     '3' high speed slewing mode
             // xxx0   0 means AutoGoto, 1 means manual slew or stopped
             // xx0x   0 means HiRate if Goto else LoRate if Slew
             //        speed mode for AutoGoto is ignored for EQ6Pro
       
// A
        switch ((EQGP1 >> 4) & 0x03) {
          case 00:    // 0 HIGH SPEED GOTO
            axis[EQGMOTOR].EQGMotorStatus &= ~MOVESLEW;     // GoTo target
            axis[EQGMOTOR].EQGMotorStatus |= MOVEHIGH;      // Enable high speed multiplier
						axis[EQGMOTOR].TargetSpeed = axis[EQGMOTOR].DEGREERATE1;
dbgSerial.print("HIGH SPEED GOTO ");
            break;
          case 01:    // 1 LOW  SPEED SLEW
            axis[EQGMOTOR].EQGMotorStatus |= MOVESLEW;      // Just move the axis
            axis[EQGMOTOR].EQGMotorStatus &= ~MOVEHIGH;     // Disable high speed multiplier
						axis[EQGMOTOR].TargetSpeed = (axis[EQGMOTOR].SIDEREALRATE);
dbgSerial.print("LOW  SPEED SLEW ");
            break;
          case 02:    // 2 LOW  SPEED GOTO
            axis[EQGMOTOR].EQGMotorStatus &= ~MOVESLEW;     // GoTo target
            axis[EQGMOTOR].EQGMotorStatus &= ~MOVEHIGH;     // Disable high speed multiplier
						axis[EQGMOTOR].TargetSpeed = (axis[EQGMOTOR].DEGREERATE1 << 3);
dbgSerial.print("LOW  SPEED GOTO ");
            break;
          case 03:    // 3 HIGH SPEED SLEW
            axis[EQGMOTOR].EQGMotorStatus |= MOVESLEW;      // Just move the axis
            axis[EQGMOTOR].EQGMotorStatus |= MOVEHIGH;      // Enable high speed multiplier
						axis[EQGMOTOR].TargetSpeed = axis[EQGMOTOR].DEGREERATE1;
dbgSerial.print("HIGH SPEED SLEW ");
            break;
        }
dbgSerial.print(axis[EQGMOTOR].TargetSpeed);

        axis[EQGMOTOR].ETXMotorStatus = axis[EQGMOTOR].EQGMotorStatus; // Copy the status for ETXProtocol
				break;

				case 'H':                                      				// Set the goto target increment
          axis[EQGMOTOR].Increment = EQGP1;
          if (axis[EQGMOTOR].EQGMotorStatus & MOVEDECR)
            axis[EQGMOTOR].Target = axis[EQGMOTOR].Position - axis[EQGMOTOR].Increment;   // subtract the relative target
          else
            axis[EQGMOTOR].Target = axis[EQGMOTOR].Position + axis[EQGMOTOR].Increment;   // add the relative target      
          axis[EQGMOTOR].MotorControl |= GoToHBX;
				  break;

				case 'I':   // Set motor speed
/*
					:I is used to set the speed.
						The value used is basically the number of timer interrupts per microstep
						------------------------------------------------------------------------
						I = (b * SiderealSecs / a) / Speed			(where Speed is in arcsec/sec, SiderealSecs=86164.nn)
						Speed = g * (b * SiderealSecs / a) / I
						======================================
						If I is greater than about 10, then the slew will need to use :G = LoSpeed mode
						If I is less than 10, then the slew will need :G = HiRate, and :I =  :I * :g
						In LoSpeed mode, once moving, simply resending a new :I will cause the speed to change.
						In HiSpeed mode, you must issue a stop, resend :G, reset :I then restart.
						b = I * Speed / g * a / SiderealSecs
*/

					axis[EQGMOTOR].EQGSpeed = EQGP1;							// Set EQG speed value (I)
					axis[EQGMOTOR].TargetSpeed = EQGP1;           // Set ETX target speed
					axis[EQGMOTOR].TargetSpeed = axis[EQGMOTOR].SIDEREALRATE * ((axis[EQGMOTOR].bVALUE*SIDEREALSECS)/axis[EQGMOTOR].aVALUE) / EQGP1;
					if (axis[EQGMOTOR].EQGMotorStatus & MOVEHIGH)
						axis[EQGMOTOR].TargetSpeed *= EQG_gVALUE;
					if (axis[EQGMOTOR].ETXMotorStatus & MOVEAXIS)      // If already moving
          	axis[EQGMOTOR].ETXMotorState = ETXStepMotor;     //  update speed
					break;

				case 'J':             // Tell motor to Go
					axis[EQGMOTOR].ETXMotorStatus |= MOVEAXIS;                // Signal moving
          axis[EQGMOTOR].ETXMotorState = ETXCheckStartup;           // General entry

				  break;

				case 'K':							// Tell motor to stop
          axis[EQGMOTOR].EQGMotorStatus |= MOVESLEW;              // Set slew as default
          axis[EQGMOTOR].ETXMotorStatus |= MOVESLEW;              // Set slew as default
          axis[EQGMOTOR].TargetSpeed = 0;
          axis[EQGMOTOR].ETXMotorState = ETXCheckSpeed;           //  to enable motor slowdown
				  break;

        case 'L':             // Tell motor to stop immediately
          axis[EQGMOTOR].EQGMotorStatus |= MOVESLEW;              // Clear speed change
					axis[EQGMOTOR].ETXMotorStatus |= MOVESLEW;              // Set slew as default
					axis[EQGMOTOR].TargetSpeed = 0;
          axis[EQGMOTOR].ETXMotorState = ETXStopMotor;            // Immediate stop
          break;

				case 'M':             // Set the break point increment
          axis[EQGMOTOR].SlowDown = EQGP1;
          if ((axis[EQGMOTOR].EQGMotorStatus & MOVEDECR) == 0)
            axis[EQGMOTOR].SlowDown = axis[EQGMOTOR].Position + axis[EQGMOTOR].SlowDown;  // add the relative target
          else
            axis[EQGMOTOR].SlowDown = axis[EQGMOTOR].Position - axis[EQGMOTOR].SlowDown;  // subtract the relative target
          axis[EQGMOTOR].MotorControl |= GoToHBX;            // Signal pending GoTo
				  break;

        case 'O':
          axis[EQGMOTOR].HBXSnapPort = (char)EQGP1;
          break;
        
        case 'P':
					axis[EQGMOTOR].HBXGuide = (char)EQGP1;
				  break;

				case 'U':             // Set the break point steps
// JMA TODO          axis[EQGMOTOR].SlowDown = EQGP1;
				break;

				case 'V':             // Set the LED brightness
          axis[EQGMOTOR].LEDValue = EQGP1;
					break;

				default:
          EQGErrorValue = '0';     // Failure - Bad Command
				  break;
			}
      EQGTx(CR);
}

void EQGTxHex(unsigned char data) {
	if ((data & 0x0f) < 0x0a) EQGTx((data & 0x0f) + 0x30);
	else EQGTx((data & 0x0f) + 0x37);
}

void EQGTxHex2(unsigned char data) {
	EQGTxHex(data >> 4);
	EQGTxHex(data);
}

void EQGTxHex3(unsigned int data) {
	EQGTxHex((unsigned char)data);
	EQGTxHex((unsigned char)(data >> 4));
	EQGTxHex((unsigned char)(data >> 8));
}

void EQGTxHex6(unsigned long data) {
	EQGTxHex2((unsigned char)data);
	EQGTxHex2((unsigned char)(data >> 8));
	EQGTxHex2((unsigned char)(data >> 16));
}

/**********************************************
	Debug routines
***********************************************/
void dbgRx(void) {
	while (dbgSerial.available() > 0) {
		dbgRxBuffer[dbgRxiPtr++] = dbgSerial.read();
		dbgRxiPtr &= dbgMASK;
	}
}

void putbyte(unsigned char data) {
	dbgSerial.write(data);
}

void puthexb(unsigned char data) {
	if (((data >> 4) & 0x0f) < 0x0a) putbyte(((data >> 4) & 0x0f) + 0x30);
	else putbyte(((data >> 4) & 0x0f) + 0x37);
	if ((data & 0x0f) < 0x0a) putbyte((data & 0x0f) + 0x30);
	else putbyte((data & 0x0f) + 0x37);
}

void putdecb(unsigned char data) {
	dbgSerial.print(data);
}

void puthexw(unsigned int data) {
	puthexb((data >> 8) & 0xFF);
	puthexb(data & 0xFF);
}

void putdecw(unsigned int data) {
	dbgSerial.print(data);
}

void puthex6(unsigned long data) {
	puthexb((data >> 16) & 0xFF);
	puthexw(data & 0xFFFF);
}

void puthexl(unsigned long data) {
	puthexw((data >> 16) & 0xFFFF);
	puthexw(data & 0xFFFF);
}

void putdecl(unsigned long data) {
	dbgSerial.print(data);
}

void EQGSend(unsigned char data) {
	dbgSerial.write(data);
}

void EQGSendHex(unsigned char data) {
	if ((data & 0x0f) < 0x0a) EQGSend((data & 0x0f) + 0x30);
	else EQGSend((data & 0x0f) + 0x37);
}

void EQGSendHex2(unsigned char data) {
	EQGSendHex(data >> 4);
	EQGSendHex(data);
}

void EQGSendHex6(unsigned long data) {
	EQGSendHex2((unsigned char)data);
	EQGSendHex2((unsigned char)(data >> 8));
	EQGSendHex2((unsigned char)(data >> 16));
}

void debugEQG() {
	dbgSerial.println("");
	dbgSerial.print("Az:<");
	dbgSerial.print(axis[AzMotor].EQGMotorStatus, HEX);
	dbgSerial.print("> Pos: ");
	dbgSerial.print(axis[AzMotor].Position, HEX);
	dbgSerial.print(" SD: ");
	dbgSerial.print(axis[AzMotor].SlowDown, HEX);
	dbgSerial.print(" Tgt: ");
	dbgSerial.print(axis[AzMotor].Target, HEX);
	dbgSerial.print(" Speed: ");
	dbgSerial.print(axis[AzMotor].ETXSpeed, HEX);

	dbgSerial.print(", Alt:<");
	dbgSerial.print(axis[AltMotor].EQGMotorStatus, HEX);
	dbgSerial.print(">Pos: ");
	dbgSerial.print(axis[AltMotor].Position, HEX);
	dbgSerial.print(" SD: ");
	dbgSerial.print(axis[AltMotor].SlowDown, HEX);
	dbgSerial.print(" Tgt: ");
	dbgSerial.print(axis[AltMotor].Target, HEX);
	dbgSerial.print(" Speed: ");
	dbgSerial.print(axis[AltMotor].ETXSpeed, HEX);
/*
	while (dbgRxoPtr != dbgRxiPtr) {
		dbgCommand[dbgIndex] = dbgRxBuffer[dbgRxoPtr];              // Copy character
		dbgSerial.write(dbgCommand[dbgIndex]);
		if ((dbgCommand[dbgIndex] != ':') && (dbgFlag == 0)) {      // Wait for start of command string
					dbgSerial.write(dbgCommand[dbgIndex]);                // Output to debug and skip
				}
		else {
			if (dbgCommand[dbgIndex] == CR) {
				dbgCommand[dbgIndex + 1] = 0;
				processdbgCommand();
				dbgFlag = 0;
				dbgIndex = 0;
			}
			else {
				dbgFlag |= 1;
				dbgIndex += 1;
			}
		}
		dbgRxoPtr += 1;
		dbgRxoPtr &= dbgMASK;
	}

}

// Format - ":","Motor","Command","Paramters"
// Motor			1, 2
// Command		t, w, g, v, x

void processdbgCommand(void) {
	unsigned char m, t;
	char argStr[256];

	dbgSerial.println(""); dbgSerial.print("Scope: "); dbgSerial.println(telescope);
	dbgSerial.println(dbgCommand);

	strcpy(argStr, &dbgCommand[3]);
	m = dbgCommand[2] - '1';
	if ((m == 0) || (m == 1)) {
		switch (dbgCommand[1]) {

		case 't':
			t = dbgCommand[3] - '0';
			if (t > 9) t -= 7;
			if ((t >= 0) & (t < 16))
				telescope = t;
			dbgSerial.println(""); dbgSerial.print("Scope: "); dbgSerial.println(telescope);
			break;

		case 'w':	// Number of Worm teeth
			sscanf(argStr, "%ld", &v);
			dbgSerial.println("");
			dbgSerial.print("WormTeeth: ");
			dbgSerial.print(argStr);
			dbgSerial.print(" ");
			dbgSerial.println(f);
			ratio[telescope][m].WormTeeth = v;
			break;

		case 'g':	// Gearbox Ratio (float)
			sscanf(argStr, "%f", &f);                 // Warning: you need the float libraries for this
			dbgSerial.println("");
			dbgSerial.print("GearBox: ");             // "-Wl,-u,vfscanf -lscanf_flt -lm" in platform.local.txt
			dbgSerial.print(argStr);
			dbgSerial.print(" ");
			dbgSerial.println(f);
			ratio[telescope][m].GbxRatio = f;
			break;

		case 'v':	// Number of optical vanes per revolution
			sscanf(argStr, "%ld", &v);
			dbgSerial.println("");
			dbgSerial.print("Vanes: ");
			dbgSerial.print(argStr);
			dbgSerial.print(" ");
			dbgSerial.println(v);
			ratio[telescope][m].Vanes = v;
			break;

		case 'x':	// Gear transfer ratio
			sscanf(argStr, "%f", &f);                 // Warning: you need the float libraries for this
			dbgSerial.println("");
			dbgSerial.print("XferRatio: ");           // "-Wl,-u,vfscanf -lscanf_flt -lm" in platform.local.txt
			dbgSerial.print(argStr);
			dbgSerial.print(" ");
			dbgSerial.println(f);
			ratio[telescope][m].XferRatio = f;
			break;

		default:
			break;

		}
		PrintRatioValues(telescope);
	}
	*/
}


/**********************************************
  Handle EQG communications
***********************************************/
bool EQGRx(void) {
  if (EQGSerial.available() == 0)
    return false;
	digitalWrite(EQGLED, HIGH);
  while (EQGSerial.available() > 0) {
    EQGRxBuffer[EQGRxiPtr++] = EQGSerial.read();
    EQGRxiPtr &= EQGMASK;
  }
	digitalWrite(EQGLED, LOW);
  return true;
}

// Just put it in the output buffer
// Main loop handles transmission
void EQGTx(unsigned char data) {
  EQGTxBuffer[EQGTxiPtr++] = data;
  EQGTxiPtr &= EQGMASK;
}

