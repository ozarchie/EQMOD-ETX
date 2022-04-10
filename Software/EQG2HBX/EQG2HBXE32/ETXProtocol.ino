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

/********************************************************
  ETX Protocol related functions
  ==============================
 *********************************************************/

bool ETXState(unsigned char Motor) {
  long  distance;
  
  switch(axis[Motor].ETXMotorState) {

    case ETXIdle:
      break;
    
    case ETXCheckStartup:
			if (axis[Motor].ETXMotorStatus & MOVEAXIS) {              // Start moving
//jma				CheckAltFlip(Motor);

				dbgSerial.println(""); dbgSerial.print("ETXCheckStartup - Motor: "); dbgSerial.print(axis_name[Motor]); dbgSerial.print(" MOVING");

				if (axis[Motor].ETXMotorStatus & MOVESLEW) {
					axis[Motor].MotorControl |= SlewHBX;
					axis[Motor].ETXMotorState = ETXSlewMotor;
					if (axis[Motor].ETXMotorStatus & MOVEHIGH) {        // Use high speed multiplier?
						dbgSerial.print(" HIGH SLEW");										// Ramp up to HIGH SPEED
					}
					else {
						axis[Motor].ETXMotorState = ETXStepMotor;
						axis[Motor].MotorControl |= SpeedHBX;											// Use 0x01 command for next speed
						dbgSerial.print(" LOW  SLEW");
					}
				}
				else {                                                // GoTo or Low Speed Slew
					axis[Motor].MotorControl &= ~SlewHBX;
					axis[Motor].ETXMotorState = ETXCheckSpeed;					//  so just check speed
					dbgSerial.print(" GOTO");
				}

				if (axis[Motor].MotorControl & GoToHBX) {               	// Check GoTo?
					distance = axis[Motor].Target - axis[Motor].Position;   // Distance to target
					if (axis[Motor].ETXMotorStatus & MOVEDECR)              // If it is decreasing
						distance = TwosComplement(distance);
					dbgSerial.printf(" Distance: %6lX, Tgt: %6lX, Pos: %6lX", distance, axis[Motor].Target, axis[Motor].Position);
					if (axis[Motor].MotorControl & SlewHBX) {             // May need to slew for large changes
						axis[Motor].ETXMotorState = ETXSlewMotor;						// Slew to M-point
					}
					else {
						axis[Motor].ETXMotorState = ETXCheckSpeed;
						axis[Motor].TargetSpeed = axis[Motor].DEGREERATE1;  // Set initial speed for 'HIGH SPEED GOTO'
						if (distance < axis[Motor].OneDegree)								// Scale target speed depending on distance to target
							axis[Motor].TargetSpeed = (axis[Motor].TargetSpeed >> 1);	// 1/2
						if (distance < (axis[Motor].OneDegree >> 2))
							axis[Motor].TargetSpeed = (axis[Motor].TargetSpeed >> 1);	// 1/4
						axis[Motor].ETXSpeed = 0;                              // Starting from 0 
						dbgSerial.print(" GoTo-STEP");
					}
					if (distance < OffsetMax) {                           // Check for really small moves (< 16 steps)
						axis[Motor].ETXMotorState = ETXMotorEnd;            // Use Adjust offset
						dbgSerial.print(" GoTo-OFFSET");
					}
					if (distance > (axis[Motor].OneDegree << 3)) {        // Always slew for > 8 degrees
						axis[Motor].ETXMotorState = ETXSlewMotor;
						dbgSerial.print(" GoTo-SLEW");
					}
				}
			}
      break;
  
    case ETXSlewMotor:
//dbgSerial.println(""); dbgSerial.print("ETXSlewMotor Motor: "); dbgSerial.print(axis_name[Motor]); dbgSerial.print(" SLEW Cmd: ");
/* Andrew Johansen Melbourne Australia
All gotos are done in 2 phases
In the first phase, it calcs distance to "target minus a set time"
( ie it aims for a spot "behind" the target )
It then starts slewing, at Max speed first  if longer than a set distance.
It checks distance intermittently and recalcs speed based on distance to go.
When within a radius of about 40 arcsec it stops.
It then recalcs for a position a few clock seconds ahead of target
and does a second slew to finalise.
It also sets a timer when starting the second slew.
Once within about 10 arcsec of target, it stops until the timer trips
then restarts tracking. ie no hunting around etc at the end,
it just assumes it is correct.
JMA
In the ETX-arduino code, I do similar to what you have said previously.
1. send a high speed command to get somewhere close (originally the breakpoint location)
2. change to a low speed command to get closer (the maximum value of the offset command)
3. read the current position from the ETX and issue an offset command to get to the target

0x06/0x07    Move at fastest rate (reverse/forward)
0x01        Start moving, set speed and direction to the value specified in command
0x00        While moving, adjust speed (but not direction) to the value specified in command
0x02        Adjust position, while moving
0x05        Stop moving

0x08        Return change in position

When, moving the speed set by subsequent 0x00 commands, appears to be ~0.8 of the previous speed.
So the motor asymptotes down to a very low speed. And, at the end the 0x02 command adjusts the final position.

*/
      digitalWrite(ETXLED, HIGH);																		// Turn on the ETX telescope LED
      HBXSendCommand(Stop, Motor);                                  // Stop the motor
      if (axis[Motor].ETXMotorStatus & MOVEDECR)										// -ve i.e. -CCW
        axis[Motor].Command = SlewReverse;
      else
        axis[Motor].Command = SlewForward;
      HBXSendCommand(axis[Motor].Command, Motor);										// SLEW

      axis[Motor].EQGMotorStatus |= MOVEAXIS;												// Tell EQx
      axis[Motor].ETXSpeed = axis[Motor].DEGREERATE1;								// Set ETXSpeed as approx (SLEW=DEGREERATE1) for later speed checks
			axis[Motor].ETXMotorState = ETXCheckPosition;									// Preset position check
			
      if (axis[Motor].MotorControl & GoToHBX) {											// Check if slew was caused by a high speed long distance GoTo
        distance = axis[Motor].Target - axis[Motor].Position;       // Check Distance to target
        if (axis[Motor].ETXMotorStatus & MOVEDECR)                  // If it is decreasing
          distance = TwosComplement(distance);
        if (distance < axis[Motor].OneDegree) {                     // Change to GoTo if within one degree
          while(!(HBXSendCommand(Stop, Motor)));										// Stop the motor command
          axis[Motor].ETXMotorState = ETXStepMotor;
          axis[Motor].MotorControl |= SpeedHBX;											// Use 0x01 command for next speed
          axis[Motor].TargetSpeed = axis[Motor].DEGREERATE1;				// Set initial speed
          axis[Motor].ETXSpeed = axis[Motor].DEGREERATE1;
          axis[Motor].SpeedState = 0;
          }
      }
      if (axis[Motor].MotorControl & SlewHBX) {                     // Slewing to M-point
        axis[Motor].ETXSpeed = axis[Motor].DEGREERATE1;             // Indicate current speed (approx)
      	axis[Motor].ETXMotorState = ETXCheckSlowDown;               // Slew until SlowDown
      }

//*
dbgSerial.println(""); dbgSerial.print("ETXSlewMotor Motor: "); dbgSerial.print(axis_name[Motor]); 
dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
dbgSerial.print(", Cmd: "); dbgSerial.print(axis[Motor].Command, HEX);
dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
dbgSerial.print("->Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
dbgSerial.print(", Inc: "); dbgSerial.print(axis[Motor].Increment, HEX);
dbgSerial.print(", ETXSpeed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
dbgSerial.print(" TgtSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);
dbgSerial.print(" SpeedState: "); dbgSerial.println(SpeedStateDesc[(axis[Motor].SpeedState)]);
//*/
			break;
  
    case ETXStepMotor:
//*
dbgSerial.println(""); dbgSerial.print("ETXStepMotor Motor: "); dbgSerial.print(axis_name[Motor]);
dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
dbgSerial.print(", Cmd: "); dbgSerial.print(axis[Motor].Command, HEX);
dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
dbgSerial.print("->Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
dbgSerial.print(", Inc: "); dbgSerial.print(axis[Motor].Increment, HEX);
dbgSerial.print(" Speed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
dbgSerial.print(" TargetSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);
dbgSerial.print(" SpeedState: "); dbgSerial.println(SpeedStateDesc[(axis[Motor].SpeedState)]);
//*/

      digitalWrite(ETXLED, HIGH);      											// Turn on the LED
      if (axis[Motor].MotorControl & SpeedHBX) {            // Stepping, High or Low speed
        axis[Motor].Command = SpeedStart;                   // Use SpeedStart to start motion
				axis[Motor].MotorControl &= ~SpeedHBX;                // Clear flag
			}
      else
        axis[Motor].Command = SpeedChnge;                   // Use SpeedChnge once started
     
      // Set the speed, and direction
      // ----------------------------
      P1 = axis[Motor].TargetSpeed;
      if (axis[Motor].ETXMotorStatus & MOVEDECR)            // If negative, change P
        P1 = TwosComplement(P1);                            //  to 2's complement
      axis[Motor].HBXP1 = (P1 >> 16) & 0xFF;								// Initialize command bytes
      axis[Motor].HBXP2 = (P1 >>  8) & 0xFF;
      axis[Motor].HBXP3 = P1 & 0xFF;
			if (HBXSendCommand(axis[Motor].Command, Motor)) {     // Send Command, check OK?
				HBXSend3Bytes(Motor);																// Send the speed
				axis[Motor].EQGMotorStatus |= MOVEAXIS;             // Tell EQx
				axis[Motor].ETXSpeed = axis[Motor].TargetSpeed;
			}
			else break;

      axis[Motor].ETXMotorState = ETXCheckSpeed;						// Preset Checkspeed, & if needed, make sure we are up to target speed
      if (axis[Motor].MotorControl & GoToHBX) {							// If it is a GoTo and up to speed, check position
        if (axis[Motor].ETXSpeed == axis[Motor].TargetSpeed)
          axis[Motor].ETXMotorState = ETXCheckPosition;
      }
      else if (axis[Motor].ETXSpeed == 0) {                 // Stop issued
        axis[Motor].ETXMotorState = ETXStopMotor;
      }
			else if ((axis[Motor].MotorControl & SlewHBX) &&
							 !(axis[Motor].MotorControl & GoToHBX)) { // SLEW without GoTo = speed change
          axis[Motor].ETXMotorState = ETXIdle;
      }
      else if (axis[Motor].ETXSpeed == axis[Motor].TargetSpeed) {  // Else slewing at speed
        axis[Motor].ETXMotorState = ETXCheckPosition;
      }

    break;

    case ETXCheckSlowDown:
      // Check if Slowdown reached
      // Calculate absolute distance to slowdown
      // ---------------------------------------

///*
dbgSerial.println(""); dbgSerial.print("ETXCheckSlowDown Motor: "); dbgSerial.print(axis_name[Motor]); 
dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
dbgSerial.print(", Cmd: "); dbgSerial.print(axis[Motor].Command, HEX);
dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
dbgSerial.print("  Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
dbgSerial.print(",  SD: "); dbgSerial.print(axis[Motor].SlowDown, HEX);
dbgSerial.print(", ETXSpeed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
dbgSerial.print(" TgtSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);
//*/
//    distance = axis[Motor].SlowDown - axis[Motor].Position;
    distance = (axis[Motor].Target - 0x1000) - axis[Motor].Position;    // Distance to target
    if (axis[Motor].Target < axis[Motor].Position)                // If it is decreasing
      distance = TwosComplement(distance);

//dbgSerial.print(" distance: ");
//dbgSerial.print(distance, HEX);

    if (distance <= 0) {
      while(!(HBXSendCommand(Stop, Motor)));                        // Stop the motor
//      HBXSendCommand(Stop, Motor);                                 // Stop the motor
      axis[Motor].TargetSpeed = (axis[Motor].SIDEREALRATE << 7);   // target is 128xSidereal
      axis[Motor].ETXMotorState = ETXCheckSpeed;
      axis[Motor].MotorControl &= ~SlewHBX;                        // Clear slew bit (if it was set)
      axis[Motor].MotorControl |= SpeedHBX;                        // Use 0x01 command for first slow-down
    }
    break;
      
    case ETXCheckSpeed:

/*
ETXSlew1        1               // 1  x sidereal (0.25 arc-min/sec or 0.0042°/sec) 
ETXSlew2        2               // 2  x sidereal (0.50 arc-min/sec or 0.0084°/sec)
ETXSlew3        8               // 8  x sidereal (   2 arc-min/sec or 0.0334°/sec)
ETXSlew4        16              // 16 x sidereal (   4 arc-min/sec or 0.0669°/sec)
ETXSlew5        64              // 64 x sidereal (  16 arc-min/sec or 0.2674°/sec)
ETXSlew6        120             // 30  arc-min/sec or 0.5°/sec
ETXSlew7        240             // 60  arc-min/sec or 1.0°/sec
ETXSlew8        600             // 150 arc-min/sec or 2.5°/sec
ETXSlew9       1080             // 270 arc-min/sec or 4.5°/sec
*/

///*
dbgSerial.println(""); dbgSerial.print("ETXCheckSpeed Motor: "); dbgSerial.print(axis_name[Motor]); 
dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
dbgSerial.print("  Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
dbgSerial.print(", Inc: "); dbgSerial.print(axis[Motor].Increment, HEX);
dbgSerial.print(", ETXSpeed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
dbgSerial.print(" TgtSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);
//*/
// Speeding Up
// ===========
      axis[Motor].ETXMotorState = ETXStepMotor;                   // Preset set speed as next action
// Ramp up to speed      
      if ((axis[Motor].TargetSpeed != 0) && (axis[Motor].TargetSpeed > axis[Motor].ETXSpeed)) {
        if ((axis[Motor].TargetSpeed - axis[Motor].ETXSpeed) > (axis[Motor].SIDEREALRATE << 6)) {  // 64x sidereal
          axis[Motor].ETXSpeed += ((axis[Motor].TargetSpeed - axis[Motor].ETXSpeed) >> 1);   // Ramp up approx .5 difference
          axis[Motor].MotorControl &= ~SpeedHBX;                  // Use 0x00 command (changes speed, but not direction)
        }
        else {
          axis[Motor].ETXSpeed = axis[Motor].TargetSpeed;            
          while(!(HBXSendCommand(Stop, Motor)));                 // Stop the motor command
          axis[Motor].MotorControl |= SpeedHBX;                  // Use 0x01 command (changes speed and direction)
        }
      }
// Ramp down to speed
      else if ((axis[Motor].TargetSpeed != 0) && (axis[Motor].ETXSpeed > axis[Motor].TargetSpeed)) {
        axis[Motor].ETXSpeed -= ((axis[Motor].ETXSpeed - axis[Motor].TargetSpeed) >> 2);   // Approx .75
        if ((axis[Motor].ETXSpeed - axis[Motor].TargetSpeed) <= (axis[Motor].SIDEREALRATE << 7)) {
          axis[Motor].ETXSpeed = axis[Motor].TargetSpeed;            // Close enough at 128x sidereal, so set the speed
//          while(!(HBXSendCommand(Stop, Motor)));                  // Stop the motor command
          axis[Motor].MotorControl &= ~SpeedHBX;                   // Use 0x00 command (changes speed, but not direction)
        }
      }
// Ramp down to stop
      else if ((axis[Motor].TargetSpeed == 0) && (axis[Motor].ETXSpeed != 0)) {
        if (axis[Motor].ETXMotorStatus & MOVESLEW) {
          axis[Motor].ETXMotorState = ETXStopMotor;
        }
        else if (axis[Motor].ETXSpeed >= (axis[Motor].SIDEREALRATE << 7)) {   // Ramp down to 128 sidereal
          axis[Motor].ETXSpeed -= (axis[Motor].ETXSpeed >> 2);              // Approximately .75
          while(!(HBXSendCommand(Stop, Motor)));                      			// Stop the motor command
          axis[Motor].MotorControl |= SpeedHBX;                       			// Use 0x01 command (changes speed and direction)
        }
        else
          axis[Motor].ETXMotorState = ETXStopMotor;             // OK, Stop the motor
      }
// Already stopped
      else if ((axis[Motor].TargetSpeed == 0) && (axis[Motor].ETXSpeed == 0)) {
				axis[Motor].ETXMotorStatus |= MOVESLEW;                   	 // ETX Set slewing mode
				axis[Motor].ETXMotorStatus &= ~MOVEHIGH;                     //  and speed
				axis[Motor].ETXMotorStatus &= ~MOVEAXIS;                     // Clear the motor moving flag 
				axis[Motor].MotorControl &= ~GoToHBX;                        // Clear the GoTo flag
				axis[Motor].MotorControl &= ~SlewHBX;                        //  and the slew flag
				axis[Motor].EQGMotorStatus |= MOVESLEW;                      // EQG Set slewing mode
				axis[Motor].EQGMotorStatus &= ~MOVEHIGH;                     //  and speed
				axis[Motor].EQGMotorStatus &= ~MOVEAXIS;                     // Clear the motor moving flag 
				axis[Motor].EQGMotorStatus &= ~MOVEDECR;                     // Clear the direction flag 
				axis[Motor].ETXMotorState = ETXCheckStartup;		
			}
// Switch to position check, when we are at speed - check done in ETXStepMotor
      
//dbgSerial.print(" oSpeed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
//dbgSerial.print(" oTargetSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);

      break;
  
    case ETXCheckPosition:
      // Check if Target acquired
      // ------------------------

      // Calculate absolute distance to target
      // -------------------------------------

///*
if ( ((Motor == MotorAlt) && ((pAltCount % 10) == 0)) ||
		 ((Motor == MotorAz) && ((pAzCount % 10) == 0)) ) {
			dbgSerial.println(""); dbgSerial.print("ETXCheckPosition Motor: "); dbgSerial.print(axis_name[Motor]); 
			dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
			dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
			dbgSerial.print("->Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
			dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
			dbgSerial.print(", Inc: "); dbgSerial.print(axis[Motor].Increment, HEX);
			dbgSerial.print(", ETXSpeed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
			dbgSerial.print(" TgtSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);
		}
		pAzCount++;
		pAltCount++;
//*/
				if ( (axis[Motor].MotorControl & SlewHBX) &&
								!(axis[Motor].MotorControl & GoToHBX) )  // SLEW without GoTo = speed change
						break;
      

        distance = axis[Motor].Target - axis[Motor].Position;       // Distance to target
        if (axis[Motor].ETXMotorStatus & MOVEDECR)                  // If it is decreasing
          distance = TwosComplement(distance);

        if (distance == 0)
          axis[Motor].ETXMotorState = ETXMotorEnd;
        else if (distance > 0) {
          // Start to slow motor if getting near target
          // ------------------------------------------
          if (distance <= OffsetMax) {
            axis[Motor].ETXMotorState = ETXMotorEnd;                    // Stop motor, set offset
            axis[Motor].SpeedState = 0;
          }
          else if (axis[Motor].SpeedState == 3) {
            axis[Motor].TargetSpeed = (axis[Motor].SIDEREALRATE) << 2;
            axis[Motor].MotorControl &= ~SpeedHBX;                      // Use 0x00 command
            axis[Motor].ETXMotorState = ETXStepMotor;                   // Change speed
            axis[Motor].SpeedState += 1;
          }
          else if ((distance <= 0x200) && (axis[Motor].SpeedState == 2)) {
            axis[Motor].TargetSpeed = axis[Motor].ETXSpeed >> 2;           // 1/16
            axis[Motor].MotorControl &= ~SpeedHBX;                      // Use 0x00 command
            axis[Motor].ETXMotorState = ETXStepMotor;                   // Change speed
            axis[Motor].SpeedState += 1;
          }
          else if ((distance <= 0x400) && (axis[Motor].SpeedState == 1)) {
            axis[Motor].TargetSpeed = axis[Motor].ETXSpeed >> 1;           // 1/4
            axis[Motor].MotorControl &= ~SpeedHBX;                      // Use 0x00 command
            axis[Motor].ETXMotorState = ETXStepMotor;                   // Change speed
            axis[Motor].SpeedState += 1;
          }
          else if ((distance <= 0x800) && (axis[Motor].SpeedState == 0)) {
            axis[Motor].TargetSpeed = axis[Motor].ETXSpeed >> 1;           // 1/2
            while(!(HBXSendCommand(Stop, Motor)));                      // Stop the motor command
            axis[Motor].MotorControl |= SpeedHBX;                       // Use 0x01 command
            axis[Motor].ETXMotorState = ETXStepMotor;                   // Change speed
            axis[Motor].SpeedState += 1;
          }
        }
        else {
          if ((TwosComplement(distance)) > OffsetMax) {                // Not sure how good offset is!
            // Motor has over-shot the target
            // ------------------------------
            if (axis[Motor].ETXMotorStatus & MOVEDECR)                // ETX -> change direction
              axis[Motor].ETXMotorStatus &= ~MOVEDECR;
            else
              axis[Motor].ETXMotorStatus |= MOVEDECR;                        
            while(!(HBXSendCommand(Stop, Motor)));                    // Stop the motor command
            axis[Motor].MotorControl |= SpeedHBX;                     // Use 0x01 command
            axis[Motor].TargetSpeed = (axis[Motor].SIDEREALRATE) << 2;
            axis[Motor].ETXMotorState = ETXStepMotor;                 // Change ETX speed
          }
          else{
            axis[Motor].ETXMotorState = ETXMotorEnd;									// Stop motor, set offset
            axis[Motor].SpeedState = 0;
          }          
        }

      break;
  
    case ETXStopMotor:

      while(!(HBXSendCommand(Stop, Motor)));                    // Stop the motor
      axis[Motor].ETXMotorStatus |= MOVESLEW;                   // ETX Set slewing mode
      axis[Motor].ETXMotorStatus &= ~MOVEHIGH;                     //  and speed
      axis[Motor].ETXMotorStatus &= ~MOVEAXIS;                     // Clear the motor moving flag 
      axis[Motor].MotorControl &= ~GoToHBX;                        // Clear the GoTo flag
      axis[Motor].MotorControl &= ~SlewHBX;                        //  and the slew flag
      axis[Motor].EQGMotorStatus |= MOVESLEW;                      // EQG Set slewing mode
      axis[Motor].EQGMotorStatus &= ~MOVEHIGH;                     //  and speed
      axis[Motor].EQGMotorStatus &= ~MOVEAXIS;                     // Clear the motor moving flag 
      axis[Motor].EQGMotorStatus &= ~MOVEDECR;                     // Clear the direction flag 
			axis[Motor].TargetSpeed = 0;
			axis[Motor].ETXSpeed = 0;
      axis[Motor].ETXMotorState = ETXCheckStartup;
					
//jma			CheckAltFlipState();
			dbgSerial.println(""); dbgSerial.print("ETXStopMotor Motor: ");  dbgSerial.print(axis_name[Motor]);
			dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
			dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
			dbgSerial.print("->Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
			dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
			dbgSerial.print(", Inc: "); dbgSerial.print(axis[Motor].Increment, HEX);
			dbgSerial.print(", ETXSpeed: "); dbgSerial.print(axis[Motor].ETXSpeed, HEX);
			dbgSerial.print(" TgtSpeed: "); dbgSerial.print(axis[Motor].TargetSpeed, HEX);
			
      break;
      
    case ETXMotorEnd:
///*
dbgSerial.println(""); dbgSerial.print("ETXMotorEnd Motor: "); dbgSerial.print(axis_name[Motor]); 
dbgSerial.print(" <sts: "); dbgSerial.print(axis[Motor].ETXMotorStatus, HEX); dbgSerial.print("> ");
dbgSerial.print(", Pos: "); dbgSerial.print(axis[Motor].Position, HEX);
dbgSerial.print("->Tgt: "); dbgSerial.print(axis[Motor].Target, HEX);
dbgSerial.print("  Chg: "); dbgSerial.print(axis[Motor].Position-axis[Motor].Target, HEX);
dbgSerial.print(", Inc: "); dbgSerial.print(axis[Motor].Increment, HEX);
//*/
      digitalWrite(ETXLED, LOW);           											// Turn off the ETX LED
      distance = axis[Motor].Target - axis[Motor].Position;     // Distance to target
      if (axis[Motor].Target < axis[Motor].Position)          	// If it is decreasing
        distance = TwosComplement(distance);
      
      if (distance == 0) {
        axis[Motor].ETXMotorState = ETXStopMotor;               // Stop the motor
      }
      else {
        axis[Motor].HBXP1 = (distance >> 8) & 0xFF;              // Initialize offset bytes
        axis[Motor].HBXP2 = distance & 0xFF;
        axis[Motor].Command = SetOffset;
        if (HBXSendCommand(axis[Motor].Command, Motor))   // Command OK?
          HBXSend2Bytes(Motor);                           // Send the offset

///*
dbgSerial.print(" OFFSET");
dbgSerial.print(" "); dbgSerial.print(axis[Motor].Command, HEX);
dbgSerial.print(" "); dbgSerial.print(axis[Motor].HBXP1, HEX);
dbgSerial.print(" "); dbgSerial.print(axis[Motor].HBXP2, HEX);
dbgSerial.print(" "); dbgSerial.print(axis[Motor].HBXP3, HEX);
//*/
      }  
      axis[Motor].Position = axis[Motor].Target;
      axis[Motor].MotorControl &= ~GoToHBX;                  // Clear the flag
      axis[Motor].ETXMotorState = ETXStopMotor;
      break;
      
    default:  
      break;
  }
  return(true);
}

// Motor functions

bool HBXGetStatus(unsigned char Motor) {
  if (!HBXSendCommand(GetStatus, Motor)) {
		dbgSerial.println(""); dbgSerial.print("HBXGetStatus Motor: "); dbgSerial.print(axis_name[Motor]); dbgSerial.println(" Cmd Fail");
		return(false);
  }
	HBXGet3Bytes(Motor);     
  P1 = (axis[Motor].HBXP1 << 8);							// Update calculated position
  P1 |= axis[Motor].HBXP2;										// Convert to 16bits
  if (axis[Motor].HBXP1 & 0x80)
    P1 |= 0xffff0000;													// Sign extend HBXP1 for 2s complement
  axis[Motor].Position += P1;

	// Wrap the position if above (+12H or +90deg)
	if (axis[Motor].Position >= (ETX_CENTRE + (axis[Motor].aVALUE >> 1)))
		axis[Motor].Position -= axis[Motor].aVALUE;
	// Wrap the position if below (-12H or -90deg)
	if (axis[Motor].Position <= (ETX_CENTRE - (axis[Motor].aVALUE >> 1)))
		axis[Motor].Position += axis[Motor].aVALUE;

	if (StatusCount++ > 51) {
		StatusCount = 0;
		dbgSerial.print(axis_name[MotorAlt]); dbgSerial.print(": "); dbgSerial.print(axis[MotorAlt].Position, HEX); dbgSerial.print(", ");
		dbgSerial.print(axis_name[MotorAz]); dbgSerial.print(": "); dbgSerial.println(axis[MotorAz].Position, HEX);
	}
	return(true);
}

bool HBXGet2Status(void) {
  int i;
  do {
    i = 0;       
		if (HBXGetStatus(AzMotor))	i += 1;
		TimerDelayuS(HBXBitTime << 2);

    if (HBXGetStatus(AltMotor))	i += 1;
		TimerDelayuS(HBXBitTime << 2);

  } while (i < 2);
  return(true);  
}

// Calibrate motor LED
// ===================
void CalibrateLEDs(void) {


	while (!(HBXSendCommand(Stop, AzMotor)));			// Stop both motors
	TimerDelaymS(100);
	while (!(HBXSendCommand(Stop, AltMotor)));
	TimerDelaymS(100);

	while (!(HBXGetStatus(AzMotor)))
		TimerDelaymS(100);
	while (!(HBXGetStatus(AltMotor)))
		TimerDelaymS(100);

	HBXSendCommand(SleepHBX, AzMotor);						// Issue Sleep 
	TimerDelaymS(100);
	HBXSendCommand(CalibrateLED, AzMotor);				// Issue Calibrate LED
	TimerDelaymS(4000);

	HBXSendCommand(SleepHBX, AltMotor);
	TimerDelaymS(100);
	HBXSendCommand(CalibrateLED, AltMotor);
	TimerDelaymS(4000);

	HBXSendCommand(GetLEDI, AzMotor); HBXGetByte(AzMotor);
	HBXSendCommand(GetLEDI, AltMotor);	HBXGetByte(AltMotor);

	while (!(HBXGetStatus(AzMotor)))
		TimerDelaymS(100);
	while (!(HBXGetStatus(AltMotor)))
		TimerDelaymS(100);

	// Read the calibration
	dbgSerial.println(""); dbgSerial.print("Read LEDs - AzMotor: ");
	if (HBXSendCommand(GetLEDI, AzMotor))
		axis[AzMotor].HBXLEDI = HBXGetByte(AzMotor);
	dbgSerial.print(axis[AzMotor].HBXLEDI);
	dbgSerial.print(", AltMotor: ");
	if (HBXSendCommand(GetLEDI, AltMotor))
		axis[AltMotor].HBXLEDI = HBXGetByte(AltMotor);
	dbgSerial.print(axis[AltMotor].HBXLEDI);

	// Save it to preferences
	// Prefences.begin must be set by the caller
	preferences.putUChar("AzLEDI", axis[AzMotor].HBXLEDI);
	preferences.putUChar("AltLEDI", axis[AltMotor].HBXLEDI);
}

void WaitForMotors(void) {
// GetLED commands always return a vaild value - motors not online until this happens
// "Valid" values are not 0 and not 0xFF for Az, Alt. (exception here is if LEDRA || LEDAlt == 0xff)
// 
	bool PAz = false;
	bool PAlt = false;
	do {
		if (!PAz) {
			if (HBXSendCommand(GetLEDI, AzMotor))
				axis[AzMotor].HBXLEDI = HBXGetByte(AzMotor);
			dbgSerial.print("Az  HBXLEDI: "); dbgSerial.println(axis[AzMotor].HBXLEDI);
			TimerDelaymS(CMNDTIME);
			if ((axis[AzMotor].HBXLEDI != 0) && (axis[AzMotor].HBXLEDI != 0xFF))
				PAz = true;
		}
		if (!PAlt) {
			if (HBXSendCommand(GetLEDI, AltMotor))
				axis[AltMotor].HBXLEDI = HBXGetByte(AltMotor);
			TimerDelaymS(CMNDTIME);
			dbgSerial.print("Alt HBXLEDI: "); dbgSerial.println(axis[AltMotor].HBXLEDI);
			if ((axis[AltMotor].HBXLEDI != 0) && (axis[AltMotor].HBXLEDI != 0xFF))
				PAlt = true;
		}
		// Attempt to reset Motor Controller
		if (!PAz) {
			HBXMotorReset(AzMotor);
		}
		if (!PAlt) {
			HBXMotorReset(AltMotor);
		}
  } while ((!PAz) || (!PAlt));
}
  
void AzInitialise(unsigned char scope) {
// Telescope specific
// telescope steps
  axis[AzMotor].Vanes = ratio[scope][AzMotor-1].Vanes; 
  axis[AzMotor].GbxRatio = ratio[scope][AzMotor-1].GbxRatio; 
  axis[AzMotor].XferRatio = ratio[scope][AzMotor-1].XferRatio; 
  axis[AzMotor].WormTeeth = ratio[scope][AzMotor-1].WormTeeth;
// EQMOD
  axis[AzMotor].aVALUE = axis[AzMotor].Vanes * (float)4 * axis[AzMotor].GbxRatio * axis[AzMotor].XferRatio * axis[AzMotor].WormTeeth;
  axis[AzMotor].MeadeRatio = (float)axis[AzMotor].aVALUE / ArcSecs360;   // Count for one arcsec
  axis[AzMotor].bVALUE = (MeadeSidereal * axis[AzMotor].MeadeRatio) * ((float)axis[AzMotor].aVALUE / SIDEREALSECS) * SiderealArcSecs;
  axis[AzMotor].BASERATE = axis[AzMotor].bVALUE / axis[AzMotor].MeadeRatio;
  axis[AzMotor].SIDEREALRATE = MeadeSidereal * axis[AzMotor].MeadeRatio;
  axis[AzMotor].SOLARRATE = axis[AzMotor].SIDEREALRATE * SOLARSECS / SIDEREALSECS;
  axis[AzMotor].LUNARRATE = axis[AzMotor].SIDEREALRATE * LUNARSECS / SIDEREALSECS;
  axis[AzMotor].DEGREERATE1 = axis[AzMotor].SIDEREALRATE * ETXSlew7;
  axis[AzMotor].PEC = axis[AzMotor].aVALUE / axis[AzMotor].WormTeeth;

// ETX
  axis[AzMotor].HBXP1 = 0x00;             
  axis[AzMotor].HBXP2 = 0x00;
  axis[AzMotor].HBXP3 = 0x00;
  axis[AzMotor].HBXP4 = 0x00;
	axis[AzMotor].ETXSpeed = 0x00;
	axis[AzMotor].SpeedState = 0x00;
	axis[AzMotor].TargetSpeed = axis[AzMotor].SIDEREALRATE;
  axis[AzMotor].Position = ETX_CENTRE;								// ETX RA initially at 0 hours
  axis[AzMotor].OneDegree = axis[AzMotor].aVALUE / (float)360;    	// Distance for one degree
  axis[AzMotor].Target = axis[AzMotor].Position;
  axis[AzMotor].DirnSpeed = 0x000;
  axis[AzMotor].ETXMotorStatus = MOVESLEW;
  axis[AzMotor].EQGMotorStatus = MOVESLEW;
  axis[AzMotor].ETXMotorState = ETXCheckStartup;
}

void AltInitialise(unsigned char scope) {
// Telescope specific
// telescope steps
  axis[AltMotor].Vanes = ratio[scope][AltMotor-1].Vanes; 
  axis[AltMotor].GbxRatio = ratio[scope][AltMotor-1].GbxRatio; 
  axis[AltMotor].XferRatio = ratio[scope][AltMotor-1].XferRatio; 
  axis[AltMotor].WormTeeth = ratio[scope][AltMotor-1].WormTeeth;
// EQMOD 
  axis[AltMotor].aVALUE = axis[AltMotor].Vanes * (float)4 * axis[AltMotor].GbxRatio * axis[AltMotor].XferRatio * axis[AltMotor].WormTeeth;
  axis[AltMotor].MeadeRatio = axis[AltMotor].aVALUE / ArcSecs360;   // Count for one arcsec
  axis[AltMotor].bVALUE = (MeadeSidereal * axis[AltMotor].MeadeRatio) * (axis[AltMotor].aVALUE / SIDEREALSECS) * SiderealArcSecs;  
	axis[AltMotor].BASERATE = axis[AltMotor].bVALUE / axis[AltMotor].MeadeRatio;
	axis[AltMotor].SIDEREALRATE = MeadeSidereal * axis[AltMotor].MeadeRatio;
	axis[AltMotor].SOLARRATE = axis[AltMotor].SIDEREALRATE * SOLARSECS / SIDEREALSECS;
  axis[AltMotor].LUNARRATE = axis[AltMotor].SIDEREALRATE * LUNARSECS / SIDEREALSECS;
  axis[AltMotor].DEGREERATE1 = axis[AltMotor].SIDEREALRATE * ETXSlew7;
  axis[AltMotor].PEC = axis[AltMotor].aVALUE / axis[AltMotor].WormTeeth;
	axis[AltMotor].EQGMotorStatus = MOVESLEW;
// ETX
  axis[AltMotor].HBXP1 = 0x00;             
  axis[AltMotor].HBXP2 = 0x00;
  axis[AltMotor].HBXP3 = 0x00;
	axis[AltMotor].HBXP4 = 0x00;
	axis[AltMotor].ETXSpeed = 0x00;
	axis[AltMotor].SpeedState = 0x00;
	axis[AltMotor].TargetSpeed = axis[AzMotor].SIDEREALRATE;
	axis[AltMotor].Position = ETX_CENTRE;											// ALTAZ at 0
	if (mounttype == 1) axis[AltMotor].Position += (axis[AltMotor].aVALUE >> 2);	// GEM Initially at +90 degrees
	axis[AltMotor].OneDegree = axis[AltMotor].aVALUE / (float)360;					// Distance for one degree
  axis[AltMotor].Target = axis[AltMotor].Position;
  axis[AltMotor].DirnSpeed = 0x000;
  axis[AltMotor].ETXMotorStatus = MOVESLEW;

  axis[AltMotor].ETXMotorState = ETXCheckStartup;  
}  

/*
// Check if ETX mount will hit fork physical limits
// When used as GEM, ETX mounts cannot go past vertical
// This means E in Sthn, W in Nthn, for East side pointing West
// So rotate 180 degrees in Az and flip the Alt destination
*/
void CheckAltFlip(unsigned char Motor) {
	long  distance;
	distance = axis[Motor].Target - axis[Motor].Position;   // Distance to target
	if ((scopetype < 3) && (Motor == AltMotor)) {																		// ETXn0 series
		long p;
		if (distance > 0) {
			// GoTo
			if (axis[AltMotor].Target >= (ETX_CENTRE + (axis[AltMotor].aVALUE >> 2))) {		// E[Sthn] (or W[Nthn])
				dbgSerial.println(""); dbgSerial.print("Flip: target "); dbgSerial.print(axis[AltMotor].Target);																																													// Set Target
				p = axis[AltMotor].Target - (ETX_CENTRE + (axis[AltMotor].aVALUE >> 2));		// Offset from EQG midpoint
				axis[AltMotor].Target -= (p << 1);																					// Swap to other side
				distance = axis[Motor].Target - axis[Motor].Position;												// Distance to target
				dbgSerial.print(", new  "); dbgSerial.print(axis[AltMotor].Target);
				// Flip direction
				if (axis[AltMotor].ETXMotorStatus & MOVEDECR) axis[AltMotor].ETXMotorStatus &= ~MOVEDECR;
				else axis[AltMotor].ETXMotorStatus |= MOVEDECR;
				// Set SlowDown
				p = axis[AltMotor].SlowDown - (ETX_CENTRE + (axis[AltMotor].aVALUE >> 2));
				axis[AltMotor].SlowDown -= (p << 1);
				dbgSerial.print(", Slowdown  "); dbgSerial.print(p);
				if (axis[AltMotor].Flip == NORMAL) {
					axis[AltMotor].Flip = FLIP;
					dbgSerial.print(", Type  "); dbgSerial.print("ALREADY FLIPPED");
				}
				else {
					dbgSerial.print(", Type  "); dbgSerial.print("NORMAL");
				}
			}
			else {
				if (axis[AltMotor].Flip == FLIPPED) {
					axis[AltMotor].Flip = UNFLIP;
					dbgSerial.print(", Action  "); dbgSerial.print("UNFLIP");
				}
				else {
					dbgSerial.print(", Action  "); dbgSerial.print("NOTHING");
				}
			}
		}
		else {
			// SLEW
			if (axis[AltMotor].ETXMotorStatus & MOVEDECR) {
				axis[AltMotor].ETXMotorStatus &= ~MOVEDECR;
				if (axis[AltMotor].Flip == NORMAL) {
					axis[AltMotor].Flip = FLIP;
					dbgSerial.print(", Action  "); dbgSerial.print("FLIP");
				}
				else {
					dbgSerial.print(", Action  "); dbgSerial.print("NOTHING");
				}
			}
			if (axis[AltMotor].Flip == FLIPPED) {
				axis[AltMotor].Flip = UNFLIP;
				dbgSerial.print(", Action  "); dbgSerial.print("FLIP");
			}
			else {
				dbgSerial.print(", Action  "); dbgSerial.print("NOTHING");
			}
		}
	}
}

void CheckAltFlipState(void) {
	if ((axis[AltMotor].Flip == FLIPPING) || (axis[AltMotor].Flip == UNFLIPPING)) {
		dbgSerial.println(""); dbgSerial.print("Stop Motor:  ");
		if (axis[AltMotor].Flip == FLIPPING) {
			axis[AltMotor].Flip = FLIPPED; dbgSerial.print("FLIPPING");
		}
		else {
			axis[AltMotor].Flip = NORMAL; dbgSerial.print("NORMAl");
		}
	}
}

void CheckAltFlipReqd(void) {
	if (axis[AltMotor].ETXMotorState == ETXIdle) {
		if ((axis[AltMotor].Flip == FLIP) || (axis[AltMotor].Flip == UNFLIP)) {
			if (axis[AltMotor].Flip == FLIP) axis[AltMotor].Flip = FLIPPING;
			else axis[AltMotor].Flip = UNFLIPPING;
			// Rotate 180 degrees in RA
			if (axis[AzMotor].Target > ETX_CENTRE) axis[AzMotor].Target -= (axis[AzMotor].aVALUE >> 1);
			else axis[AzMotor].Target += (axis[AzMotor].aVALUE >> 1);
			axis[AzMotor].ETXMotorStatus |= (MOVEAXIS || MOVEHIGH);
			axis[AzMotor].ETXMotorStatus &= ~MOVESLEW;
			axis[AzMotor].MotorControl |= GoToHBX;
			CheckETXState(AzMotor);
		}
	}
}

void PrintHbxValues(unsigned char Motor) {
  if (Motor == AzMotor) 
    dbgSerial.println("AzMotor");
  else 
    dbgSerial.println("AltMotor");

  dbgSerial.print("Vanes "); dbgSerial.print(axis[Motor].Vanes); 
  dbgSerial.print(", GbxRatio "); dbgSerial.print(axis[Motor].GbxRatio,4); 
  dbgSerial.print(", XferRatio "); dbgSerial.print(axis[Motor].XferRatio,4); 
  dbgSerial.print(", WormTeeth "); dbgSerial.println(axis[Motor].WormTeeth); 
  dbgSerial.print("MeadeRatio "); dbgSerial.print(axis[Motor].MeadeRatio,6);
  dbgSerial.print(", MeadeSidereal "); dbgSerial.println(MeadeSidereal,4);
  
  dbgSerial.print("aVALUE 0x"); dbgSerial.print(axis[Motor].aVALUE, HEX);
  dbgSerial.print(", bVALUE 0x"); dbgSerial.print(axis[Motor].bVALUE, HEX); 
  dbgSerial.print(", PEC 0x"); dbgSerial.println(axis[Motor].PEC, HEX);

	dbgSerial.print("BASERATE 0x"); dbgSerial.print(axis[Motor].BASERATE, HEX);
	dbgSerial.print(", SIDEREALRATE 0x"); dbgSerial.print(axis[Motor].SIDEREALRATE, HEX);
	dbgSerial.print(", SOLARRATE 0x"); dbgSerial.print(axis[Motor].SOLARRATE, HEX);
  dbgSerial.print(", LUNARRATE 0x"); dbgSerial.print(axis[Motor].LUNARRATE, HEX);
  dbgSerial.print(", DEGREERATE1 0x"); dbgSerial.println(axis[Motor].DEGREERATE1, HEX);

  dbgSerial.print("One DEGREE 0x"); dbgSerial.println(axis[Motor].OneDegree, HEX);
  dbgSerial.println("");
}

void PrintRatioValues(unsigned char scope) {
  int j;
    for (j = 0; j < 2; j++) {
      if (j == 0) 
        dbgSerial.print("AzMotor:  ");
      else 
        dbgSerial.print("AltMotor: ");
      dbgSerial.print("Vanes "); dbgSerial.print(ratio[scope][j].Vanes); 
      dbgSerial.print(", GbxRatio "); dbgSerial.print(ratio[scope][j].GbxRatio,4); 
      dbgSerial.print(", XferRatio "); dbgSerial.print(ratio[scope][j].XferRatio,4); 
      dbgSerial.print(", WormTeeth "); dbgSerial.print(ratio[scope][j].WormTeeth);
			float r = (ratio[scope][j].Vanes * (float) 4 * ratio[scope][j].GbxRatio * ratio[scope][j].XferRatio * ratio[scope][j].WormTeeth) / (float) 1296000;
      dbgSerial.print(", MeadeRatio "); dbgSerial.println(r, 6);      
    }
}

void HBXPrintStatus(unsigned char Motor) {
	axis[Motor].HBXCount = 0;
	if ((axis[Motor].Command != GetStatus) || (axis[Motor].HBXP1 | axis[Motor].HBXP2 | axis[Motor].HBXP3 | axis[Motor].HBXP4) || axis[Motor].PrintStatus0) {

		/*		dbgSerial.println("");
				dbgSerial.print("Motor: ");
				dbgSerial.print(axis_name[Motor]);
				dbgSerial.print(", Cmnd: ");
				dbgSerial.print(axis[Motor].Command, HEX);
				dbgSerial.print(" -  ");
		 */
		switch (axis[Motor].Command) {
		case SpeedChnge:
			//            dbgSerial.print("SpeedChnge ");
			axis[Motor].HBXCount = 3;
			break;
		case SpeedStart:
			//            dbgSerial.print("SpeedStart ");
			axis[Motor].HBXCount = 3;
			break;
		case SetOffset:
			//            dbgSerial.print("SetOffset ");
			axis[Motor].HBXCount = 4;
			break;
		case SetLEDI:
			//            dbgSerial.print("SetLEDI ");
			axis[Motor].HBXCount = 1;
			break;
		case CalibrateLED:
			//            dbgSerial.print("CalibrateLED ");
			break;
		case Stop:
			//            dbgSerial.print("Stop ");
			break;
		case SlewReverse:
			//            dbgSerial.print("SlewReverse ");
			break;
		case SlewForward:
			//            dbgSerial.print("SlewForward ");
			break;
		case GetStatus:
			//            dbgSerial.print("GetStatus ");
			axis[Motor].HBXCount = 4;
			break;
		case GetLEDI:
			//            dbgSerial.print("GetLEDI ");
			axis[Motor].HBXCount = 1;
			break;
		case GetMotorType:
			//            dbgSerial.print("GetMotorType ");
			axis[Motor].HBXP1 = axis[Motor].MotorType;
			axis[Motor].HBXCount = 1;
			break;
		case SleepHBX:
			//            dbgSerial.print("SleepHBX ");
			break;
		default:
			dbgSerial.print("UNKNOWN ");
			break;
		}

		/*
				if (axis[Motor].HBXCount != 0) {
					dbgSerial.print(", Data: ");
					dbgSerial.print(axis[Motor].HBXP1, HEX);
					if (axis[Motor].HBXCount >= 2)  dbgSerial.print(", ");
					axis[Motor].HBXCount -= 1;
				}
				if (axis[Motor].HBXCount != 0) {
					dbgSerial.print(axis[Motor].HBXP2, HEX);
					if (axis[Motor].HBXCount >= 2)  dbgSerial.print(", ");
					axis[Motor].HBXCount -= 1;
				}
				if (axis[Motor].HBXCount != 0) {
					dbgSerial.print(axis[Motor].HBXP3, HEX);
					if (axis[Motor].HBXCount >= 2)  dbgSerial.print(", ");
					axis[Motor].HBXCount -= 1;
				}
				if (axis[Motor].HBXCount > 0) {
					dbgSerial.print(axis[Motor].HBXP4, HEX);
					axis[Motor].HBXCount -= 1;
				}
				dbgSerial.println("");
		*/
	}
}
