/*
 * Copyright 2017, 2018 John Archbold
*/


#include <Arduino.h>

/********************************************************
  EQG Protocol related functions
  ==============================
 *********************************************************/

// Process received EQG characters
// ===============================
void EQGState(void) {
  while ((EQGRxiPtr != EQGRxoPtr) && (EQGDone == 0)) {
      digitalWrite(FROMEQG, HIGH);             // Set Indicator LED
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
            switch (EQGCmnd) {    // Need data?
              case 'A':           // Unknown
              case 'B':           // Unknown
              case 'E':           // Set Current Position
              case 'G':           // Set Move direction and speed multiplier
              case 'H':           // Set Target Position
              case 'I':           // Set GOTO Speed
              case 'M':           // Set Mount Motor Change Speed
              case 'P':           // Set the AutoGuide Speed
              case 'R':           // Unknown
              case 'S':           // Unknown
              case 'T':           // Unknown
              case 'U':           // Unknown
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

				case 0x04:  // First nibble				
						EQGP1 = EQGRxChar; // EQGRxChar already converted to binary nibble
						EQGRxCount++;           // Increase character count
						switch (EQGCmnd) {		  // Need to get more data?
              case 'B':             // B only takes one nibble
              case 'P':             // P only takes one nibble
								EQGRxState = EQG_INTERPRET;
							break;
							default:
								EQGRxState++;
							break;
						}
				  break;

 				case 0x05:  // Second nibble - first byte (B1 = N1N2)
						EQGP2 = EQGRxChar;   // EQGRxChar already converted to binary nibble
						EQGP1 = ((EQGP1 << 4) | EQGP2); // First byte
            EQGRxCount++;
						switch (EQGCmnd) {		    // Need to get more data?
              case 'A':               // A unknown
              case 'G':               // G Set direction, range
              case 'R':               // R unknown
								EQGRxState = EQG_INTERPRET;
							break;
							default:
								EQGRxState++;
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
						EQGRxState++;			        // Get next data
				  break;

 				case 0x08:                    // Fifth nibble - N5xxN3N4N1N2
						EQGP2 = EQGRxChar;
						EQGRxCount++;
						EQGP1 |= (EQGP2 << 20);
						EQGRxState++;			        // Get next data
				  break;

 				case 0x09:                    // Sixth nibble - N5N6N3N4N1N2
						EQGP2 = EQGRxChar;
						EQGRxCount++;
						EQGP1 |= (EQGP2 << 16);
						EQGRxState = EQG_INTERPRET;		// All done
				  break;

 				case EQG_INTERPRET:					
					if ((EQGRxChar == 0x0d) && (EQGRxCount >= 3)) {
						EQGRxState = EQG_CMNDSTART;    // Reset state machine
						switch (EQGCmnd) {		
                                  // Commands that need no data
							case 'a':           // Read steps per rotation
							case 'b':           // Read tracking scale
							case 'c':           // Read Motor Speed Change
							case 'd':           // 
							case 'e':           // Read firmware version
							case 'f':           // Read motor status
							case 'g':           // Read speed divisor
							case 'j':           // Read position
							case 'm':           // Read motor slowdown point
							case 's':           // Read steps per arcsec
							case 'D':           // Read track rate
							case 'F':           // Initialize motors
							case 'J':           // GoTo position, track
							case 'K':           // Stop movement
								EQGDone++;
							  break;
                                  // Commands that need data
							case 'E':           // Set current position
							case 'H':           // Set target position
							case 'I':           // Set GoTo speed
							case 'M':           // Set motor slowdown position
								if (EQGRxCount == (3 + 6)) {
									EQGDone++;
								}
								else {
                  if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                  else EQGDone++;
                  EQGErrorValue = '3'; // Failure - Bad Parameters
								}
							  break;

							case 'G':           // Set direction and range
								if (EQGRxCount == (3 + 2)) {
									EQGDone++;
								}
								else {
                  if (EQGRxChar != 0x0d) EQGRxState = EQG_WAITFORCR;
                  else EQGDone++;
									EQGErrorValue = '3'; // Failure - Bad Parameters
							  }
							  break;

							case 'P':           // Set autoguide speed
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
  digitalWrite(FROMEQG, LOW);             // Clear Indicator LED
}

void EQGError(unsigned char errorbyte) {
  EQGTx('!') ;        // Failure - Bad Parameters
  EQGTx(errorbyte);
  EQGTx(CR);
  EQGDone = 0;        // Process errors
  EQGRxState = EQG_CMNDSTART;
  EQGRxCount = 0;     // Count for # parameters
  EQGErrorValue = 0;
}

// Received a valid command so execute it, if supported
// ====================================================
void EQGAction(void) {

		EQGDone = 0;						            // Reset flag
    EQGRxState = EQG_CMNDSTART;
		digitalWrite(FROMEQG, HIGH);        //  and status LED
		EQGTx('=');                         // Answer (with parameters)

		switch (EQGCmnd) {

				case 'a':						          // Request total number of steps per revolution
          EQGTxHex6(axis[EQGMOTOR].STEPSPER360);
				  break;

				case 'b':                     // Request step interrupt frequency	
          EQGTxHex6(axis[EQGMOTOR].HBX_bVALUE);  // Let EQMOD calculate SIDEREAL, etc
				  break;

				case 'c':
				  EQGTxHex6(0x00D800);	      // Return same as EQ6 to keep the speed commands consistent
				  break;

				case 'd':						          // Request Mount center point
					EQGTxHex6(EQG_CENTRE);      // Return same as EQ6 to keep the speed commands consistent
				  break;

				case 'e':                     // Request version
					EQGTxHex6(EQGVERSION);	    // V5.01
				  break;

				case 'f':                     // Request motor status  

          // MotorState bit definitions
// nibble 1
// MOVESTEP        0x0001      // Slew(0)        Step(1)
// MOVEDIRN        0x0002      // Decreasing(0)  Increasing(1)
// MOVELOW         0x0004      // Low(0)         High(1)
// nibble2
// MOVEAXIS        0x0010      // Stopped(0)     Moving(1)
// MOVEFACE        0x0020      // Front(0)       Rear(1)
// nibble3
// COILACTIVE      0x0100      // Inactive(0)    Active(1)
            EQGTxHex3(axis[EQGMOTOR].HBXMotorStatus);

        break;

				case 'g':                     // Request speed multiplier
					EQGTxHex2(EQG_gVALUE);      // Return same as EQ6 to keep the speed commands consistent
				  break;

				case 'j':                     // Request axis position    
          EQGTxHex6(axis[EQGMOTOR].HBXPosn);
				  break;

				case 'm':                       // GET Point at which to change from fast to slow
					EQGTxHex6(axis[EQGMOTOR].HBXSlowDown);
				  break;

				case 's':                                 // PEC period
					EQGTxHex6(axis[EQGMOTOR].PEC);
				  break;

				case 'E':							            // Set current motor position
          axis[EQGMOTOR].HBXPosn = EQGP1;
          break;

				case 'F':                           // Initialize and activate motors
						axis[EQGMOTOR].HBXMotorStatus |= COILACTIVE;		  
				  break;

				case 'G':                           // EQG 'G' Command      - SET move parameters
                                             // DIRECTION       0x0001      // N/E(0)       S/W(1)
                                             // HEMISPHERE??    0x0002      // North(0)     South(1)

				axis[EQGMOTOR].HBXDirSpeed = (int)EQGP1;      // Save the actual value
        if (EQGP1 & DIRECTION)                        // Set direction bit
          axis[EQGMOTOR].HBXMotorStatus &= ~MOVEDIRN;
        else 
          axis[EQGMOTOR].HBXMotorStatus |= MOVEDIRN;
       
                                             // SLEWSTEP        0x0010      // Slew(0)      Step(1)
                                             // HIGHLOW         0x0020      // High(0)      Low(1)  
        switch ((EQGP1 >> 4) & 0x03) {
          case 00:    // 0 HIGH SPEED SLEW
            axis[EQGMOTOR].HBXMotorStatus &= ~MOVESTEP;
            axis[EQGMOTOR].HBXMotorStatus &= ~MOVELOW;
            break;
          case 01:    // 1 HIGH  SPEED STEP
            axis[EQGMOTOR].HBXMotorStatus |= MOVESTEP;
            axis[EQGMOTOR].HBXMotorStatus &= ~MOVELOW;
            break;
          case 02:    // 2 LOW  SPEED GOTO
            axis[EQGMOTOR].HBXMotorStatus &= ~MOVESTEP;
            axis[EQGMOTOR].HBXMotorStatus |= MOVELOW;
            break;
          case 03:    // 3 LOW SPEED STEP
            axis[EQGMOTOR].HBXMotorStatus |= MOVESTEP;
            axis[EQGMOTOR].HBXMotorStatus |= MOVELOW;
            break;
        }
				break;

				case 'H':                           // SetGotoTargetIncrement(AXISID Axis, long StepsCount)
          axis[EQGMOTOR].HBXDelta = EQGP1;
          if ((axis[EQGMOTOR].HBXMotorStatus & MOVEDIRN) == FORWARD)
            axis[EQGMOTOR].HBXTarget = axis[EQGMOTOR].HBXPosn + axis[EQGMOTOR].HBXDelta;  // add the relative target
          else
            axis[EQGMOTOR].HBXTarget = axis[EQGMOTOR].HBXPosn - axis[EQGMOTOR].HBXDelta;  // subtract the relative target

dbgSerial.println("");
dbgSerial.print("HBX:<");
dbgSerial.print(EQGMOTOR);
dbgSerial.print("-");
dbgSerial.print(axis[EQGMOTOR].HBXMotorStatus, HEX);
dbgSerial.print("> ");
dbgSerial.print(", Pos: ");
dbgSerial.print(axis[EQGMOTOR].HBXPosn, HEX);
dbgSerial.print(", Del: ");
dbgSerial.print(axis[EQGMOTOR].HBXDelta, HEX);
dbgSerial.print("->Tgt: ");
dbgSerial.print(axis[EQGMOTOR].HBXTarget, HEX);

          axis[EQGMOTOR].HBXMotorControl |= MoveHBX;            // Signal pending move
				  break;

				case 'I':   // Set motor speed

          if ( EQGP1 == 0x177C ) {    // axis[EQGMOTOR].SIDEREALRATE) {
            EQGP1 = axis[EQGMOTOR].SIDEREALRATE;
          }
          else if ( EQGP1 == 0x1785 ) {     // axis[EQGMOTOR].SOLARRATE) {
            EQGP1 = axis[EQGMOTOR].SOLARRATE;
          }
          else if ( EQGP1 == 0x185B ) {     // axis[EQGMOTOR].LUNARRATE) {
            EQGP1 = axis[EQGMOTOR].LUNARRATE;
          }
          else { 
            if (axis[EQGMOTOR].HBXMotorStatus & MOVELOW) {    // 1 -> HIGH
                                                              // EQG is (400, 480, 600, 800)
                                                              //  value:  18,  14,  10,  0c
                                                              // Map 800 to 2 degrees / sec
              
              EQGF1 = (float)EQGMAXIMUMSPEED / (float)EQGP1;        // Scale to maximum EQG speed
              EQGF1 = EQGF1 * (float)axis[EQGMOTOR].DEGREESPEED2;   // Multiply by maximum ETX speed
            }
            else {                                            // EQG is (.5, 1, 2, 8, .. 64) * Sidereal
              EQGF1 = EQGF1 * (float)axis[EQGMOTOR].SIDEREALSPEED;        // Multiply by Sidereal ETX speed
            }
            EQGP1 = (unsigned long) EQGF1;                    // Convert to ETX format
            EQGP2 = (unsigned long) ((EQGF1 - float(EQGP1)) * 65536L);
            EQGP1 = ((EQGP1 << 16) & 0xFF0000);               // Format to 8.16 rate
            EQGP1 |= (EQGP2 & 0xFFFF);
		      }
					axis[EQGMOTOR].HBXSpeed = EQGP1;                // Set the speed
				  break;

				case 'J':             // Tell motor to Go
          axis[EQGMOTOR].HBXMotorStatus |= MOVEAXIS;       // Signal moving
				  break;

				case 'K':							// Tell motor to stop
          axis[EQGMOTOR].ETXMotorState = ETXStopMotor;
				  break;

				case 'M':             // SetBreakPointIncrement(AXISID Axis, long StepsCount)

          axis[EQGMOTOR].HBXSlowDown = EQGP1;
          if ((axis[EQGMOTOR].HBXMotorStatus & MOVEDIRN) == FORWARD)
            axis[EQGMOTOR].HBXSlowDown = axis[EQGMOTOR].HBXTarget + axis[EQGMOTOR].HBXSlowDown;  // add the relative target
          else
            axis[EQGMOTOR].HBXSlowDown = axis[EQGMOTOR].HBXTarget - axis[EQGMOTOR].HBXSlowDown;  // subtract the relative target

dbgSerial.println("");
dbgSerial.print("HBX:<");
dbgSerial.print(EQGMOTOR);
dbgSerial.print("-");
dbgSerial.print(axis[EQGMOTOR].HBXMotorStatus, HEX);
dbgSerial.print("> ");
dbgSerial.print(", Pos: ");
dbgSerial.print(axis[EQGMOTOR].HBXPosn, HEX);
dbgSerial.print(", Del: ");
dbgSerial.print(EQGP1, HEX);
dbgSerial.print("-> SD: ");
dbgSerial.println(axis[EQGMOTOR].HBXSlowDown, HEX);

				  break;

				case 'P':
					axis[EQGMOTOR].HBXGuide = (char)EQGP1;
				  break;

				default:
          EQGErrorValue = '0';     // Failure - Bad Command
				  break;
			}
      EQGTx(CR);
      digitalWrite(FROMEQG, LOW);             // Clear Indicator LED
}

/**********************************************
  Handle EQG communications
***********************************************/
void EQGRx(void) {
  while (EQGSerial.available() > 0) {
    EQGRxBuffer[EQGRxiPtr++] = EQGSerial.read();
    EQGRxiPtr &= EQGMASK;
  }
}

void EQGTx(unsigned char data) {
  EQGTxBuffer[EQGTxiPtr++] = data;
  EQGTxiPtr &= EQGMASK;
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
  EQGTxHex( (unsigned char) data);
  EQGTxHex( (unsigned char) (data >> 4));
  EQGTxHex( (unsigned char) (data >> 8));
}

void EQGTxHex6(unsigned long data) {
  EQGTxHex2( (unsigned char) data);
  EQGTxHex2( (unsigned char) (data >> 8));
  EQGTxHex2( (unsigned char) (data >> 16));
}

/**********************************************
  Debug routines
***********************************************/
void putbyte(unsigned char data) {
  dbgSerial.write(data);
}

void puthexb(unsigned char data) {
  if (((data>>4) & 0x0f) < 0x0a) putbyte(((data>>4) & 0x0f) + 0x30);
  else putbyte(((data>>4) & 0x0f) + 0x37);
  if ((data & 0x0f) < 0x0a) putbyte((data & 0x0f) + 0x30);
  else putbyte((data & 0x0f) + 0x37);
}

void putdecb(unsigned char data) {
  dbgSerial.print(data);
}

void puthexw(unsigned int data) {
  puthexb((data>>8) & 0xFF);
  puthexb(data & 0xFF);
}

void putdecw(unsigned int data) {
  dbgSerial.print(data);
}

void puthex6(unsigned long data) {
  puthexb((data>>16) & 0xFF);
  puthexw(data & 0xFFFF);
}

void puthexl(unsigned long data) {
  puthexw((data>>16) & 0xFFFF);
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
  EQGSendHex2( (unsigned char) data);
  EQGSendHex2( (unsigned char) (data >> 8));
  EQGSendHex2( (unsigned char) (data >> 16));
}

void debugEQG() {
  dbgSerial.println("");
  dbgSerial.print("Az:<");
  dbgSerial.print(axis[AzMotor].HBXMotorStatus, HEX);
  dbgSerial.print(">Pos: ");
  dbgSerial.print(axis[AzMotor].HBXPosn, HEX);
  dbgSerial.print(" SD: ");
  dbgSerial.print(axis[AzMotor].HBXSlowDown, HEX);
  dbgSerial.print(" Tgt: ");
  dbgSerial.print(axis[AzMotor].HBXTarget, HEX);
  
  dbgSerial.print(", Alt:<");
  dbgSerial.print(axis[AzMotor].HBXMotorStatus, HEX);
  dbgSerial.print(">Pos: ");
  dbgSerial.print(axis[AltMotor].HBXPosn, HEX);
  dbgSerial.print(" SD: ");
  dbgSerial.print(axis[AltMotor].HBXSlowDown, HEX);
  dbgSerial.print(" Tgt: ");
  dbgSerial.print(axis[AltMotor].HBXTarget, HEX);
}
