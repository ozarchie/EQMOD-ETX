/*
 * Copyright 2017, 2018 John Archbold
*/


/********************************************************
  ETX Protocol related functions
  ==============================
 *********************************************************/

bool ETXState(unsigned char Motor) {
  
  long  c, d;
  long  l1;
  int   s1;
  
  switch(axis[Motor].ETXMotorState) {

    case ETXCheckStartup:
      if (axis[Motor].HBXMotorStatus & MOVEAXIS) {         // Are we moving?
        if (axis[EQGMOTOR].HBXMotorStatus & MOVESTEP) {    // Step or Slew
          axis[Motor].ETXMotorState = ETXStepMotor;
        }
        else {
        if (axis[Motor].HBXDelta > 0x800)                 // Slew large enough for ETX slew?
          axis[Motor].ETXMotorState = ETXSlewMotor;       // Yes, go
          else {
            axis[EQGMOTOR].HBXMotorStatus |= MOVESTEP;    // No, change to step
            if (axis[EQGMOTOR].HBXDelta < 0x100) {        // Scale speed down for really small slews
              axis[Motor].HBXTargetSpeed = axis[Motor].HBXTargetSpeed >> 2;
            }
            axis[Motor].HBXSpeed = 0;          
            axis[Motor].ETXMotorState = ETXStepMotor;
          }
        }
      }
      break;
  
    case ETXSlewMotor:
      if (Motor == MotorAz) digitalWrite(AzLED, HIGH);    // Turn on the LED
      else digitalWrite(AltLED, HIGH);
      if ((axis[Motor].HBXMotorStatus & MOVEDIRN) == FORWARD)
        axis[Motor].HBXCmnd = SlewForward;
      else
        axis[Motor].HBXCmnd = SlewReverse;
      if (!(HBXSendCommand(axis[Motor].HBXCmnd, Motor))) {
        axis[Motor].ETXMotorState = ETXCheckStartup;      // Motor failed to start
      }
      else
        axis[Motor].ETXMotorState = ETXCheckSlowDown;
      break;
  
    case ETXStepMotor:
      if (Motor == MotorAz) digitalWrite(AzLED, HIGH);    // Turn on the LED
      else digitalWrite(AltLED, HIGH);
      if (axis[Motor].HBXMotorStatus & MOVELOW)            // Stepping, High or Low speed
        axis[Motor].HBXCmnd = RotateSlow;
      else axis[Motor].HBXCmnd = RotateFast;
      if (axis[Motor].HBXSpeed == 0) {                    // Starting up
        axis[Motor].HBXSpeed = axis[Motor].HBXTargetSpeed >> 2;
      }

      // Set the speed, and direction
      // ----------------------------
      P1 = axis[Motor].HBXSpeed;
      if ((axis[Motor].HBXMotorStatus & MOVEDIRN) == REVERSE)  // If negative, change P
        P1 = TwosComplement(P1);                          //  to 2's complement
      axis[Motor].HBXP1 = (P1 >> 16) & 0xFF;              // Initialize command bytes
      axis[Motor].HBXP2 = (P1 >>  8) & 0xFF;
      axis[Motor].HBXP3 = P1 & 0xFF;
    
      // Send the command
      // ----------------
      if (HBXSendCommand(axis[Motor].HBXCmnd, Motor))   // Command OK?
        HBXSend3Bytes(Motor);                           // Send the speed
      if (axis[Motor].HBXMotorControl & MoveHBX) {
        axis[Motor].ETXMotorState = ETXCheckSpeed;      // For a position move      
      }
      else { 
        axis[Motor].ETXMotorState = ETXCheckStartup;    // Just a speed command
        axis[Motor].HBXMotorStatus &= ~MOVEAXIS;         // Speed set, all done
      }
      break;
  
    case ETXCheckSlowDown:
      // Check if Slowdown reached
      // Ramp-up speed til we get to slowdown point
      if ((axis[Motor].HBXMotorStatus & MOVEDIRN) == FORWARD) { // Moving forwards
        d = axis[Motor].HBXSlowDown;
        c = axis[Motor].HBXPosn;
      } 
      else {                                              // Moving backwards
        d = axis[Motor].HBXPosn;
        c = axis[Motor].HBXSlowDown;   
      } 
      if ((c >= d) || ((d - c) < ETXSLOWPOSN)) {          // Check distance to slowdown              
        axis[Motor].HBXSpeed = axis[Motor].DEGREERATE2;   // Default slowdown 1st speed
        axis[Motor].HBXTargetSpeed = 0;                   // Default slowdown target i.e. stopped
        axis[Motor].HBXSpeedState = 0;                    // Speed ramp state
        axis[Motor].HBXMotorStatus |= MOVESTEP;           // Set the step flag
        axis[Motor].ETXMotorState = ETXStepMotor;         // GoTo speed ramp
      }
      break;
      
    case ETXCheckSpeed:
      // Speeding Up
      // ===========
      if ((axis[Motor].HBXTargetSpeed != 0) && (axis[Motor].HBXTargetSpeed > axis[Motor].HBXSpeed)) {
        if ((axis[Motor].HBXTargetSpeed - axis[Motor].HBXSpeed) >= axis[Motor].DEGREERATE1) {
          axis[Motor].HBXSpeed += ((axis[Motor].HBXTargetSpeed - axis[Motor].HBXSpeed) >> 2);
          axis[Motor].ETXMotorState = ETXStepMotor;
        }
        else {
          axis[Motor].HBXSpeed = axis[Motor].HBXTargetSpeed;
          axis[Motor].ETXMotorState = ETXCheckPosition;
        }
      }
      // Slowing Down
      // ============
      else {
        if ((axis[Motor].HBXSpeed - axis[Motor].HBXTargetSpeed) >= axis[Motor].DEGREERATE1) {
          axis[Motor].HBXSpeed -= ((axis[Motor].HBXSpeed - axis[Motor].HBXTargetSpeed) >> 2);
          axis[Motor].ETXMotorState = ETXStepMotor;
        }
        else 
          axis[Motor].ETXMotorState = ETXCheckPosition;
      }
      break;
  
    case ETXCheckPosition:
      // Check if Target acquired
      // ------------------------

      // Calculate absolute distance to target
      // -------------------------------------
      if (axis[Motor].HBXMotorControl & MoveHBX) {
        if ((axis[Motor].HBXMotorStatus & MOVEDIRN) == REVERSE) {
          d = axis[Motor].HBXTarget;
          c = axis[Motor].HBXPosn;
        }
        else {
          c = axis[Motor].HBXTarget;
          d = axis[Motor].HBXPosn;          
        }
        l1 = c - d;

        if (l1 >= 0) {
          // Start to slow motor if getting near target
          // ------------------------------------------
          if ((l1 <= 0x400) && (axis[Motor].HBXSpeedState == 0)) {
            axis[Motor].HBXSpeed = axis[Motor].HBXSpeed >> 1;   // 1/2
            axis[Motor].ETXMotorState = ETXStepMotor;           // Change speed
            axis[Motor].HBXSpeedState += 1;
          }
          else if ((l1 <= 0x100) && (axis[Motor].HBXSpeedState == 1)) {
            axis[Motor].HBXSpeed = axis[Motor].HBXSpeed >> 1;   // 1/4
            axis[Motor].ETXMotorState = ETXStepMotor;           // Change speed
            axis[Motor].HBXSpeedState += 1;
          }
          else if ((l1 <= 0x40) && (axis[Motor].HBXSpeedState == 2)) {
            axis[Motor].HBXSpeed = axis[Motor].HBXSpeed >> 2;   // 1/16
            axis[Motor].ETXMotorState = ETXStepMotor;           // Change speed;
            axis[Motor].HBXSpeedState += 1;
          }
          else if ((l1 <= 0x20) && (axis[Motor].HBXSpeedState == 3)) {
            axis[Motor].HBXSpeed = axis[Motor].HBXSpeed >> 2;   // 1/64
            axis[Motor].ETXMotorState = ETXStepMotor;           // Change speed;
            axis[Motor].HBXSpeedState += 1;
          }
          else if (l1 <= 0x08) {
            axis[Motor].ETXMotorState = ETXStopMotor;           // Stop motor, next state
          }
        }
        else {
          // Motor has over-shot the target
          // ------------------------------
          if ((axis[Motor].HBXMotorStatus & MOVEDIRN) == FORWARD) // EQG -> change direction
            axis[Motor].HBXMotorStatus &= ~FORWARD;
          else
            axis[Motor].HBXMotorStatus |= FORWARD;                        
//          axis[Motor].HBXSpeed = TwosComplement(axis[Motor].HBXSpeed);  // ETX invert speed
          axis[Motor].ETXMotorState = ETXStepMotor;                     // Change ETX speed
        }
      }
      else {
        axis[Motor].ETXMotorState = ETXStopMotor;
      }
      break;
  
    case ETXStopMotor:
      while(!(HBXSendCommand(Stop, Motor)));                    // Stop the motor
      axis[Motor].ETXMotorState = ETXMotorEnd;                  // Final state
      break;
      
    case ETXMotorEnd:
      if (Motor == MotorAz) digitalWrite(AzLED, LOW);           // Turn off the LED
      else digitalWrite(AltLED, LOW);
      if (axis[Motor].HBXMotorControl & MoveHBX) {              // if we were moving
        // Check we got there exactly
        // --------------------------
        if ((axis[Motor].HBXMotorStatus & MOVEDIRN) == REVERSE) {
          d = axis[Motor].HBXTarget;
          c = axis[Motor].HBXPosn;
        }
        else {
          c = axis[Motor].HBXTarget;
          d = axis[Motor].HBXPosn;          
        }
        l1 = c - d;

          // Set the offset
          // ----------------------------
        if (l1 != 0) {
          axis[Motor].HBXP1 = (l1 >> 8) & 0xFF;              // Initialize offset bytes
          axis[Motor].HBXP2 = l1 & 0xFF;
          axis[Motor].HBXCmnd = SetOffset;
          if (HBXSendCommand(axis[Motor].HBXCmnd, Motor))   // Command OK?
            HBXSend2Bytes(Motor);                           // Send the offset
        }  
        axis[Motor].HBXPosn = axis[Motor].HBXTarget;
        axis[Motor].HBXMotorControl &= ~MoveHBX;                // Clear the flag       
      }
      axis[Motor].HBXMotorStatus |= MOVESTEP;                    // Set stepping mode
      axis[Motor].HBXMotorStatus &= ~MOVELOW;                    //  and speed
      axis[Motor].HBXMotorStatus &= ~MOVEAXIS;                   // Clear the motor moving flag 
      axis[Motor].ETXMotorState = ETXCheckStartup;
      axis[Motor].HBXTargetSpeed = axis[Motor].DEGREERATE1 >> 3;  // For any subsequent move
      axis[Motor].HBXSpeed = 0;
      break;
      
    default:  
      break;
  }
}

// Motor functions

bool HBXGetStatus(unsigned char Motor) {
  axis[Motor].HBXP4 = 0xFF;     // Preset error     
  if (!HBXSendCommand(GetStatus, Motor)) {
    return(false);
  }
  HBXGet3Bytes(Motor);     
  if (axis[Motor].HBXP4 == 0) {     // If no error, update position
    P1 = (axis[Motor].HBXP1 << 8);
    P1 |= axis[Motor].HBXP2;        // Convert to 16bits
    if (axis[Motor].HBXP1 & 0x80)
      P1 |= 0xffff0000;             // Sign extend HBXP1 for 2s complement
    axis[Motor].HBXPosn += P1;
    axis[Motor].HBXPosn &= 0x00FFFFFF;
    return(true);
  }
  else return(false);
}

bool HBXGet2Status(void) {
  int i;
  do {
    i = 0;       
    if (HBXGetStatus(AzMotor))
      i += 1;
    if (HBXGetStatus(AltMotor))
      i += 1;
  } while (i < 2);
  return(true);  
}

void WaitForMotors(void) {
// GetLED commands always return a vaild value - motors not online until this happens
// Valid values are not 0 and not 0xFF for Az, Alt. (exception here is if LEDRA || LEDAlt == 0xff)
  do {
    P1 = 0;
    if (HBXSendCommand(GetLEDI, AzMotor))
      axis[AzMotor].HBXLEDI = HBXGetByte(AzMotor);
    if ((axis[AzMotor].HBXLEDI != 0) && (axis[AzMotor].HBXLEDI != 0xFF))
      P1 += 1;
    if (HBXSendCommand(GetLEDI, AltMotor))
      axis[AltMotor].HBXLEDI = HBXGetByte(AltMotor);
    if ((axis[AltMotor].HBXLEDI != 0) && (axis[AltMotor].HBXLEDI != 0xFF))
      P1 += 1;
    TimerDelaymS(MOTORDETECT);      // Wait .5s between loops
  } while (P1 < 2);
}

void AzInitialise(void) {
  // ETX
  axis[AzMotor].HBXP1 = 0x00;             
  axis[AzMotor].HBXP2 = 0x00;
  axis[AzMotor].HBXP3 = 0x00;
  axis[AzMotor].HBXP4 = 0x00;
  
  axis[AzMotor].HBXPosn = ETX_CENTRE;
  axis[AzMotor].HBXTarget = axis[AzMotor].HBXPosn;
  axis[AzMotor].HBXDirSpeed = 0x000;
  axis[AzMotor].HBXMotorStatus = MOVESTEP;
  
  axis[AzMotor].HBX_bVALUE = EQG_RAbVALUE;
  axis[AzMotor].SIDEREALRATE = AzSIDEREALRATE;
  axis[AzMotor].SOLARRATE = AzSOLARRATE;
  axis[AzMotor].LUNARRATE = AzLUNARRATE;
  axis[AzMotor].DEGREERATE1 = AzDEGREERATE1;
  axis[AzMotor].DEGREERATE2 = AzDEGREERATE2;
  axis[AzMotor].SIDEREALSPEED = AzSIDEREALSPEED;
  axis[AzMotor].SOLARSPEED = AzSOLARSPEED;
  axis[AzMotor].LUNARSPEED = AzLUNARSPEED;
  axis[AzMotor].DEGREESPEED1 = AzDEGREESPEED1;
  axis[AzMotor].DEGREESPEED2 = AzDEGREESPEED2;
  axis[AzMotor].STEPSPER360 = AzSTEPSPER360; 
  axis[AzMotor].WORM = AzWORM; 
  axis[AzMotor].PEC = AzPEC;

  axis[AzMotor].ETXMotorState = ETXCheckStartup;
}

void AltInitialise(void) {
  // ETX
  axis[AltMotor].HBXP1 = 0x00;             
  axis[AltMotor].HBXP2 = 0x00;
  axis[AltMotor].HBXP3 = 0x00;
  axis[AltMotor].HBXP4 = 0x00;
  
  axis[AltMotor].HBXPosn = ETX_CENTRE;
  axis[AltMotor].HBXTarget = axis[AltMotor].HBXPosn;
  axis[AltMotor].HBXDirSpeed = 0x000;
  axis[AltMotor].HBXMotorStatus = MOVESTEP;
  
  axis[AltMotor].HBX_bVALUE = EQG_DECbVALUE;
  axis[AltMotor].SIDEREALRATE = AltSIDEREALRATE;
  axis[AltMotor].SOLARRATE = AltSOLARRATE;
  axis[AltMotor].LUNARRATE = AltLUNARRATE;
  axis[AltMotor].DEGREERATE1 = AltDEGREERATE1;
  axis[AltMotor].DEGREERATE2 = AltDEGREERATE2;
  axis[AltMotor].SIDEREALSPEED = AltSIDEREALSPEED;
  axis[AltMotor].SOLARSPEED = AltSOLARSPEED;
  axis[AltMotor].LUNARSPEED = AltLUNARSPEED;
  axis[AltMotor].DEGREESPEED1 = AltDEGREESPEED1;
  axis[AltMotor].DEGREESPEED2 = AltDEGREESPEED2;
  axis[AltMotor].STEPSPER360 = AltSTEPSPER360;
  axis[AltMotor].WORM = AltWORM; 
  axis[AltMotor].PEC = AltPEC;

  axis[AltMotor].ETXMotorState = ETXCheckStartup;  
}  

