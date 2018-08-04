/*
 * Copyright 2017, 2018 John Archbold
*/


#include <Arduino.h>

/********************************************************
  Utility functions to monitor HBX comms
  ======================================
  The monitor is enabled by reading the state of the MONITORHBX pin.
  The pin definition is set in EQG2HBX and changes depending on the interface board:
  #ifdef m2560
  #define MONITORHBX      11          // Mega2560 D3
  #define TESTHBX         9           // Mega2560 D2
  #endif
  #ifdef  ESP32
  #define MONITORHBX      35
  #define TESTHBX         32
  #endif

  This mode overrides the EQG2HBX protocol conversion.
  It uses the board inetrface to monitor the HBX 'serial' commands and prints the details:
    Start
    Message
    End
  ToDo:
  Change to H2XISR for interrupt driven receive
 *********************************************************/
void HBXMonitorLoop(void){
    int Messages;
    PreviousTime = millis();
    
    dbgSerial.println("ETX-Monitor");
//  H2XReset();
    axis[AzMotor].PrintStatus0 = 1;           // Enable printing "status polls" with no change
    axis[AltMotor].PrintStatus0 = 1;          // Enable printing "status polls" with no change
    HBXMonitorMode();

  do {
    Messages = 0;
    if (HBXMonitorHCL(CLOCKTIMEOUT)) {
      if (DetectedClock) { 
        HBXMonitorMessage(DetectedClock);               // Wait for low clock and print data
        axis[DetectedClock].TimeDelta = millis();       // - PreviousTime;
        Messages += 1;
      }
    }
    if (HBXMonitorHCL(CLOCKTIMEOUT)) {                  // Better read Alt
      if (DetectedClock) {                              // as 
        HBXMonitorMessage(DetectedClock);               //  Alt msg may follow immediately
        axis[DetectedClock].TimeDelta = millis();       // - PreviousTime;
        Messages += 1;
      }
    }

    if (Messages) {
      switch(Messages) {
        case 1:
        HBXPrintState(DetectedClock);
        break;
        case 2:
          if (DetectedClock == MotorAz) {
            HBXPrintState(MotorAlt);
            HBXPrintState(MotorAz);
          }
          else {
            HBXPrintState(MotorAz);
            HBXPrintState(MotorAlt);          
          }
          break;
        default:
          break;
      }
    }
    
  } while (digitalRead(MONITORHBX) == 0);                 // Check if jumper installed
}    

void HBXMonitorMode(void) {

  HDAListen();                    // HDA as input
  HCL1Listen();                   // HCL as input
  HCL2Listen();
  TimerDelaymS(STARTTIME);
}

bool HBXMonitorHCL(unsigned char Timeout) {
  int j;
  DetectedClock = 0;
  j = Timeout;                        
  while (!digitalRead(HDA1));           // Wait for data high
  while (DetectedClock == 0) {   // Wait for a low on either clock 
    if (digitalRead(HCL1) == 0) {
      TimerDelayuS(3);                  // Re-read in case of glitches
      if (digitalRead(HCL1) == 0) DetectedClock = MotorAz;
      if (digitalRead(HCL2) == 0) {     // Fail if both low
        DetectedClock = 0;
        return (false);
      }
    }
    else if (digitalRead(HCL2) == 0) {
      TimerDelayuS(3);                  // Re-read in case of glitches
      if (digitalRead(HCL2) == 0) DetectedClock = MotorAlt;
      if (digitalRead(HCL1) == 0) {     // Fail if both low
        DetectedClock = 0;
        return (false);
      }
    }
//    j -= 1;
  }

//  if (j) {
    while (digitalRead(HDA1));            // Wait for data low
    while (digitalRead(HDA1));            // Wait for data low again
    if (DetectedClock == MotorAz) {       // Check clock is still low
      if (digitalRead(HCL1)) return (false);
    }
    else if (digitalRead(HCL2)) return (false);
    
    if (DetectedClock == MotorAz) {
      while (!digitalRead(HCL1));         // Wait for clock high
    }
    else while (!digitalRead(HCL2));
    return (true);
//  }
//  return (false);                       // Timeout
}

void HBXMonitorEnd(unsigned char Motor) {

  if (Motor == MotorAz) while (!digitalRead(HCL1));   // Wait for clock high
  else while (!digitalRead(HCL2));
}

bool HBXMonitorBit(unsigned char Motor) {
//  dbgSerial.write('!');
  axis[Motor].HBXData = 0;
  if (Motor == MotorAz) while (!digitalRead(HCL1));   // Wait for clock high
  else while (!digitalRead(HCL2));
  if (Motor == MotorAz) while (digitalRead(HCL1));    // Wait for clock low
  else while (digitalRead(HCL2));
  if (digitalRead(HDA1)) axis[Motor].HBXData |= 0x01; // Read the bit
  return(true);
}

bool HBXMonitorByte(unsigned char Motor) {
//  dbgSerial.write('>');
  axis[Motor].HBXBitCount = 8;  
  axis[Motor].HBXData = 0;
  while (axis[Motor].HBXBitCount) {
    axis[Motor].HBXData = axis[Motor].HBXData << 1;       // Shift previous bit
    axis[Motor].HBXBitCount--;                            // Need eight bits
    if (Motor == MotorAz) while (!digitalRead(HCL1));     // Wait for clock high
    else while (!digitalRead(HCL2));
    if (Motor == MotorAz) while (digitalRead(HCL1));      // Wait for clock low
    else while (digitalRead(HCL2));  
    if (digitalRead(HDA1)) axis[Motor].HBXData |= 0x01;   // Read next bit
  }
  return(true);
}

void HBXMonitorMessage(unsigned char Motor) {
//  dbgSerial.write('<'); 
  if (HBXMonitorByte(Motor))
    axis[Motor].Command = axis[Motor].HBXData; 
  
  switch (axis[Motor].Command) {

    case GetStatus:                    // Four bytes of data
      axis[Motor].HBXCount = 4;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP1 = axis[Motor].HBXData;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP2 = axis[Motor].HBXData;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP3 = axis[Motor].HBXData;
      if (HBXMonitorBit(Motor)) axis[Motor].HBXP4 = axis[Motor].HBXData;
      break;
      
    case RotateSlow:                    // Three bytes of data
    case RotateFast:
      axis[Motor].HBXCount = 3;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP1 = axis[Motor].HBXData;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP2 = axis[Motor].HBXData;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP3 = axis[Motor].HBXData;
      break;
  
    case SetOffset:                    // Two bytes of data
      axis[Motor].HBXCount = 2;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP1 = axis[Motor].HBXData;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP2 = axis[Motor].HBXData;
      break;
  
    case SetLEDI:                      // One byte of data
    case GetLEDI:
    case GetMotorType:
      axis[Motor].HBXCount = 1;
      if (HBXMonitorByte(Motor)) axis[Motor].HBXP1 = axis[Motor].HBXData;
      break;
  
    case CalibrateLED:                  // No data
    case Stop:
    case SlewReverse:
    case SlewForward:
    case ResetH2X:
      axis[Motor].HBXCount = 0;
      break;

    default:
      break;
  }
  if (Motor == MotorAz) while (!digitalRead(HCL1));   // Wait for clock high
  else while (!digitalRead(HCL2));
}

void HBXPrintState(unsigned char Motor) {
  if (axis[Motor].Command != GetStatus) {    // Handle all other commands
    dbgSerial.print(Motor);
    dbgSerial.write(',');
    dbgSerial.print(axis[Motor].TimeDelta);
    dbgSerial.write(',');
    dbgSerial.print(axis[Motor].Command, HEX);
    if (axis[Motor].HBXCount) {
      dbgSerial.write(',');
      dbgSerial.print(axis[Motor].HBXP1);
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      dbgSerial.write(',');
      dbgSerial.print(axis[Motor].HBXP2);
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      dbgSerial.write(',');
      dbgSerial.print(axis[Motor].HBXP3);
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      dbgSerial.write(',');
      dbgSerial.print(axis[Motor].HBXP4, HEX);
    }
  }
  
  // Handle report status - exclude all 0 - ie nothing happened
  else if (axis[Motor].HBXP1 | axis[Motor].HBXP2 | axis[Motor].HBXP3 | axis[Motor].HBXP4 | axis[Motor].PrintStatus0) {
    dbgSerial.print(Motor);
    dbgSerial.write(',');
    dbgSerial.print(axis[Motor].TimeDelta);
    dbgSerial.write(',');
    dbgSerial.print(axis[Motor].Command);
    dbgSerial.write(',');
    dbgSerial.print((axis[Motor].HBXP1<<8) + axis[Motor].HBXP2);
    dbgSerial.write(',');
    dbgSerial.print(axis[Motor].HBXP3);
    dbgSerial.write(',');
    dbgSerial.print(axis[Motor].HBXP4, HEX);
  }
  dbgSerial.println("");
}



