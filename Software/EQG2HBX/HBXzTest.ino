/*
 * Copyright 2017, 2018 John Archbold
*/


#include <Arduino.h>

/********************************************************
  Test HBX communications
  =======================
 *********************************************************/
void HBXTestLoop(void) {
  TestCount = 0;
  while (digitalRead(TESTHBX) == 0) {
    dbgSerial.println("Test HBX commands to ETX");
    HBXTest();
    TestCount += 1;
    TestLoopTime = millis();
    // Read motor status until jumper removed
    while((millis() - TestLoopTime) < 5000)  { // 5s between tests
      HBXGet2Status();
    }
  }
}
  
void HBXTest(void)
{	
  int i;

  // Initialize HBX communications as outputs
  // It will use I2C-like communications
  dbgSerial.println("**********************************************");
  dbgSerial.print("Test Number - ");
  dbgSerial.println(TestCount);
  dbgSerial.println("**********************************************");

    HCL1Talk();                 // Set for Talking on RAClk
    HCL2Talk();                 // Set for Talking on DECClk
    HDAListen();
    TimerDelaymS(STARTTIME);
    axis[AzMotor].PrintStatus0 = 1;           // Enable print of status = no change
    axis[AltMotor].PrintStatus0 = 1;           // Enable print of status = no change

//if (TestCount == 0) {
  
    dbgSerial.println("Test - HBX Initialization");  
    axis[AzMotor].HBXPosn = ETX_CENTRE;
    axis[AzMotor].HBXTarget = axis[AzMotor].HBXPosn;
    axis[AzMotor].HBXDirSpeed = 0x000;
    axis[AzMotor].HBXSpeed = 0x000000;
    axis[AzMotor].HBXMotorStatus = MOVESTEP;

    axis[AltMotor].HBXPosn = ETX_CENTRE;
    axis[AltMotor].HBXTarget = axis[AltMotor].HBXPosn;
    axis[AltMotor].HBXDirSpeed = 0x000;
    axis[AltMotor].HBXSpeed = 0x000000;
    axis[AltMotor].HBXMotorStatus = MOVESTEP;
  
    // Reset the motors (RA and DEC)  
    dbgSerial.println("Test - Wait for motors");
  
    // GetLED commands always return a vaild value - motors not online until this happens
    // Valid values are not 0 and not 0xFF for Az, Alt. (exception here is if LEDRA || LEDAlt == 0xff)
    
    do {
      P1 = 0;
      if (HBXSendCommand(GetLEDI, AzMotor))
        P2 = HBXGetByte(AzMotor);
      if ((P2 != 0) && (P2 != 0xFF)) P1 += 1;
      TimerDelaymS(CMNDTIME);
      if (HBXSendCommand(GetLEDI, AltMotor))
        P2 = HBXGetByte(AltMotor);
      if ((P2 != 0) && (P2 != 0xFF)) P1 += 1;
      TimerDelaymS(CMNDTIME);
    } while (P1 < 2);

      
  // Stop the motors (Az and Alt) 
    dbgSerial.println("Test - Stop motors");
    HBXStop2Motors();
  
  // Read status
    dbgSerial.println("Test - Read Status");
    HBXGet2Status();          // Check and read both motor states
#ifndef mDue
    if (!check_eeprom_crc() || !EEPROM.read(EEPROMAzLEDI) || !EEPROM.read(EEPROMAltLEDI)) {
    // Calibrate Az, Alt Motor Encoder LED currents
      dbgSerial.println("Test - Calibrate RA, DEC Motor Encoder LED currents");
      if (HBXSendCommand(CalibrateLED, AzMotor));
      TimerDelaymS(2500);
      HBXPrintStatus(AzMotor);
      
      if (HBXSendCommand(CalibrateLED, AltMotor))
      TimerDelaymS(2500);
      HBXPrintStatus(AltMotor);
      
      if (HBXSendCommand(GetLEDI, AzMotor))
        axis[AzMotor].HBXLEDI = HBXGetByte(AzMotor);
      EEPROM.write(EEPROMAzLEDI, axis[AzMotor].HBXLEDI);
      set_eeprom_crc();
      axis[AzMotor].HBXP1 = axis[AzMotor].HBXLEDI;
      HBXPrintStatus(AzMotor); 
      if (HBXSendCommand(GetLEDI, AltMotor))
        axis[AltMotor].HBXLEDI = HBXGetByte(AltMotor);
      EEPROM.write(EEPROMAltLEDI, axis[AltMotor].HBXLEDI);
      set_eeprom_crc();
      axis[AltMotor].HBXP1 = axis[AltMotor].HBXLEDI;
      HBXPrintStatus(AltMotor);
    }

    axis[AzMotor].HBXLEDI = EEPROM.read(EEPROMAzLEDI);
    axis[AltMotor].HBXLEDI = EEPROM.read(EEPROMAltLEDI);

    if (HBXSendCommand(SetLEDI, AzMotor))
      HBXSendByte(axis[AzMotor].HBXLEDI, AzMotor);
    axis[AzMotor].HBXP1 = axis[AzMotor].HBXLEDI;
    HBXPrintStatus(AzMotor);
   
    if (HBXSendCommand(SetLEDI, AltMotor))
      HBXSendByte(axis[AltMotor].HBXLEDI, AltMotor);
    axis[AltMotor].HBXP1 = axis[AltMotor].HBXLEDI;
    HBXPrintStatus(AltMotor);
    
#endif
  
    dbgSerial.println("Test - Get Motor Type and set ETX Encoder LED currents");
    axis[AzMotor].MotorType = 0x00; 
    while (!axis[AzMotor].MotorType) {
      if (HBXSendCommand(GetMotorType, AzMotor))
        axis[AzMotor].MotorType = HBXGetByte(AzMotor);
      HBXPrintStatus(AzMotor);
    }
     

   
  // Set the Offset to Zero
    axis[AzMotor].HBXP1 = 0x00;             
    axis[AzMotor].HBXP2 = 0x00;
    axis[AzMotor].HBXP3 = 0x00;
    axis[AltMotor].HBXP1 = 0x00;             
    axis[AltMotor].HBXP2 = 0x00;
    axis[AltMotor].HBXP3 = 0x00;
      
  // Set the Offset Clear Command     
    dbgSerial.println("Test - Reset any ETX offset bytes");
    if (HBXSendCommand(SetOffset, AzMotor))
      HBXSend2Bytes(AzMotor);
//    HBXPrintStatus(AzMotor);
    TimerDelaymS(CMNDTIME);
  
    if (HBXSendCommand(SetOffset, AltMotor))
      HBXSend2Bytes(AltMotor);
//    HBXPrintStatus(AltMotor);
    TimerDelaymS(CMNDTIME);

//} 
/*
    dbgSerial.println("Test - Stop motors");
    HBXStop2Motors();       
// First Read, clear counters
    dbgSerial.println("Test - Read Both Motor States");
    HBXGet2Status();

    dbgSerial.println("Test - Begin motor move tests");
//}  
  // Test different motor speeds
    dbgSerial.println("Test - Motor Speed Tests"); 
    dbgSerial.println("========================"); 
 
    dbgSerial.println("Test - SIDEREAL"); 
    axis[AzMotor].HBXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].HBXMotorStatus |= MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzSIDEREALRATE;          // Sidereal
    axis[AltMotor].HBXSpeed = AltSIDEREALRATE;   
    HBXPrintPosn(5, 1000);           // Show location each second   
    dbgSerial.println("Test - SIDEREAL - Stop motors");
    HBXStop2Motors();    

    dbgSerial.println("Test - OneDegree/sec forward"); 
    axis[AzMotor].HBXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].HBXMotorStatus |= MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1;           // One degree/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1;         // One degree/sec
    HBXPrintPosn(5, 500);           // Show location each tenth of a second 
    dbgSerial.println("Test - OneDegree/sec forward - Stop motors");
    HBXStop2Motors();

    dbgSerial.println("Test - OneDegree/sec reverse"); 
    axis[AzMotor].HBXMotorStatus &= ~MOVEDIRN;          // Reverse
    axis[AltMotor].HBXMotorStatus &= ~MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1;           // One degree/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1;         // One degree/sec
    HBXPrintPosn(5, 500);           // Show location each each tenth of a second    
    dbgSerial.println("Test - OneDegree/sec reverse - Stop motors");
    HBXStop2Motors();
      
    dbgSerial.println("Test - TwoDegrees/sec forward"); 
    axis[AzMotor].HBXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].HBXMotorStatus |= MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1*2;         // Two degrees/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1*2;       // Two degrees/sec
    HBXPrintPosn(5, 1000);           // Show location each tenth of a second 
    dbgSerial.println("Test - TwoDegrees/sec forward - Stop motors");
    HBXStop2Motors();

    dbgSerial.println("Test - TwoDegrees/sec reverse"); 
    axis[AzMotor].HBXMotorStatus &= ~MOVEDIRN;          // Reverse
    axis[AltMotor].HBXMotorStatus &= ~MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1*2;         // Two degrees/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1*2;       // Two degrees/sec
    HBXPrintPosn(5, 1000);           // Show location each each tenth of a second    
    dbgSerial.println("Test - TwoDegrees/sec reverse - Stop motors");
*/
    HBXStop2Motors();
    HBXGet2Status();
  
    dbgSerial.println("Test - 0x00 Command");

    HBXStop2Motors();
    HBXGet2Status();
    
    axis[AzMotor].HBXMotorStatus |= (MOVEDIRN | MOVESTEP);        // Forward
    axis[AltMotor].HBXMotorStatus |= (MOVEDIRN | MOVESTEP);
    axis[AzMotor].HBXMotorStatus |= MOVELOW;        // Low speed
    axis[AltMotor].HBXMotorStatus |= MOVELOW;
    axis[AzMotor].HBXTargetSpeed = AzSIDEREALRATE;         // Start at Sidereal
    axis[AltMotor].HBXTargetSpeed = AltSIDEREALRATE;       // Start at Sidereal
    axis[AzMotor].HBXDelta = 0x200;                        // For startup
    axis[AltMotor].HBXDelta = 0x200;                        // For startup
   
    do {
      HBXPrintSpeed(1, 4096);                        // Print location
      axis[AzMotor].HBXTargetSpeed += AzSIDEREALRATE*4;
      axis[AltMotor].HBXTargetSpeed += AltSIDEREALRATE*4;
    } while (axis[AzMotor].HBXTargetSpeed < (long) 128*AzSIDEREALRATE);
    
    dbgSerial.println("Test - 0x00 Command - Stop motors");
    HBXStop2Motors();
    HBXGet2Status();
    
    dbgSerial.println("Test - 0x01 Command");
    axis[AzMotor].HBXMotorStatus |= (MOVEDIRN | MOVESTEP);        // Forward
    axis[AltMotor].HBXMotorStatus |= (MOVEDIRN | MOVESTEP);
    axis[AzMotor].HBXMotorStatus &= ~MOVELOW;        // High speed
    axis[AltMotor].HBXMotorStatus &= ~MOVELOW;
    axis[AzMotor].HBXTargetSpeed = 6460;                    // Start at Sidereal
    axis[AltMotor].HBXTargetSpeed = 6880;                   // Scaled by gear ratio
    axis[AzMotor].HBXDelta = 0x200;                        // For startup
    axis[AltMotor].HBXDelta = 0x200;                        // For startup
   
    do {
      HBXPrintSpeed(1, 4096);                        // Print location
      axis[AzMotor].HBXTargetSpeed += 6460*4;
      axis[AltMotor].HBXTargetSpeed += 6880*4;
    } while (axis[AzMotor].HBXTargetSpeed < (long) 128*6460);
    
    dbgSerial.println("Test - 0x01 Command - Stop motors");
    HBXStop2Motors();  
while(1);
    
// Read status
  dbgSerial.println("Test - Read Status");
  HBXGet2Status();

  // Stop the motors (Az and Alt) 
  dbgSerial.println("Test - Stop motors");
  HBXStop2Motors();  
}


void HBXPrintSpeed(unsigned int count, unsigned int duration) {
    int j = 0;
    
    axis[AzMotor].HBXPosn = ETX_CENTRE;        // Reset position
    axis[AltMotor].HBXPosn = ETX_CENTRE;
    PreviousTime = millis();
 
    HBXStart2Motors();                // Start the motors

    do {
      ETXState(AzMotor);            // Check the Az motor state
      ETXState(AltMotor);           // Check the Alt motor state
      while ((millis() - PreviousTime) < duration);
      dbgSerial.print(millis() - PreviousTime);
      dbgSerial.print(", ");
      PreviousTime = millis();
      SendSpeed();
      j += 1;
    } while(j < count);
}

void SendSpeed() {

    dbgSerial.print(axis[AzMotor].HBXCmnd, HEX);
    dbgSerial.print(", ");
    
    HBXGetStatus(AzMotor);
    axis[AzMotor].HBXDelta = axis[AzMotor].HBXP1 * 256 + axis[AzMotor].HBXP2;
    HBXGetStatus(AltMotor);    
    axis[AltMotor].HBXDelta = axis[AltMotor].HBXP1 * 256 + axis[AltMotor].HBXP2;
       
    dbgSerial.print("Az, ");
    dbgSerial.print(axis[AzMotor].HBXTargetSpeed);
    dbgSerial.print(", ");
    dbgSerial.print(axis[AzMotor].HBXSpeed);
    dbgSerial.print(", ");
    dbgSerial.print(axis[AzMotor].HBXPosn);
    dbgSerial.print(", ");
    dbgSerial.print(axis[AzMotor].HBXDelta);

    dbgSerial.print(", Alt, ");
    dbgSerial.print(axis[AltMotor].HBXTargetSpeed);
    dbgSerial.print(", ");
    dbgSerial.print(axis[AltMotor].HBXSpeed);
    dbgSerial.print(", ");
    dbgSerial.print(axis[AltMotor].HBXPosn);
    dbgSerial.print(", ");
    dbgSerial.println(axis[AltMotor].HBXDelta);


}

void HBXPrintPosn(unsigned int count, unsigned int duration) {
    int j = 0;
    
    axis[AzMotor].HBXPosn = ETX_CENTRE;        // Reset position
    axis[AltMotor].HBXPosn = ETX_CENTRE;
    axis[AzMotor].HBXDelta = 0;
    axis[AltMotor].HBXDelta = 0;
    PreviousTime = millis();
  
    HBXStart2Motors();                // Start the motors

    do {
      ETXState(AzMotor);            // Check the Az motor state
      ETXState(AltMotor);           // Check the Alt motor state
      SendData(duration);             // Duration is delay between reads
      j += 1;
    } while(j < count);
}

void SendData(unsigned int duration) {

//    dbgSerial.println("  SendPosn"); 

    while ((millis() - PreviousTime) < duration) ;  // 1/10 second
    dbgSerial.print(millis() - PreviousTime);
    dbgSerial.print(" - ");
    PreviousTime = millis();

    HBXGetStatus(AzMotor);
    axis[AzMotor].HBXDelta = axis[AzMotor].HBXP1 * 256 + axis[AzMotor].HBXP2;

    HBXGetStatus(AltMotor);    
    axis[AltMotor].HBXDelta = axis[AltMotor].HBXP1 * 256 + axis[AltMotor].HBXP2;
       
    dbgSerial.print("Az = ");
    dbgSerial.print(axis[AzMotor].HBXPosn);
    dbgSerial.print(" : ");
    dbgSerial.print(axis[AzMotor].HBXDelta);    
    dbgSerial.print(", Alt = ");
    dbgSerial.print(axis[AltMotor].HBXPosn);
    dbgSerial.print(" : ");
    dbgSerial.println(axis[AltMotor].HBXDelta);
}

void HBXPrintStatus(unsigned char Motor) {
  axis[Motor].HBXCount = 0;
  if ((axis[Motor].HBXCmnd != GetStatus) || (axis[Motor].HBXP1 | axis[Motor].HBXP2 | axis[Motor].HBXP3 | axis[Motor].HBXP4) || axis[Motor].PrintStatus0 ) {
    dbgSerial.print("Motor: ");
    dbgSerial.print(Motor);
    dbgSerial.print(", Cmnd: ");
    dbgSerial.print(axis[Motor].HBXCmnd, HEX);
    dbgSerial.print(" -  ");
  
      switch (axis[Motor].HBXCmnd) {
        case RotateSlow:
            dbgSerial.print("RotateSlow ");
            axis[Motor].HBXCount = 3;
            break;
        case RotateFast:
            dbgSerial.print("RotateFast ");
            axis[Motor].HBXCount = 3;
            break;
        case SetOffset:
            dbgSerial.print("SetOffset ");
            axis[Motor].HBXCount = 4;
            break;
        case SetLEDI:
            dbgSerial.print("SetLEDI ");
            axis[Motor].HBXCount = 1;
            break;
        case CalibrateLED:
            dbgSerial.print("CalibrateLED ");
            break;
        case Stop:
            dbgSerial.print("Stop ");
            break;
        case SlewReverse:
            dbgSerial.print("SlewReverse ");
            break;
        case SlewForward:
            dbgSerial.print("SlewForward ");
            break;
        case GetStatus:
            dbgSerial.print("GetStatus ");
            axis[Motor].HBXCount = 4;
            break;
        case GetLEDI:
            dbgSerial.print("GetLEDI ");
            axis[Motor].HBXCount = 1;
            break;
        case GetMotorType:
            dbgSerial.print("GetMotorType ");
            axis[Motor].HBXP1 = axis[Motor].MotorType;
            axis[Motor].HBXCount = 1;
            break;
        case ResetH2X:
            dbgSerial.print("ResetH2X ");
            break;
        default:
            dbgSerial.print("UNKNOWN ");
            break;
      }
      
    if (axis[Motor].HBXCount) {
      dbgSerial.print(", Data: ");
      dbgSerial.print(axis[Motor].HBXP1, HEX);
      if (axis[Motor].HBXCount > 1)  dbgSerial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      dbgSerial.print(axis[Motor].HBXP2, HEX);
      if (axis[Motor].HBXCount > 1)  dbgSerial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      dbgSerial.print(axis[Motor].HBXP3, HEX);
      if (axis[Motor].HBXCount > 1)  dbgSerial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      dbgSerial.print(axis[Motor].HBXP4, HEX);
      axis[Motor].HBXCount -= 1;
    }
    dbgSerial.println("");
  }
}

bool HBXStop2Motors(void) {
  axis[AzMotor].ETXMotorState = ETXStopMotor;
  ETXState(AzMotor);
  axis[AltMotor].ETXMotorState = ETXStopMotor;
  ETXState(AltMotor);
  return(true);  
}

bool HBXStart2Motors(void) {
  axis[AzMotor].ETXMotorState = ETXCheckStartup;
  axis[AzMotor].HBXMotorStatus |= MOVEAXIS;
//    dbgSerial.print("1Az, ");
//    dbgSerial.println(axis[AzMotor].HBXSpeed);  
  ETXState(AzMotor);
//    dbgSerial.print("2Az, ");
//    dbgSerial.println(axis[AzMotor].HBXSpeed);  
  axis[AltMotor].ETXMotorState = ETXCheckStartup;
  axis[AltMotor].HBXMotorStatus |= MOVEAXIS;
//    dbgSerial.print("3Az, ");
//    dbgSerial.println(axis[AzMotor].HBXSpeed);  
  ETXState(AltMotor);
//    dbgSerial.print("4Az, ");
//    dbgSerial.println(axis[AzMotor].HBXSpeed);  
  return(true);    
}

void HBXPrintPosn(unsigned char Motor) {
  if (Motor == MotorAz) {
    digitalWrite(FROMHBX, HIGH);    // Set Indicator LED
    dbgSerial.println("");
    dbgSerial.print("Time: ");
    dbgSerial.print(millis());
//    dbgSerial.print(StatusTimer - StatusTime);
//    StatusTime = StatusTimer;


/*    dbgSerial.print(" AzRaw: ");
    puthexb(axis[AzMotor].HBXP1);
    putbyte(',');
    puthexb(axis[AzMotor].HBXP2);
    putbyte(',');
    puthexb(axis[AzMotor].HBXP3);
    putbyte(',');
    puthexb(axis[AzMotor].HBXP4);
*/
    dbgSerial.print("  AzPosn: ");
    puthexl(axis[AzMotor].HBXPosn);
    putbyte(',');
    puthexl(axis[AzMotor].HBXTarget);
    putbyte(',');
    puthexl(axis[AzMotor].HBXSlowDown);
    dbgSerial.print("  AzSpeed: ");
    puthexl(axis[AzMotor].HBXSpeed);
    putbyte(',');
    puthexl(axis[AzMotor].HBXTargetSpeed);
    putbyte('-');
    puthexw(axis[AzMotor].HBXMotorStatus);
    putbyte(',');
    puthexw(axis[AzMotor].HBXMotorControl);

  }
  else {    
/*  
 *   dbgSerial.print(" AltRaw: ");
    puthexb(axis[AltMotor].HBXP1);
    putbyte(',');
    puthexb(axis[AltMotor].HBXP2);
    putbyte(',');
    puthexb(axis[AltMotor].HBXP3);
    putbyte(',');
    puthexb(axis[AltMotor].HBXP4);
    */
    dbgSerial.print(" AltPosn: ");
    puthexl(axis[AltMotor].HBXPosn);
    putbyte(',');
    puthexl(axis[AltMotor].HBXTarget);
    putbyte(',');
    puthexl(axis[AltMotor].HBXSlowDown);
    dbgSerial.print(" AltSpeed: ");
    puthexl(axis[AltMotor].HBXSpeed);
    putbyte(',');
    puthexl(axis[AltMotor].HBXTargetSpeed);
    putbyte('-');
    puthexw(axis[AltMotor].HBXMotorStatus);
    putbyte(',');
    puthexw(axis[AltMotor].HBXMotorControl);
    dbgSerial.println("");
  }
    digitalWrite(FROMHBX, LOW);     // Clear Indicator LED
}


