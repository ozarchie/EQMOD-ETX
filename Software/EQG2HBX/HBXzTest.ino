/*
 * Copyright 2017, 2018 John Archbold
*/


/********************************************************
  Test HBX communications
  =======================
 *********************************************************/
void HBXTestLoop(void) {
  TestCount = 0;
  while (digitalRead(TESTHBX) == 0) {
    Serial.println("Test HBX commands to ETX");
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
  Serial.println("**********************************************");
  Serial.print("Test Number - ");
  Serial.println(TestCount);
  Serial.println("**********************************************");
//do {  
  digitalWrite(FROMHBX, HIGH);   
  Serial1.println("**********************************************");
//  TimerDelaymS(100);  
  Serial1.print("Test Number - ");
//  TimerDelaymS(100);  
  Serial1.println(TestCount++);
  digitalWrite(FROMHBX, LOW);
   
//} while(1);

    HCL1Talk();                 // Set for Talking on RAClk
    HCL2Talk();                 // Set for Talking on DECClk
    HDAListen();
    TimerDelaymS(STARTTIME);
    axis[AzMotor].PrintStatus0 = 1;           // Enable print of status = no change
    axis[AltMotor].PrintStatus0 = 1;           // Enable print of status = no change

//if (TestCount == 0) {
  
    Serial.println("Test - HBX Initialization");  
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
    Serial.println("Test - Wait for motors");
  
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
    Serial.println("Test - Stop motors");
    HBXStop2Motors();
  
  // Read status
    Serial.println("Test - Read Status");
    HBXGet2Status();          // Check and read both motor states


  
    if (!check_eeprom_crc() || !EEPROM.read(EEPROMAzLEDI) || !EEPROM.read(EEPROMAltLEDI)) {
    // Calibrate Az, Alt Motor Encoder LED currents
      Serial.println("Test - Calibrate RA, DEC Motor Encoder LED currents");
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
  
    Serial.println("Test - Get Motor Type and set ETX Encoder LED currents");
    axis[AzMotor].MotorType = 0x00; 
    while (!axis[AzMotor].MotorType) {
      if (HBXSendCommand(GetMotorType, AzMotor))
        axis[AzMotor].MotorType = HBXGetByte(AzMotor);
      HBXPrintStatus(AzMotor);
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
   
  // Set the Offset to Zero
    axis[AzMotor].HBXP1 = 0x00;             
    axis[AzMotor].HBXP2 = 0x00;
    axis[AzMotor].HBXP3 = 0x00;
    axis[AltMotor].HBXP1 = 0x00;             
    axis[AltMotor].HBXP2 = 0x00;
    axis[AltMotor].HBXP3 = 0x00;
      
  // Set the Offset Clear Command     
    Serial.println("Test - Reset any ETX offset bytes");
    if (HBXSendCommand(SetOffset, AzMotor))
      HBXSend2Bytes(AzMotor);
//    HBXPrintStatus(AzMotor);
    TimerDelaymS(CMNDTIME);
  
    if (HBXSendCommand(SetOffset, AltMotor))
      HBXSend2Bytes(AltMotor);
//    HBXPrintStatus(AltMotor);
    TimerDelaymS(CMNDTIME);

//} 

    Serial.println("Test - Stop motors");
    HBXStop2Motors();       
// First Read, clear counters
    Serial.println("Test - Read Both Motor States");
    HBXGet2Status();

    Serial.println("Test - Begin motor move tests");
//}  
  // Test different motor speeds
    Serial.println("Test - Motor Speed Tests"); 
    Serial.println("========================"); 
 
    Serial.println("Test - SIDEREAL"); 
    axis[AzMotor].HBXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].HBXMotorStatus |= MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzSIDEREALRATE;          // Sidereal
    axis[AltMotor].HBXSpeed = AltSIDEREALRATE;   
    HBXPrintPosn(5, 1000);           // Show location each second   
    Serial.println("Test - SIDEREAL - Stop motors");
    HBXStop2Motors();    

    Serial.println("Test - OneDegree/sec forward"); 
    axis[AzMotor].HBXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].HBXMotorStatus |= MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1;           // One degree/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1;         // One degree/sec
    HBXPrintPosn(5, 500);           // Show location each tenth of a second 
    Serial.println("Test - OneDegree/sec forward - Stop motors");
    HBXStop2Motors();

    Serial.println("Test - OneDegree/sec reverse"); 
    axis[AzMotor].HBXMotorStatus &= ~MOVEDIRN;          // Reverse
    axis[AltMotor].HBXMotorStatus &= ~MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1;           // One degree/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1;         // One degree/sec
    HBXPrintPosn(5, 500);           // Show location each each tenth of a second    
    Serial.println("Test - OneDegree/sec reverse - Stop motors");
    HBXStop2Motors();
      
    Serial.println("Test - TwoDegrees/sec forward"); 
    axis[AzMotor].HBXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].HBXMotorStatus |= MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1*2;         // Two degrees/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1*2;       // Two degrees/sec
    HBXPrintPosn(5, 1000);           // Show location each tenth of a second 
    Serial.println("Test - TwoDegrees/sec forward - Stop motors");
    HBXStop2Motors();

    Serial.println("Test - TwoDegrees/sec reverse"); 
    axis[AzMotor].HBXMotorStatus &= ~MOVEDIRN;          // Reverse
    axis[AltMotor].HBXMotorStatus &= ~MOVEDIRN;
    axis[AzMotor].HBXSpeed = AzDEGREERATE1*2;         // Two degrees/sec
    axis[AltMotor].HBXSpeed = AltDEGREERATE1*2;       // Two degrees/sec
    HBXPrintPosn(5, 1000);           // Show location each each tenth of a second    
    Serial.println("Test - TwoDegrees/sec reverse - Stop motors");
    HBXStop2Motors();

// Read status
  Serial.println("Test - Read Status");
  HBXGet2Status();

  // Stop the motors (Az and Alt) 
  Serial.println("Test - Stop motors");
  HBXStop2Motors();  
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
      SendPosn(duration);             // Duration is delay between reads
      j += 1;
    } while(j < count);
}

void SendPosn(unsigned int duration) {

//    Serial.println("  SendPosn"); 

    while ((millis() - PreviousTime) < duration) ;  // 1/10 second
    Serial.print(millis() - PreviousTime);
    Serial.print(" - ");
    PreviousTime = millis();

    HBXGetStatus(AzMotor);
    axis[AzMotor].HBXDelta = axis[AzMotor].HBXP1 * 256 + axis[AzMotor].HBXP2;

    HBXGetStatus(AltMotor);    
    axis[AltMotor].HBXDelta = axis[AltMotor].HBXP1 * 256 + axis[AltMotor].HBXP2;
       
    Serial.print("Az = ");
    Serial.print(axis[AzMotor].HBXPosn);
    Serial.print(" : ");
    Serial.print(axis[AzMotor].HBXDelta);    
    Serial.print(", Alt = ");
    Serial.print(axis[AltMotor].HBXPosn);
    Serial.print(" : ");
    Serial.println(axis[AltMotor].HBXDelta);
}

void HBXPrintStatus(unsigned char Motor) {
  axis[Motor].HBXCount = 0;
  if ((axis[Motor].HBXCmnd != GetStatus) || (axis[Motor].HBXP1 | axis[Motor].HBXP2 | axis[Motor].HBXP3 | axis[Motor].HBXP4) || axis[Motor].PrintStatus0 ) {
    Serial.print("Motor: ");
    Serial.print(Motor);
    Serial.print(", Cmnd: ");
    Serial.print(axis[Motor].HBXCmnd, HEX);
    Serial.print(" -  ");
  
      switch (axis[Motor].HBXCmnd) {
        case RotateSlow:
            Serial.print("RotateSlow ");
            axis[Motor].HBXCount = 3;
            break;
        case RotateFast:
            Serial.print("RotateFast ");
            axis[Motor].HBXCount = 3;
            break;
        case SetOffset:
            Serial.print("SetOffset ");
            axis[Motor].HBXCount = 4;
            break;
        case SetLEDI:
            Serial.print("SetLEDI ");
            axis[Motor].HBXCount = 1;
            break;
        case CalibrateLED:
            Serial.print("CalibrateLED ");
            break;
        case Stop:
            Serial.print("Stop ");
            break;
        case SlewReverse:
            Serial.print("SlewReverse ");
            break;
        case SlewForward:
            Serial.print("SlewForward ");
            break;
        case GetStatus:
            Serial.print("GetStatus ");
            axis[Motor].HBXCount = 4;
            break;
        case GetLEDI:
            Serial.print("GetLEDI ");
            axis[Motor].HBXCount = 1;
            break;
        case GetMotorType:
            Serial.print("GetMotorType ");
            axis[Motor].HBXP1 = axis[Motor].MotorType;
            axis[Motor].HBXCount = 1;
            break;
        case ResetH2X:
            Serial.print("ResetH2X ");
            break;
        default:
            Serial.print("UNKNOWN ");
            break;
      }
      
    if (axis[Motor].HBXCount) {
      Serial.print(", Data: ");
      Serial.print(axis[Motor].HBXP1, HEX);
      if (axis[Motor].HBXCount > 1)  Serial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      Serial.print(axis[Motor].HBXP2, HEX);
      if (axis[Motor].HBXCount > 1)  Serial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      Serial.print(axis[Motor].HBXP3, HEX);
      if (axis[Motor].HBXCount > 1)  Serial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
      Serial.print(axis[Motor].HBXP4, HEX);
      axis[Motor].HBXCount -= 1;
    }
    Serial.println("");
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
  ETXState(AzMotor);
  axis[AltMotor].ETXMotorState = ETXCheckStartup;
  axis[AltMotor].HBXMotorStatus |= MOVEAXIS;
  ETXState(AltMotor);
  return(true);    
}

void HBXPrintPosn(unsigned char Motor) {
  if (Motor == MotorAz) {
    digitalWrite(FROMHBX, HIGH);    // Set Indicator LED
    Serial.println("");
    Serial.print("Time: ");
    Serial.print(millis());
//    Serial.print(StatusTimer - StatusTime);
//    StatusTime = StatusTimer;


/*    Serial.print(" AzRaw: ");
    puthexb(axis[AzMotor].HBXP1);
    putbyte(',');
    puthexb(axis[AzMotor].HBXP2);
    putbyte(',');
    puthexb(axis[AzMotor].HBXP3);
    putbyte(',');
    puthexb(axis[AzMotor].HBXP4);
*/
    Serial.print("  AzPosn: ");
    puthexl(axis[AzMotor].HBXPosn);
    putbyte(',');
    puthexl(axis[AzMotor].HBXTarget);
    putbyte(',');
    puthexl(axis[AzMotor].HBXSlowDown);
    Serial.print("  AzSpeed: ");
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
 *   Serial.print(" AltRaw: ");
    puthexb(axis[AltMotor].HBXP1);
    putbyte(',');
    puthexb(axis[AltMotor].HBXP2);
    putbyte(',');
    puthexb(axis[AltMotor].HBXP3);
    putbyte(',');
    puthexb(axis[AltMotor].HBXP4);
    */
    Serial.print(" AltPosn: ");
    puthexl(axis[AltMotor].HBXPosn);
    putbyte(',');
    puthexl(axis[AltMotor].HBXTarget);
    putbyte(',');
    puthexl(axis[AltMotor].HBXSlowDown);
    Serial.print(" AltSpeed: ");
    puthexl(axis[AltMotor].HBXSpeed);
    putbyte(',');
    puthexl(axis[AltMotor].HBXTargetSpeed);
    putbyte('-');
    puthexw(axis[AltMotor].HBXMotorStatus);
    putbyte(',');
    puthexw(axis[AltMotor].HBXMotorControl);
    Serial.println("");
  }
    digitalWrite(FROMHBX, LOW);     // Clear Indicator LED
}


