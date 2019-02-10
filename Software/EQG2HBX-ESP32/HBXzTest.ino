/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  Test HBX communications
  =======================
 *********************************************************/
void HBXTestLoop(void) {
  TestCount = 0;
	//while (digitalRead(TESTHBX) == 0) {
	while (1) {
			dbgSerial.println("Test HBX commands to ETX");
    HBXTest();
    TestCount += 1;
    TestLoopTime = millis();
    // Read motor status until jumper removed
    while((millis() - TestLoopTime) < 1000)  { // 5s between tests
//      HBXGet2Status();
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

	HCL = HCLAz;
	HDA = HDAAz;
	HCLTalk();                 // Set for Talking on Clock
	HDAListen();

	HCL = HCLAlt;
	HDA = HDAAlt;
	HCLTalk();                 // Set for Talking on Clock
	HDAListen();

  TimerDelaymS(STARTTIME);
  axis[AzMotor].PrintStatus0 = 1;           // Enable print of status = no change
  axis[AltMotor].PrintStatus0 = 1;           // Enable print of status = no change

//if (TestCount == 0) {
  
    dbgSerial.println("Test - HBX Initialization");  
    axis[AzMotor].Position = ETX_CENTRE;
    axis[AzMotor].Target = axis[AzMotor].Position;
    axis[AzMotor].DirnSpeed = 0x000;
    axis[AzMotor].Speed = 0x000000;
    axis[AzMotor].ETXMotorStatus = MOVESLEW;

    axis[AltMotor].Position = ETX_CENTRE;
    axis[AltMotor].Target = axis[AltMotor].Position;
    axis[AltMotor].DirnSpeed = 0x000;
    axis[AltMotor].Speed = 0x000000;
    axis[AltMotor].ETXMotorStatus = MOVESLEW;
  
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
		dbgSerial.println("");
		dbgSerial.println("1 Test - Read Status");
		HBXGet2Status();          // Check and read both motor states

		dbgSerial.println("Test - Get Motor Type and set ETX Encoder LED currents");
		axis[AzMotor].MotorType = 0x00;
		while (!axis[AzMotor].MotorType) {
			if (HBXSendCommand(GetMotorType, AzMotor))
				axis[AzMotor].MotorType = HBXGetByte(AzMotor);
			HBXPrintStatus(AzMotor);
		}

		preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace

// Handle position sensors LED current
// -----------------------------------
		if (preferences.getUChar("AzLEDI", 0) == 0) {		// If it does not exist, return 0
	// Calibrate motors
			if (HBXSendCommand(CalibrateLED, AzMotor));
			TimerDelaymS(2500);
			if (HBXSendCommand(CalibrateLED, AltMotor))
				TimerDelaymS(2500);
			// Read the calibration
			dbgSerial.print("Read LEDs - AzMotor: ");
			if (HBXSendCommand(GetLEDI, AzMotor))
				axis[AzMotor].HBXLEDI = HBXGetByte(AzMotor);
			dbgSerial.print(axis[AzMotor].HBXLEDI);
			dbgSerial.print(", AltMotor: ");
			if (HBXSendCommand(GetLEDI, AltMotor))
				axis[AltMotor].HBXLEDI = HBXGetByte(AltMotor);
			dbgSerial.println(axis[AltMotor].HBXLEDI);
			// Save it to preferences
			preferences.putUChar("AzLEDI", axis[AzMotor].HBXLEDI);
			preferences.putUChar("AltLEDI", axis[AltMotor].HBXLEDI);
		}
		// Read stored LED currents
		axis[AzMotor].HBXLEDI = preferences.getUChar("AzLEDI", 0);
		axis[AltMotor].HBXLEDI = preferences.getUChar("AltLEDI", 0);
		preferences.end();

		// Set the MC values
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
    dbgSerial.println("Test - Reset any ETX offset bytes");
    if (HBXSendCommand(SetOffset, AzMotor))
      HBXSend2Bytes(AzMotor);
    HBXPrintStatus(AzMotor);
    TimerDelaymS(CMNDTIME);
  
    if (HBXSendCommand(SetOffset, AltMotor))
      HBXSend2Bytes(AltMotor);
    HBXPrintStatus(AltMotor);
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
    axis[AzMotor].ETXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].ETXMotorStatus |= MOVEDIRN;
    axis[AzMotor].Speed = AzSIDEREALRATE;          // Sidereal
    axis[AltMotor].Speed = AltSIDEREALRATE;   
    HBXPrintPosn(5, 1000);           // Show location each second   
    dbgSerial.println("Test - SIDEREAL - Stop motors");
    HBXStop2Motors();    

    dbgSerial.println("Test - OneDegree/sec forward"); 
    axis[AzMotor].ETXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].ETXMotorStatus |= MOVEDIRN;
    axis[AzMotor].Speed = AzDEGREERATE1;           // One degree/sec
    axis[AltMotor].Speed = AltDEGREERATE1;         // One degree/sec
    HBXPrintPosn(5, 500);           // Show location each tenth of a second 
    dbgSerial.println("Test - OneDegree/sec forward - Stop motors");
    HBXStop2Motors();

    dbgSerial.println("Test - OneDegree/sec reverse"); 
    axis[AzMotor].ETXMotorStatus &= ~MOVEDIRN;          // Reverse
    axis[AltMotor].ETXMotorStatus &= ~MOVEDIRN;
    axis[AzMotor].Speed = AzDEGREERATE1;           // One degree/sec
    axis[AltMotor].Speed = AltDEGREERATE1;         // One degree/sec
    HBXPrintPosn(5, 500);           // Show location each each tenth of a second    
    dbgSerial.println("Test - OneDegree/sec reverse - Stop motors");
    HBXStop2Motors();
      
    dbgSerial.println("Test - TwoDegrees/sec forward"); 
    axis[AzMotor].ETXMotorStatus |= MOVEDIRN;        // Forward
    axis[AltMotor].ETXMotorStatus |= MOVEDIRN;
    axis[AzMotor].Speed = AzDEGREERATE1*2;         // Two degrees/sec
    axis[AltMotor].Speed = AltDEGREERATE1*2;       // Two degrees/sec
    HBXPrintPosn(5, 1000);           // Show location each tenth of a second 
    dbgSerial.println("Test - TwoDegrees/sec forward - Stop motors");
    HBXStop2Motors();

    dbgSerial.println("Test - TwoDegrees/sec reverse"); 
    axis[AzMotor].ETXMotorStatus &= ~MOVEDIRN;          // Reverse
    axis[AltMotor].ETXMotorStatus &= ~MOVEDIRN;
    axis[AzMotor].Speed = AzDEGREERATE1*2;         // Two degrees/sec
    axis[AltMotor].Speed = AltDEGREERATE1*2;       // Two degrees/sec
    HBXPrintPosn(5, 1000);           // Show location each each tenth of a second    
    dbgSerial.println("Test - TwoDegrees/sec reverse - Stop motors");
*/
    HBXStop2Motors();

		dbgSerial.println("");
		dbgSerial.println("Test - 0x01 Command");
		axis[AzMotor].MotorControl  &= ~MOVEDECR;							// Forward
    axis[AltMotor].MotorControl &= ~MOVEDECR;
    axis[AzMotor].MotorControl |= SpeedHBX;								// High speed
    axis[AltMotor].MotorControl |= SpeedHBX;
    axis[AzMotor].Speed  = axis[AzMotor].SIDEREALRATE;    // Start at Sidereal
    axis[AltMotor].Speed = axis[AltMotor].SIDEREALRATE;
   
    do {
 //     HBXPrintSpeed(1, 10000);													// Print location
      axis[AzMotor].Speed  += axis[AzMotor].SIDEREALRATE;
			axis[AzMotor].ETXMotorState = ETXStepMotor;
			ETXState(AzMotor);
			TimerDelaymS(CMNDTIME);
//			delay(2000);
//    } while (axis[AzMotor].Speed < 0x600000);

//		do {
//			HBXPrintSpeed(1, 10000);													// Print location
			axis[AltMotor].Speed += axis[AltMotor].SIDEREALRATE;
			axis[AltMotor].ETXMotorState = ETXStepMotor;
			ETXState(AltMotor);
			TimerDelaymS(CMNDTIME);
			HBXGet2Status();
			TimerDelaymS(CMNDTIME);
			TimerDelaymS(2000);
		} while (axis[AltMotor].Speed < (axis[AltMotor].SIDEREALRATE << 3));

    dbgSerial.println("Test - 0x01 Command - Stop motors");
    HBXStop2Motors();    

// Read status
		dbgSerial.println("");
		dbgSerial.println("2 Test - Read Status");
		HBXGet2Status();

  // Stop the motors (Az and Alt) 
  dbgSerial.println("Test - Stop motors");
  HBXStop2Motors();  

	dbgSerial.println("Test - End of Loop");

}


void HBXPrintSpeed(unsigned int count, unsigned int duration) {
    int j = 0;
    
    axis[AzMotor].Position = ETX_CENTRE;        // Reset position
    axis[AltMotor].Position = ETX_CENTRE;
    axis[AzMotor].Increment = 0;
    axis[AltMotor].Increment = 0;
    PreviousTime = millis();
    dbgSerial.print("Az, ");
    dbgSerial.println(axis[AzMotor].Speed);  
    dbgSerial.print("Alt, ");
    dbgSerial.println(axis[AltMotor].Speed);
		HBXStart2Motors();                // Start the motors
    do {
      ETXState(AzMotor);            // Check the Az motor state
      ETXState(AltMotor);           // Check the Alt motor state
      dbgSerial.print(millis() - PreviousTime);
      dbgSerial.print(", ");
      SendSpeed(duration);             // Duration is delay between reads
      j += 1;
      while ((millis() - PreviousTime) < duration);
      PreviousTime = millis();
    } while(j < count);
}

void SendSpeed(unsigned long duration) {

    HBXGetStatus(AzMotor);
    axis[AzMotor].Increment = axis[AzMotor].HBXP1 * 256 + axis[AzMotor].HBXP2;
    HBXGetStatus(AltMotor);    
    axis[AltMotor].Increment = axis[AltMotor].HBXP1 * 256 + axis[AltMotor].HBXP2;
       
		dbgSerial.println("");
		dbgSerial.print("Az-Speed: ");
		dbgSerial.print(axis[AzMotor].Speed);
    dbgSerial.print(",Posn: ");
    dbgSerial.print(axis[AzMotor].Position);
    dbgSerial.print(",Incr: ");
    dbgSerial.println(axis[AzMotor].Increment); 
		dbgSerial.print("Alt-Speed: ");
    dbgSerial.print(axis[AltMotor].Speed);
    dbgSerial.print(",Posn: ");
    dbgSerial.print(axis[AltMotor].Position);
    dbgSerial.print(",Incr: ");
    dbgSerial.println(axis[AltMotor].Increment);
}

void HBXPrintPosn(unsigned int count, unsigned int duration) {
    int j = 0;
    
    axis[AzMotor].Position = ETX_CENTRE;        // Reset position
    axis[AltMotor].Position = ETX_CENTRE;
    axis[AzMotor].Increment = 0;
    axis[AltMotor].Increment = 0;
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
    axis[AzMotor].Increment = axis[AzMotor].HBXP1 * 256 + axis[AzMotor].HBXP2;

    HBXGetStatus(AltMotor);    
    axis[AltMotor].Increment = axis[AltMotor].HBXP1 * 256 + axis[AltMotor].HBXP2;
       
    dbgSerial.print("Az = ");
    dbgSerial.print(axis[AzMotor].Position);
    dbgSerial.print(" : ");
    dbgSerial.print(axis[AzMotor].Increment);    
    dbgSerial.print(", Alt = ");
    dbgSerial.print(axis[AltMotor].Position);
    dbgSerial.print(" : ");
    dbgSerial.println(axis[AltMotor].Increment);
}

void HBXPrintStatus(unsigned char Motor) {
  axis[Motor].HBXCount = 0;
  if ((axis[Motor].Command != GetStatus) || (axis[Motor].HBXP1 | axis[Motor].HBXP2 | axis[Motor].HBXP3 | axis[Motor].HBXP4) || axis[Motor].PrintStatus0 ) {

/*		dbgSerial.println(""); 
		dbgSerial.print("Motor: ");
    dbgSerial.print(Motor);
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
      
    if (axis[Motor].HBXCount) {
//      dbgSerial.print(", Data: ");
//      dbgSerial.print(axis[Motor].HBXP1, HEX);
      if (axis[Motor].HBXCount > 1)  dbgSerial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
//      dbgSerial.print(axis[Motor].HBXP2, HEX);
      if (axis[Motor].HBXCount > 1)  dbgSerial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
//      dbgSerial.print(axis[Motor].HBXP3, HEX);
      if (axis[Motor].HBXCount > 1)  dbgSerial.print(", ");
      axis[Motor].HBXCount -= 1;
    }
    if (axis[Motor].HBXCount) {
//      dbgSerial.print(axis[Motor].HBXP4, HEX);
      axis[Motor].HBXCount -= 1;
    }
//    dbgSerial.println("");
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
  axis[AzMotor].EQGMotorStatus |= MOVEAXIS;
    dbgSerial.print("Az, ");
    dbgSerial.println(axis[AzMotor].Speed);  
  ETXState(AzMotor);
    dbgSerial.print("Az, ");
    dbgSerial.println(axis[AzMotor].Speed);  
  axis[AltMotor].ETXMotorState = ETXCheckStartup;
  axis[AltMotor].EQGMotorStatus |= MOVEAXIS;
    dbgSerial.print("Az, ");
    dbgSerial.println(axis[AzMotor].Speed);  
  ETXState(AltMotor);
    dbgSerial.print("Az, ");
    dbgSerial.println(axis[AzMotor].Speed);  
  return(true);    
}

void HBXPrintPosn(unsigned char Motor) {
  if (Motor == MotorAz) {
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
    puthexl(axis[AzMotor].Position);
    putbyte(',');
    puthexl(axis[AzMotor].Target);
    putbyte(',');
    puthexl(axis[AzMotor].SlowDown);
    dbgSerial.print("  AzSpeed: ");
    puthexl(axis[AzMotor].Speed);
    putbyte(',');
    puthexl(axis[AzMotor].TargetSpeed);
    putbyte('-');
    puthexw(axis[AzMotor].EQGMotorStatus);
    putbyte(',');
    puthexw(axis[AzMotor].MotorControl);

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
    puthexl(axis[AltMotor].Position);
    putbyte(',');
    puthexl(axis[AltMotor].Target);
    putbyte(',');
    puthexl(axis[AltMotor].SlowDown);
    dbgSerial.print(" AltSpeed: ");
    puthexl(axis[AltMotor].Speed);
    putbyte(',');
    puthexl(axis[AltMotor].TargetSpeed);
    putbyte('-');
    puthexw(axis[AltMotor].EQGMotorStatus);
    putbyte(',');
    puthexw(axis[AltMotor].MotorControl);
    dbgSerial.println("");
  }
}



