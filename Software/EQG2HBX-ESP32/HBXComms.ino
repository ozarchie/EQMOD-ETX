/**@file*/
/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  HBX Comms related functions
  ===========================
 *********************************************************/

 // HBX Attempt to reset
 // --------------------
void HBXMotorReset(unsigned char Motor)
{
	/*
	int i;

	// Write LOW
	HDATalk();
	digitalWrite(HDA1, LOW);
	TimerDelayuS(HBXBitTime);
	for (i = 0; i < 8; i++)
	{
		if (Motor == MotorAz) digitalWrite(HCL1, LOW);
		else digitalWrite(HCL2, LOW);
		TimerDelayuS(HBXBitTime);
		if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
		else digitalWrite(HCL2, HIGH);
		TimerDelayuS(HBXBitTime);
	}

	// Write HIGH
	digitalWrite(HDA1, HIGH);
	TimerDelayuS(HBXBitTime);
	for (i = 0; i < 8; i++)
	{
		if (Motor == MotorAz) digitalWrite(HCL1, LOW);
		else digitalWrite(HCL2, LOW);
		TimerDelayuS(HBXBitTime);
		if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
		else digitalWrite(HCL2, HIGH);
		TimerDelayuS(HBXBitTime);
	}

	// Read, and discard, a byte
	HDAListen();
	for (i = 0; i < 8; i++) {
		if (Motor == MotorAz) digitalWrite(HCL1, LOW);
		else digitalWrite(HCL2, LOW);
		TimerDelayuS(HBXBitTime);
		if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
		else digitalWrite(HCL2, HIGH);
		TimerDelayuS(HBXBitTime);
	}
	*/

	// Force Clock Low, High for reset, time ~.75s
	if (Motor == MotorAz) digitalWrite(HCLAz, LOW);
	else digitalWrite(HCLAlt, LOW);
	TimerDelaymS(MOTORDETECT);
	if (Motor == MotorAz) digitalWrite(HCLAz, HIGH);
	else digitalWrite(HCLAlt, HIGH);
	TimerDelaymS(MOTORDETECT >> 1);
}

// HBX transmission functions
// ==========================

// HBX Send a command
// ------------------
bool HBXSendCommand(unsigned char Command, unsigned char Motor) {

	if (Command != GetStatus){
		dbgSerial.println(""); dbgSerial.print("+++ "); dbgSerial.print(Motor);
	}
  axis[Motor].Command = Command;

	// Select the interface
	if (Motor == MotorAz) {
		HDA = HDAAz;
		HCL = HCLAz;
	}
	else {
		HDA = HDAAlt;
		HCL = HCLAlt;
	}

// Send the start sequence
// -----------------------
  if (HBXStartSequence(Motor)) { 
// Send the command byte
// ---------------------
  	HBXSendByte(Command, Motor);
  	return(true);
  }
	else {
		HDAListen();					// Set data inbound
		return(false);
	}
}

// HBX Initiate start sequence
// ---------------------------
bool HBXStartSequence(unsigned char Motor) {
// 1. HDA as input, Clock as output
	HDAListen(); 
	HCLTalk();
// 2. Set clock low
	digitalWrite(HCL, LOW);
  TimerDelayuS(HBXBitTime >> 1);		// 1/2 bit-time
// 3. Wait for data low (HDA = 0) by MC, or timeout
  H2XStart = micros();
  do {
    H2XTimer = micros() - H2XStart;
  } while ((digitalRead(HDA) == 1) && (H2XTimer < (HBXBitTime << 3)));
  TimerDelayuS((HBXBitTime >> 5));	// 1/32 bit-time delay, in case of data line glitch
// 4. Re-read data line, check if (data low) or (MC timeout)
  if ((digitalRead(HDA) == 1) || (H2XTimer >= (HBXBitTime << 3))) {
    digitalWrite(HCL, HIGH);
    return(false);                  //  error exit if no response from Motor
  }
// 5. Set clock high if data low occurred (i.e. MC acknowledged clock low)
	digitalWrite(HCL, HIGH);
  TimerDelayuS(HBXBitTime >> 1);
// 6. Wait for data line release (HDA = 1) by MC,  or timeout
  H2XStart = micros();
  do {
    H2XTimer = micros() - H2XStart;
  } while ((digitalRead(HDA) == 0) && (H2XTimer < (HBXBitTime << 3)));
  TimerDelayuS(HBXBitTime);					// Wait one bit-time, in case of success
// 7. Check timeout for data line released or no response from MC
  if (H2XTimer >= (HBXBitTime << 3)) {
    return(false);                  // Error Exit if no response from MC
  }
  return(true);                     // Success                     
}

// HBX Send a single byte
// ----------------------
void HBXSendByte(unsigned char databyte, unsigned char Motor) {

	unsigned char mask;
	if (axis[Motor].Command != GetStatus) {
		dbgSerial.print("-> "); dbgSerial.print(databyte, HEX);
	}

	HDATalk();										  // HDA as output
	axis[Motor].HBXBitCount = 8;	  // 8bits to go
	mask = 0x80;									  // MSB first
// Clock was set high before entry	
	TimerDelayuS(HBXBitTime);
 	do {
		axis[Motor].HBXBitCount -= 1;
// Set data bit
		if (databyte & mask) digitalWrite(HDA, HIGH);
		else digitalWrite(HDA, LOW);
    TimerDelayuS(HBXBitTime >> 1);        // Let data stabilise
		mask = mask >> 1;											// Next data bit
// Set clock low
		digitalWrite(HCL, LOW);
    TimerDelayuS(HBXBitTime);
// Set clock high
		digitalWrite(HCL, HIGH);
    TimerDelayuS(HBXBitTime-(HBXBitTime >> 1)); // Data is written DSTABLE before clock low
// for 8 bits
	} 	while (axis[Motor].HBXBitCount);
  TimerDelayuS(HBXBitTime >> 1);        // Last high clock
	HDAListen();													// Turn data pin inbound
	TimerDelayuS(HBXBitTime);
}

// HBX Send two bytes in sequence
// ------------------------------
void HBXSend2Bytes(unsigned char Motor) {
  HBXSendByte(axis[Motor].HBXP1, Motor);
  HBXSendByte(axis[Motor].HBXP2, Motor);
}

// HBX Send three bytes in sequence
// --------------------------------
void HBXSend3Bytes(unsigned char Motor) {
  HBXSendByte(axis[Motor].HBXP1, Motor);
  HBXSendByte(axis[Motor].HBXP2, Motor);
  HBXSendByte(axis[Motor].HBXP3, Motor);
}

// HBX Get a single  byte
// ----------------------
unsigned char HBXGetByte(unsigned char Motor) {

// HDA as input  
  HDAListen();                    
  axis[Motor].HBXBitCount = 8;	
	axis[Motor].HBXData = 0;
// Clock was set high before entry  
	while (axis[Motor].HBXBitCount) {
// Set clock low
		digitalWrite(HCL, LOW);
    TimerDelayuS(HBXBitTime >> 1);
// Read data bit
		axis[Motor].HBXData = axis[Motor].HBXData << 1;				// Shift previous bit
		if (digitalRead(HDA)) axis[Motor].HBXData |=	0x01;		// Read next bit
		axis[Motor].HBXBitCount--;								            // Need eight bits
    TimerDelayuS(HBXBitTime-(HBXBitTime >> 1));           // Wait for low time
// Set clock high
		digitalWrite(HCL, HIGH);
    TimerDelayuS(HBXBitTime);
	}
	TimerDelayuS(HBXBitTime);
	if (axis[Motor].Command != GetStatus) {
		dbgSerial.print("<- "); dbgSerial.print(axis[Motor].HBXData, HEX);
	}
// Return data byte
  axis[Motor].HBXCount = 1;  
	return (axis[Motor].HBXData);
}

// HBX Get the status bytes (25 bits)
// ----------------------------------
void HBXGet3Bytes(unsigned char Motor) {
  
	axis[Motor].HBXP1 = HBXGetByte(Motor);
  TimerDelayuS(HBXBitTime);
	axis[Motor].HBXP2 = HBXGetByte(Motor);
  TimerDelayuS(HBXBitTime);
	axis[Motor].HBXP3 = HBXGetByte(Motor);
  TimerDelayuS(HBXBitTime);
	axis[Motor].HBXP4 = 0;

// Read 'byte4' = error bit
// ------------------------
	digitalWrite(HCL, LOW);
	TimerDelayuS(HBXBitTime >> 1);
	axis[Motor].HBXP4 |= digitalRead(HDA);		  // Read the battery error bit
  TimerDelayuS(HBXBitTime-(HBXBitTime >> 1));
	digitalWrite(HCL, HIGH);
	TimerDelayuS(HBXBitTime);
	if (axis[Motor].Command != GetStatus) {
		dbgSerial.print("- "); dbgSerial.print(axis[Motor].HBXP4, HEX);
	}
  axis[Motor].HBXCount = 4;
}

// H2X Low level Functions
// -----------------------
void HDAListen(void) {
//	digitalWrite(HDA, HIGH);
	pinMode(HDA, H2C_INPUT);
}
void HDAFloat(void) {
  pinMode(HDA, H2C_INPUT);
}
void HDATalk(void) {
  digitalWrite(HDA, HIGH);
  pinMode(HDA, H2C_OUTPUT);
}
void HCLListen(void) {
//	digitalWrite(HCL, HIGH);
	pinMode(HCL, H2C_INPUTPU);
}
void HCLFloat(void) {
	pinMode(HCL, H2C_INPUT);
}
void HCLTalk(void) {
	digitalWrite(HCL, HIGH);
	pinMode(HCL, H2C_OUTPUT);
}
void HDAAuxListen(void) {
	pinMode(HDAAux, H2C_INPUT);
}
void HDAAuxFloat(void) {
	pinMode(HDAAux, H2C_INPUT);
}
void HDAAuxTalk(void) {
	digitalWrite(HDAAux, HIGH);
	pinMode(HDAAux, H2C_OUTPUT);
}
void HCLAuxListen(void) {
	pinMode(HCLAux, H2C_INPUTPU);
}
void HCLAuxTalk(void) {
	digitalWrite(HCLAux, HIGH);
	pinMode(HCLAux, H2C_OUTPUT);
}

void HBXReset() {
	int ClockCount = 0;
	// Do Az first
	// ===========
	HDA = HDAAz;
	HCL = HCLAz;
	// Set clock high
	HCLTalk();
	// Set data inbound
	HDAListen();
	TimerDelayuS(HBXBitTime);
	// Data should be high
	while ((!digitalRead(HDA)) && (ClockCount < 25)) {
		digitalWrite(HCL, LOW);
		TimerDelayuS(HBXBitTime);
		digitalWrite(HCL, HIGH);
		TimerDelayuS(HBXBitTime);
		ClockCount += 1;
	}
	// Then Alt
	// ========
	HDA = HDAAlt;
	HCL = HCLAlt;
	// Set clock high
	HCLTalk();
	// Set data inbound
	HDAListen();
	TimerDelayuS(HBXBitTime);
	// Data should be high
	while ((!digitalRead(HDA)) && (ClockCount < 25)) {
		digitalWrite(HCL, LOW);
		TimerDelayuS(HBXBitTime);
		digitalWrite(HCL, HIGH);
		TimerDelayuS(HBXBitTime);
		ClockCount += 1;
	}
	// Finally AUX
	// Set clock high
	HCLAuxTalk();
	// Set data inbound
	HDAAuxListen();
}

long TwosComplement(long p) {					// Calculate 2s complement
  long q;
	q = ~p;										  // Bitwise invert
	q = q + 1;									// +1
	return q;
}
