/*
 * Copyright 2017, 2018 John Archbold
*/
#include <Arduino.h>

/********************************************************
  HBX Comms related functions
  ===========================
 *********************************************************/

 // HBX Attempt to reset
 // --------------------
void HBXMotorReset(unsigned char Motor)
{
	int i;
	/*
	// Write LOW
	HDATalk();
	digitalWrite(HDA1, HIGH);
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
	digitalWrite(HDA1, LOW);
	for (i = 0; i < 8; i++)
	{
		if (Motor == MotorAz) digitalWrite(HCL1, LOW);
		else digitalWrite(HCL2, LOW);
		TimerDelayuS(HBXBitTime);
		if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
		else digitalWrite(HCL2, HIGH);
		TimerDelayuS(HBXBitTime);
	}
	*/
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
	// Force Clock Low for reset
	if (Motor == MotorAz) digitalWrite(HCL1, LOW);
	else digitalWrite(HCL2, LOW);
}

// HBX transmission functions
// ==========================

// HBX Send a command
// ------------------
bool HBXSendCommand(unsigned char Command, unsigned char Motor) {

	unsigned long i;
  axis[Motor].Command = Command;
  
// Send the start sequence
// -----------------------
  if (HBXStartSequence(Motor)) { 
// Send the command byte
// ---------------------
  	HBXSendByte(Command, Motor);
  	return(true);
  }
	else {
		return(false);
	}
}

// HBX Initiate start sequence
// ---------------------------
bool HBXStartSequence(unsigned char Motor) {
// 1. HDA as input
	HDAListen();                    
// 2. Set clock low
  if (Motor == MotorAz) digitalWrite(HCL1, LOW);
  else digitalWrite(HCL2, LOW);
  TimerDelayuS(HBXBitTime >> 1);		// 1/2 bit-time
// 3. Wait for data low (HDA1 = 0) by MC, or timeout
  H2XStart = micros();
  do {
    H2XTimer = micros() - H2XStart;
  } while ((digitalRead(HDA1) == 1) && (H2XTimer < (HBXBitTime << 3)));
  TimerDelayuS((HBXBitTime >> 5));	// 1/32 bit-time delay, in case of data line glitch
// 4. Re-read data line, check if (data low) or (MC timeout)
  if ((digitalRead(HDA1) == 1) || (H2XTimer >= (HBXBitTime << 3))) {
    if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
    else digitalWrite(HCL2, HIGH);	// Set clock high, and 
    return(false);                  //  error exit if no response from Motor
  }
// 5. Set clock high if data low occurred (i.e. MC acknowledged clock low)
  if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
  else digitalWrite(HCL2, HIGH);
  TimerDelayuS(HBXBitTime >> 1);
// 6. Wait for data line release (HDA1 = 1) by MC,  or timeout
  H2XStart = micros();
  do {
    H2XTimer = micros() - H2XStart;
  } while ((digitalRead(HDA1) == 0) && (H2XTimer < (HBXBitTime << 3)));
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

	unsigned char b, mask;

	HDATalk();										  // HDA as output
	axis[Motor].HBXBitCount = 8;	  // 8bits to go
	mask = 0x80;									  // MSB first
// Clock was set high before entry	
	TimerDelayuS(HIGHTIME);
 	do {
		axis[Motor].HBXBitCount -= 1;
// Set data bit
		if (databyte & mask) digitalWrite(HDA1, HIGH);
		else digitalWrite(HDA1, LOW);
    TimerDelayuS(HBXBitTime >> 1);        // Let data stabilise
		mask = mask >> 1;											// Next data bit
// Set clock low
    if (Motor == MotorAz) digitalWrite(HCL1, LOW);
    else digitalWrite(HCL2, LOW);
    TimerDelayuS(HBXBitTime);
    if (!(axis[Motor].HBXBitCount)) {			// Last bit -> force float on data
      digitalWrite(HDA1, LOW);
      HDAListen();
    }
// Set clock high
    if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
    else digitalWrite(HCL2, HIGH);
    TimerDelayuS(HBXBitTime-(HBXBitTime >> 1)); // Data is written DSTABLE before clock low
// for 8 bits
	} 	while (axis[Motor].HBXBitCount);
  TimerDelayuS(HBXBitTime >> 1);        // Last high clock
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

	unsigned long i;
	unsigned char b;
// HDA as input  
  HDAListen();                    
  axis[Motor].HBXBitCount = 8;	
	axis[Motor].HBXData = 0;
// Clock was set high before entry  
	while (axis[Motor].HBXBitCount) {
// Set clock low
		if (Motor == MotorAz) digitalWrite(HCL1, LOW);
		else digitalWrite(HCL2, LOW);
    TimerDelayuS(HBXBitTime >> 1);
// Read data bit
		axis[Motor].HBXData = axis[Motor].HBXData << 1;				// Shift previous bit
		if (digitalRead(HDA1)) axis[Motor].HBXData |=	0x01;		// Read next bit
		axis[Motor].HBXBitCount--;								            // Need eight bits
    TimerDelayuS(HBXBitTime-(HBXBitTime >> 1));            // Wait for low time
// Set clock high
    if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
    else digitalWrite(HCL2, HIGH);
    TimerDelayuS(HBXBitTime);
	}  
// Return data byte
  axis[Motor].HBXCount = 1;  
	return (axis[Motor].HBXData);
}

// HBX Get the status bytes (25 bits)
// ----------------------------------
void HBXGet3Bytes(unsigned char Motor) {
  unsigned char b;
  
	axis[Motor].HBXP1 = HBXGetByte(Motor);
  TimerDelayuS(HBXBitTime);
	axis[Motor].HBXP2 = HBXGetByte(Motor);
  TimerDelayuS(HBXBitTime);
	axis[Motor].HBXP3 = HBXGetByte(Motor);
  TimerDelayuS(HBXBitTime);
	axis[Motor].HBXP4 = 0;

// Read 'byte4' = error bit
// ------------------------
	if (Motor == MotorAz)	digitalWrite(HCL1, LOW);
	else digitalWrite(HCL2, LOW);
	TimerDelayuS(HBXBitTime >> 1);
	axis[Motor].HBXP4 |= digitalRead(HDA1);		  // Read the battery error bit
  TimerDelayuS(HBXBitTime-(HBXBitTime >> 1));
  if (Motor == MotorAz)digitalWrite(HCL1, HIGH);
  else digitalWrite(HCL2, HIGH);
	TimerDelayuS(HBXBitTime); 

  axis[Motor].HBXCount = 4;
}

// H2X Low level Functions
// -----------------------
void HDAListen() {
  pinMode(HDA1, H2X_INPUT);
//  digitalWrite(HDA1, HIGH);
}
void HDAFloat() {
  pinMode(HDA1, H2X_INPUT);
}
void HDATalk() {
  digitalWrite(HDA1, HIGH);
  pinMode(HDA1, H2X_OUTPUT);
}
void HCL1Listen() {
  pinMode(HCL1, H2X_INPUT);
}
void HCL1Talk() {
  digitalWrite(HCL1, HIGH);
  pinMode(HCL1, H2X_OUTPUT);
}
void HCL2Listen() {
  pinMode(HCL2, H2X_INPUT);
}
void HCL2Talk() {
  digitalWrite(HCL2, HIGH);
  pinMode(HCL2, H2X_OUTPUT);
}

void H2XReset() {
  HCL1Talk();
  HCL2Talk();
  HDATalk();
  digitalWrite(HDA1, LOW);
  TimerDelayuS(H2XRESETTIME);
  digitalWrite(HDA1, HIGH);
  TimerDelayuS(H2XRESETTIME);
  HDAListen();
}

long TwosComplement(long p) {					// Calculate 2s complement
  long q;
	q = ~p;										  // Bitwise invert
	q = q + 1;									// +1
	return q;
}
