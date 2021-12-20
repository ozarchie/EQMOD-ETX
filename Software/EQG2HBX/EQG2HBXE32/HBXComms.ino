/*
 * Copyright 2017, 2018 John Archbold
*/
#include <Arduino.h>

/********************************************************
  HBX Comms related functions
  ===========================
 *********************************************************/
 
// HBX transmission functions
// ==========================

// HBX Send a command
// ------------------
bool HBXSendCommand(unsigned char Command, unsigned char Motor) {

  axis[Motor].Command = Command;
  
// Send the start sequence
// -----------------------
  if (HBXStartSequence(Motor)) {
 
// Send the command byte
// ---------------------
  	HBXSendByte(Command, Motor);
  	return(true);
  }
  else return(false);
}

// HBX Initiate start sequence
// ---------------------------
bool HBXStartSequence(unsigned char Motor) {
 
  HDAListen();                    // HDA as input
  
// Set clock low
  if (Motor == MotorAz) digitalWrite(HCL1, LOW);
  else digitalWrite(HCL2, LOW);
  TimerDelayuS(HBXBitTime >> 1);          // Wait for answer

// Wait for data low by MC, or timeout
  H2XStart = micros();              // Get the start microseconds
  do {                              // Wait for MC to answer with HDA1 = 0
    H2XTimer = micros() - H2XStart;
  } while ((digitalRead(HDA1) == 1) && (H2XTimer < (HBXBitTime << 3)));
  TimerDelayuS((HBXBitTime >> 5));            // Just in case of data line glitch

// Re-read data line, check if (data low transition) or (MC timeout)
  if ((digitalRead(HDA1) == 1) || (H2XTimer >= (HBXBitTime << 3))) {
    if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
    else digitalWrite(HCL2, HIGH);    
    return(false);                  // Error Exit if no response from Motor
  }

// Set clock high if data low transition (i.e. MC acknowledged clock)
  if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
  else digitalWrite(HCL2, HIGH);
  TimerDelayuS(HBXBitTime >> 1);
  
// Wait for data line release by MC,  or timeout
  H2XStart = micros();              // Get the start microseconds
  do {                              // Wait for MC to answer
    H2XTimer = micros() - H2XStart;
  } while ((digitalRead(HDA1) == 0) && (H2XTimer < (HBXBitTime << 3)));
  TimerDelayuS(HBXBitTime);

// Check timeout for data line released
  if (H2XTimer >= (HBXBitTime << 3)) {
    return(false);                  // Error Exit if no response from Motor
  }
  return(true);                     // Success                     
}

// HBX Send a single byte
// ----------------------
void HBXSendByte(unsigned char databyte, unsigned char Motor) {

	unsigned char mask;

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
		mask = mask >> 1;             // Next data bit

// Set clock low
    if (Motor == MotorAz) digitalWrite(HCL1, LOW);
    else digitalWrite(HCL2, LOW);
    TimerDelayuS(HBXBitTime);
    
    if (!(axis[Motor].HBXBitCount)) { // Last bit -> force float on data
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

  HDAListen();                    // HDA as input
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
	axis[Motor].HBXP4 |= digitalRead(HDA1);		  // Read the error bit
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
