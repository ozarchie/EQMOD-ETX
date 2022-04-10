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
  HBX Comms related functions
  ===========================
 *********************************************************/
/**********************************************
	Multiple 1mS delay
***********************************************/

void TimerDelaymS(unsigned long d) {
	delay(d);
}

/**********************************************
  Multiple 1uS delay
***********************************************/

void TimerDelayuS(unsigned int d) {
	delayMicroseconds(d);
}

 // HBX Attempt to reset
 // --------------------
void HBXMotorReset(unsigned char Motor)
{
	if (Motor == AzMotor) {HCL = HCLAz; HDA = HDAAz;}
	else {HCL = HCLAlt; HDA = HDAAlt;}
	/*
	int i;

	// Write LOW
	HDATalk(Motor);
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
	HDAListen(HDA);
	for (i = 0; i < 8; i++) {
		if (Motor == MotorAz) digitalWrite(HCL1, LOW);
		else digitalWrite(HCL2, LOW);
		TimerDelayuS(HBXBitTime);
		if (Motor == MotorAz) digitalWrite(HCL1, HIGH);
		else digitalWrite(HCL2, HIGH);
		TimerDelayuS(HBXBitTime);
	}
	*/

	// Force Clock High, Low, High for reset, time ~1.25s
	HCLTalk(Motor);
	digitalWrite(HCL, HIGH);
	TimerDelaymS(MOTORDETECT);
	digitalWrite(HCL, LOW);
	TimerDelaymS(MOTORDETECT);
	digitalWrite(HCL, HIGH);
	TimerDelaymS(MOTORDETECT >> 1);
}

// HBX transmission functions
// ==========================

// HBX Send a command
// ------------------
bool HBXSendCommand(unsigned char Command, unsigned char Motor) {

	pCommand = Command;
	if (Command != GetStatus){
		dbgSerial.println("");dbgSerial.print("+++ ");dbgSerial.print(Motor);
	}
  axis[Motor].Command = Command;

	// Select the interface
	if (Motor == MotorAz) {HDA = HDAAz;HCL = HCLAz;}
	else {HDA = HDAAlt;HCL = HCLAlt;}

// Send the start sequence
// -----------------------
  if (HBXStartSequence(Motor)) { 
// Send the command byte
// ---------------------
  	HBXSendByte(Command, Motor);
  	return(true);
  }
	else {
		HDAListen(HDA);					// Set data inbound
		return(false);
	}
}

// HBX Initiate start sequence
// ---------------------------
bool HBXStartSequence(unsigned char Motor) {
	if (Motor == AzMotor) {HCL = HCLAz; HDA = HDAAz;}
	else if (Motor == AltMotor) {HCL = HCLAlt; HDA = HDAAlt;}
	else  {HCL = HCLAux; HDA = HDAAux;}
	
// 1. HDA as input, Clock as output
	HDAListen(HDA); 
	HCLTalk(HCL);
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
	if (Motor == AzMotor) {HCL = HCLAz; HDA = HDAAz;}
	else if (Motor == AltMotor) {HCL = HCLAlt; HDA = HDAAlt;}
	else  {HCL = HCLAux; HDA = HDAAux;}

	if (axis[Motor].Command != GetStatus) {
		dbgSerial.print("-> "); dbgSerial.print(databyte, HEX);
	}

	HDATalk(HDA);									// HDA as output
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
	HDAListen(HDA);											// Release data pin
	TimerDelayuS(HBXBitTime);
}

// HBX Send two bytes in sequence
// ------------------------------
void HBXSend2Bytes(unsigned char Motor) {
  HBXSendByte(axis[Motor].HBXP1, Motor);
  HBXSendByte(axis[Motor].HBXP2, Motor);
	if (pCommand != GetStatus) dbgSerial.println("");
}

// HBX Send three bytes in sequence
// --------------------------------
void HBXSend3Bytes(unsigned char Motor) {
  HBXSendByte(axis[Motor].HBXP1, Motor);
  HBXSendByte(axis[Motor].HBXP2, Motor);
  HBXSendByte(axis[Motor].HBXP3, Motor);
	if (pCommand != GetStatus) dbgSerial.println("");
}

// HBX Get a single  byte
// ----------------------
unsigned char HBXGetByte(unsigned char Motor) {

	if (Motor == AzMotor) {HCL = HCLAz; HDA = HDAAz;}
	else if (Motor == AltMotor) {HCL = HCLAlt; HDA = HDAAlt;}
	else  {HCL = HCLAux; HDA = HDAAux;}

// HDA as input  
  HDAListen(HDA);                    
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
	if (pCommand != GetStatus) dbgSerial.println("");
}

// H2X Low level Functions
// -----------------------
void HDAListen(uint8_t HDA) {
//	digitalWrite(HDA, HIGH);
	pinMode(HDA, H2C_INPUT);
}
void HDAFloat(uint8_t HDA) {
  pinMode(HDA, H2C_INPUT);
}
void HDATalk(uint8_t HDA) {
  digitalWrite(HDA, HIGH);
  pinMode(HDA, H2C_OUTPUT);
}
void HCLListen(uint8_t HCL) {
//	digitalWrite(HCL, HIGH);
	pinMode(HCL, H2C_INPUTPU);
}
void HCLFloat(uint8_t HCL) {
	pinMode(HCL, H2C_INPUT);
}
void HCLTalk(uint8_t HCL) {
	digitalWrite(HCL, HIGH);
	pinMode(HCL, H2C_OUTPUT);
}

bool HBXReset(void) {
	int ClockCount = 0;

	HCLTalk(HCLAz);			// Set clock high
	HCLTalk(HCLAlt);		// Set clock high
	HCLTalk(HCLAux);		// Set clock high
	HDAListen(HDAAz);		// Set common data inbound
	TimerDelayuS(HBXBitTime);

	// Data should be high
	while ((!digitalRead(HDA)) && (ClockCount < 25)) {
		digitalWrite(HCLAux, LOW);
		digitalWrite(HCLAz, LOW);
		digitalWrite(HCLAlt, LOW);
		TimerDelaymS(HCLRESETTIME);
		digitalWrite(HCLAux, HIGH);
		digitalWrite(HCLAz, HIGH);
		digitalWrite(HCLAlt, HIGH);
		TimerDelayuS(HBXBitTime);
		ClockCount += 1;
	}
	if (ClockCount >= 25) return(false);
	else return(true);
}

long TwosComplement(long p) {					// Calculate 2s complement
  long q;
	q = ~p;										  // Bitwise invert
	q = q + 1;									// +1
	return q;
}
