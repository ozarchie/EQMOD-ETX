/*
 * Copyright 2017, 2018 John Archbold
*/

/********************************************************
  Initialize HBX, translate EQG to HBX
  ====================================
 *********************************************************/
 
#include <Arduino.h>
#include <EEPROM.h>
#include "EQGProtocol.h"
#include "ETXProtocol.h"
#include "HBXComms.h"
#include "EQG2HBX.h"						// All the declared variables

void setup()
{
	int	i, j, k;
  bool b;
  	
  DelayTimer = micros();            // Initialize timers, counts
  StatusTimer = DelayTimer;
  StatusTime  = DelayTimer;
  TestCount = 0;
  EQGErrorValue = 0;

  pinMode(FROMEQG, OUTPUT);         // Initialize Indicator pins
  pinMode(FROMHBX, OUTPUT);
  pinMode(AzLED, OUTPUT);
  pinMode(AltLED, OUTPUT);
  digitalWrite(FROMEQG, LOW);		    // Initialize Indicator LEDS
  digitalWrite(FROMHBX, LOW);   
  digitalWrite(AzLED, LOW);
  digitalWrite(AltLED, LOW);

  pinMode(MONITORHBX, INPUT_PULLUP);     // Initialize Operation jumpers
  pinMode(TESTHBX, INPUT_PULLUP);
  digitalWrite(MONITORHBX, HIGH);
  digitalWrite(TESTHBX, HIGH);   

  axis[AzMotor].PrintStatus0 = 0;         // Disable printing status polls with no change
  axis[AltMotor].PrintStatus0 = 0;        // Disable printing status polls with no change
  
  // Initialize EQG communications
  Serial.begin(115200);                     // Debug
  Serial.println("ETX V5.01");
  Serial1.begin(9600);                    // EQMOD
  Serial1.println("ETX-EQMOD");
  Serial2.begin(9600);
  Serial3.begin(9600);
    
  Serial.print("EEPROM length: ");
  Serial.println(EEPROM.length());
  Serial.print("CRC32 of EEPROM data: 0x");
  Serial.print(eeprom_crc(), HEX);
  if (!check_eeprom_crc()) {
    Serial.println(" - crc failed");
    set_eeprom_crc();
    check_eeprom_crc();
  } 
  else Serial.println(" - crc OK");


// **************************
// Check for HBX Monitor Mode
// ==========================
  if (digitalRead(MONITORHBX) == 0) { // Check if monitor jumper installed
    HBXMonitorLoop();                 // Execute monitor code until jumper removed
  }

// **************************
// Check for HBX Testing Mode
// ==========================
  if (digitalRead(TESTHBX) == 0) {  // Check if test jumper installed
    HBXTestLoop();                  // Execute test code until jumper removed
  }
  
  while (digitalRead(TESTHBX) == 0) {
    Serial.println("Test HBX commands to ETX");
    HBXTest();
    TimerDelaymS(5000);
  }
  
  Serial.println("HBX Initialization");
  AzInitialise();
  AltInitialise();
  
// Initialize HBX communications as outputs
// It will use H2X communications
  HCL1Talk();                 // Set for Talking on RAClk
  HCL2Talk();                 // Set for Talking on DECClk
  HDAListen();
  TimerDelaymS(STARTTIME);

// Reset the motors (RA and DEC)
//  and wait until both respond to a command 
  Serial.println("Wait for both motors to start up");
  WaitForMotors();  

// Get Motor Type from Az MC ( assume both same type of motor)

  Serial.print("Get Motor Type: ");
  do {
    axis[AzMotor].MotorType = 0x00; 
    if (HBXSendCommand(GetMotorType, AzMotor))
      axis[AzMotor].MotorType = HBXGetByte(AzMotor);
  } while (!axis[AzMotor].MotorType);  
  axis[AltMotor].MotorType = axis[AzMotor].MotorType;
  Serial.println(axis[AltMotor].MotorType);

  // Get the real LEDI values from EEPROM
  axis[AzMotor].HBXLEDI = EEPROM.read(EEPROMAzLEDI);
  axis[AltMotor].HBXLEDI = EEPROM.read(EEPROMAltLEDI);
  // Set LED values from EEPROM
  Serial.print("Set Encoder LED currents - Az: ");
  if (HBXSendCommand(SetLEDI, AzMotor))
    HBXSendByte(axis[AzMotor].HBXLEDI, AzMotor); 
  if (HBXSendCommand(SetLEDI, AltMotor))
    HBXSendByte(axis[AltMotor].HBXLEDI, AltMotor);    
  Serial.print(axis[AzMotor].HBXLEDI);
  Serial.print(", Alt: ");
  Serial.println(axis[AltMotor].HBXLEDI);

// Set the Offset Clear Command
//  Send HBXP1, HBXP2 - which were initialised to 0    
  Serial.println("Reset any ETX offset bytes");
  if (HBXSendCommand(SetOffset, AzMotor))
    HBXSend2Bytes(AzMotor);
  TimerDelaymS(CMNDTIME);
  if (HBXSendCommand(SetOffset, AltMotor))
    HBXSend2Bytes(AltMotor);
  TimerDelaymS(CMNDTIME);

  // Stop the motors (RA and DEC) 
  Serial.println("Stop motors");
  do {
    P1 = 0;
    if (HBXSendCommand(Stop, AzMotor)) P1 += 1;
    if (HBXSendCommand(Stop, AltMotor)) P1 += 1;
  } while (P1 < 2);
  
// Read status
  Serial.println("Read Status");
  HBXGet2Status();          // Check and read both motor states

  currentTime = now();
  Serial.println("Setup Complete. Listening for commands ..");
// Print debug info every 5 s
// --------------------------
  Alarm.timerRepeat(10, debugEQG);    // Every 10 seconds
}

void loop()
{

/**************************************************************************************************
 *   Check ETXState
 **************************************************************************************************/
 
  if ((micros() - DelayTimer) > 6550) {
    DelayTimer = micros();
    HBXGet2Status();              // Polls motor position every 6.55mS
  }
  
  ETXState(AzMotor);            // Check the Az motor state
  ETXState(AltMotor);           // Check the Alt motor state

/**************************************************************************************************
 *		Process EQG comms - Rx Comms are interrupt driven
 **************************************************************************************************/

  EQGRx();                            // Check for comms from EQG
  EQGState();                         // Check command state
	if (EQGDone) {								      // EQG receive complete, see what it is
		if (EQGErrorValue == 0) {  
  		EQGAction(); //  and do it
    }
    else { 
      EQGError(EQGErrorValue);        // EQGAction() may set an error
	  }
	}
	while (EQGTxoPtr != EQGTxiPtr) {		        // EQG send any response
    Serial1.write(EQGTxBuffer[EQGTxoPtr++]);  // Output to EQG
		EQGTxoPtr &= EQGMASK;
	}
  Alarm.delay(0);
}		// End loop()


/**************************************************************************************************
 *    Read / Update ETX - Timer Driven
 **************************************************************************************************/
void UpdateETX(void) {


} 

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

/**********************************************
  EEPROM support
***********************************************/

unsigned long eeprom_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < (EEPROM.length()-4)  ; ++index) {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

unsigned long get_eeprom_crc(void) {
  unsigned long crc;
  int i;
  i = EEPROM.length()-4;      // Location of stored crc (last four bytes)
  crc = 0;
  for (int j = 0; j < 4; j++) {
    crc = crc << 8;
    crc = crc | EEPROM.read(i);
    i += 1;
  }
  return (crc);
}

bool set_eeprom_crc(void) {
  unsigned long crc;
  unsigned char data;
  int i;
  i = EEPROM.length()-1;      // Location of stored crc (last four bytes)
  crc = eeprom_crc();
  for (int j = 0; j < 4; j++) {
    data = crc & 0xFF;
    EEPROM.write(i, data);
    if (EEPROM.read(i) != data) return (false);
    i -= 1;
    crc = crc >> 8;
  }
  return (true);
}

bool check_eeprom_crc(void) {
  if (eeprom_crc() == get_eeprom_crc()) return true;
  else return false;
}


