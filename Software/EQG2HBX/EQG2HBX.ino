/*
 * Copyright 2017, 2018 John Archbold
*/

#include <Arduino.h>

/********************************************************
  Initialize HBX, translate EQG to HBX
  ====================================
 *********************************************************/
#define m2560
#undef mDue
#undef mESP32

#ifdef m2560
#include <EEPROM.h>
#endif

#ifdef mESP32
#include "HardwareSerial.h"
HardwareSerial Serial1(2);
#endif

#ifdef m2560
#define dbgSerial Serial
#define EQGSerial Serial1
#endif
#ifdef mDue
#define dbgSerial Serial
#define EQGSerial Serial1
#endif
#ifdef mESP32
#define dbgSerial Serial
#define EQGSerial Serial1
#endif

#define EEPROMLENGTH 256

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

#ifdef m2560
  pinMode(MONITORHBX, INPUT_PULLUP);     // Initialize Operation jumpers
  pinMode(TESTHBX, INPUT_PULLUP);
  digitalWrite(MONITORHBX, HIGH);
  digitalWrite(TESTHBX, HIGH);   
#endif

#ifdef mDue
  pinMode(MONITORHBX, INPUT_PULLUP);     // Initialize Operation jumpers
  pinMode(TESTHBX, INPUT_PULLUP);
  digitalWrite(MONITORHBX, HIGH);
  digitalWrite(TESTHBX, HIGH);   
#endif

#ifdef m2560
  dbgSerial.begin(115200);                // debug
  EQGSerial.begin(9600);                  // EQG
#endif

#ifdef mDue
  dbgSerial.begin(115200);                // debug
  EQGSerial.begin(9600);                  // EQG
#endif

#ifdef mESP32
  dbgSerial.begin(115200);                // debug
  EQGSerial.begin(9600);                  // EQG
#endif

  dbgSerial.println("ETX V5.01");
  EQGSerial.println("ETX-EQMOD V1.01");
  digitalWrite(FROMEQG, HIGH);        // Initialize Indicator LEDS
  delay(1000);
  digitalWrite(FROMEQG, LOW);        // Initialize Indicator LEDS
 
  axis[AzMotor].PrintStatus0 = 0;         // Disable printing status polls with no change
  axis[AltMotor].PrintStatus0 = 0;        // Disable printing status polls with no change

//  Serial2.begin(9600);
//  Serial3.begin(9600);

#ifndef mDue
  dbgSerial.print("EEPROM length: ");
  dbgSerial.println(EEPROMLENGTH);
  dbgSerial.print("CRC32 of EEPROM data: 0x");
  dbgSerial.print(eeprom_crc(), HEX);
  if (!check_eeprom_crc()) {
    dbgSerial.println(" - crc failed");
    set_eeprom_crc();
    check_eeprom_crc();
  } 
  else dbgSerial.println(" - crc OK");
#endif


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
    dbgSerial.println("Test HBX commands to ETX");
    HBXTest();
    TimerDelaymS(5000);
  }
  
  dbgSerial.println("HBX Initialization");
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
  dbgSerial.println("Wait for both motors to start up");
  WaitForMotors();  

// Get Motor Type from Az MC ( assume both same type of motor)

  dbgSerial.print("Get Motor Type: ");
  do {
    axis[AzMotor].MotorType = 0x00; 
    if (HBXSendCommand(GetMotorType, AzMotor))
      axis[AzMotor].MotorType = HBXGetByte(AzMotor);
  } while (!axis[AzMotor].MotorType);  
  axis[AltMotor].MotorType = axis[AzMotor].MotorType;
  dbgSerial.println(axis[AltMotor].MotorType);

#ifndef mDue
  // Get the real LEDI values from EEPROM
  axis[AzMotor].HBXLEDI = EEPROM.read(EEPROMAzLEDI);
  axis[AltMotor].HBXLEDI = EEPROM.read(EEPROMAltLEDI);
#endif
#ifdef mDue
  axis[AzMotor].HBXLEDI = 0x72;
  axis[AltMotor].HBXLEDI = 0x72;
#endif

  // Set LED values from EEPROM
  dbgSerial.print("Set Encoder LED currents - Az: ");
  if (HBXSendCommand(SetLEDI, AzMotor))
    HBXSendByte(axis[AzMotor].HBXLEDI, AzMotor); 
  if (HBXSendCommand(SetLEDI, AltMotor))
    HBXSendByte(axis[AltMotor].HBXLEDI, AltMotor);    
  dbgSerial.print(axis[AzMotor].HBXLEDI);
  dbgSerial.print(", Alt: ");
  dbgSerial.println(axis[AltMotor].HBXLEDI);

// Set the Offset Clear Command
//  Send HBXP1, HBXP2 - which were initialised to 0    
  dbgSerial.println("Reset any ETX offset bytes");
  if (HBXSendCommand(SetOffset, AzMotor))
    HBXSend2Bytes(AzMotor);
  TimerDelaymS(CMNDTIME);
  if (HBXSendCommand(SetOffset, AltMotor))
    HBXSend2Bytes(AltMotor);
  TimerDelaymS(CMNDTIME);

  // Stop the motors (RA and DEC) 
  dbgSerial.println("Stop motors");
  do {
    P1 = 0;
    if (HBXSendCommand(Stop, AzMotor)) P1 += 1;
    if (HBXSendCommand(Stop, AltMotor)) P1 += 1;
  } while (P1 < 2);
  
// Read status
  dbgSerial.println("Read Status");
  HBXGet2Status();          // Check and read both motor states

  currentTime = now();
  dbgSerial.println("Setup Complete. Listening for commands ..");
// Print debug info every 5 s
// --------------------------
  Alarm.timerRepeat(10, debugEQG);    // Every 10 seconds
}

void loop()
{
  Serial.println("ETX V5.01");
  Serial1.println("ETX-EQMOD");
}
/**************************************************************************************************
 *   Check ETXState
 **************************************************************************************************/
void temp1(void) { 
//  if ((micros() - DelayTimer) > (6550)) {
//    DelayTimer = micros();
//    HBXGet2Status();              // Polls motor position every 6.55mS
    HBXGetStatus(AzMotor);
    HBXGetStatus(AltMotor);
    delay(ETXDELAY * 4);
//  }
  
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
    EQGSerial.write(EQGTxBuffer[EQGTxoPtr++]);  // Output to EQG
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
#ifndef mDue
unsigned long eeprom_crc(void) {

  const unsigned long crc_table[16] = {
    0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
    0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
    0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
    0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
  };

  unsigned long crc = ~0L;

  for (int index = 0 ; index < (EEPROMLENGTH - 4)  ; ++index) {
    crc = crc_table[(crc ^ EEPROM.read(index)) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM.read(index) >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }
  return crc;
}

unsigned long get_eeprom_crc(void) {
  unsigned long crc;
  int i;
  i = EEPROMLENGTH - 4;      // Location of stored crc (last four bytes)
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
  i = EEPROMLENGTH - 1;      // Location of stored crc (last four bytes)
  crc = eeprom_crc();
  for (int j = 0; j < 4; j++) {
    data = crc & 0xFF;
    EEPROM.write(i, data);
//    if (EEPROM.read(i) != data) return (false);
    i -= 1;
    crc = crc >> 8;
  }
  return (true);
}

bool check_eeprom_crc(void) {
  if (eeprom_crc() == get_eeprom_crc()) return true;
  else return false;
}
#endif


