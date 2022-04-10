/**@file*/
/*
    Name:       EQG2HBXE32.ino
    Created:		2018-09-01 10:07:17 AM
    Author:     JOHNWIN10PRO\John
*/
#include <Arduino.h>
#include <stdio.h>
#include <dummy.h>
#include "Hardware.h"

#include <FS.h>   // Include the SPIFFS library
#include <Preferences.h>
#include "ESP32Ticker.h"
//#include <BluetoothSerial.h>

#include "EQG2HBX.h"
#include "EQGProtocol.h"
#include "ETXProtocol.h"
#include "HBXComms.h"
#include "HBXWiFiServer.h"
#include "HBXFileSystem.h"


// Functions
//
/**************************************************************************************************
 *    Read / Update ETX - Timer Driven
 **************************************************************************************************/
void UpdateETX(void) {
}

/**************************************************************************************************
 *   Get ETXStatus
 **************************************************************************************************/
void CheckETXStatus(unsigned char Motor) {
	HBXGetStatus(Motor);
}

/**************************************************************************************************
 *   Check ETXState
 **************************************************************************************************/
void CheckETXState(unsigned char Motor) {
	ETXState(Motor);									// Check the motor state
}

/********************************************************
	Initialize HBX, translate EQG to HBX
	====================================
 *********************************************************/

// =======================================================================================================
void setup()
{
//	int	i, j, k;
//	bool b;

#ifdef mESP32
	dbgSerial.begin(115200);											// debug
	EQGSerial.begin(9600, SERIAL_8N1, 18, 19);    // EQG via serial, bluetooth or WiFi
	#ifdef BTSerial
		EQGBluetooth.begin("EQ6Blue");
	#endif
	delay(10);
#endif

	dbgSerial.println("");
	dbgSerial.println(EQ2HBX_Version);
	DelayTime = micros();										// Initialize timers, counts
	StatusTime = DelayTime;
	EQGErrorValue = 0;

#ifdef mESP32
	HBXWiFiSetup();
#endif
	preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace
	dbgSerial.print("SPIFFS - ssid: "); dbgSerial.print(preferences.getString("STA_SSID"));
	dbgSerial.print(", pass: "); dbgSerial.print(preferences.getString("STA_PASS"));
	dbgSerial.print(", Telescope: "); dbgSerial.print(preferences.getString("TELESCOPE"));
	dbgSerial.print(", Mount: "); dbgSerial.println(preferences.getString("MOUNT"));
	preferences.end();

	pinMode(ETXLED, OUTPUT);								// Operation indicators
	pinMode(EQGLED, OUTPUT);
	digitalWrite(ETXLED, LOW);
	digitalWrite(EQGLED, LOW);

	axis[AzMotor].PrintStatus0 = 0;         // Disable printing "status polls" with no change
	axis[AltMotor].PrintStatus0 = 0;        // Disable printing "status polls" with no change

	// Initialize EQG communications
	dbgSerial.println("HBX Initialization");

	// Read Motor Type to determine telescope type
	// -------------------------------------------
	preferences.begin("EQG2HBX", false);								// Access EQG2HBX namespace
	scopetype = 0;														// Default ETX60
	if (!(preferences.getUChar("SCOPETYPE", 0xFF) == 0xFF))	{			// If it exists check telescope table for a match
		scopetype = (preferences.getUChar("SCOPETYPE", 0));
	}
	
	scope = preferences.getString("TELESCOPE", "none");
	dbgSerial.print("Telescope: ");
	dbgSerial.print(scopetype);
	dbgSerial.print(", ");
	dbgSerial.print(scope);
	dbgSerial.print(", ");
	dbgSerial.println(ratio[scopetype][0].Telescope);

	mount = preferences.getString("MOUNT", "none");
	if (!(preferences.getUChar("MOUNTTYPE", 0xFF) == 0xFF)) {		// If it exists get mount type (ALTAZ, GEM, FORK)
		mounttype = (preferences.getUChar("MOUNTTYPE", 0));
	}
	dbgSerial.print("Mount:     ");
	dbgSerial.print(mounttype);
	dbgSerial.print(", ");
	dbgSerial.print(mount);
	dbgSerial.print(", ");
	dbgSerial.println(mountDesc[mounttype]);

	mode = preferences.getString("WIFIMODE", "none");
	if (!(preferences.getUChar("MODETYPE", 0xFF) == 0xFF)) {		// If it exists get station type (AP, STA)
		modetype = (preferences.getUChar("MODETYPE", 0));
	}
	dbgSerial.print("WiFiMode:  ");
	dbgSerial.print(modetype);
	dbgSerial.print(", ");
	dbgSerial.print(mode);
	dbgSerial.print(", ");
	dbgSerial.println(modeDesc[modetype]);
	
	protocol = preferences.getString("PROTOCOL", "none");
	if (!(preferences.getUChar("PROTOCOLTYPE", 0xFF) == 0xFF)) {	// If it exists get protocol type (UDP, NOW)
		protocoltype = (preferences.getUChar("PROTOCOLTYPE", 0));
	}
	dbgSerial.print("Protocol:  ");
	dbgSerial.print(protocoltype);
	dbgSerial.print(", ");
	dbgSerial.print(protocol);
	dbgSerial.print(", ");
	dbgSerial.println(protocolDesc[protocoltype]);

	preferences.end();

	AzInitialise(scopetype);
	AltInitialise(scopetype);
	//PrintRatioValues(telescope);
	PrintHbxValues(AzMotor);
	PrintHbxValues(AltMotor);

	// Initialize HBX communications as outputs
	// It will use H2X communications
	HBXReset();

	// Reset the motors (RA and DEC)
	//  and wait until both respond to a command 
	dbgSerial.println("Waiting for both motors to start up ..");
	WaitForMotors();

	// Get Motor Type from Az MC ( assume both same type of motor)

	do {
		axis[AzMotor].MotorType = 0x00;
		if (HBXSendCommand(GetMotorType, AzMotor))
			axis[AzMotor].MotorType = HBXGetByte(AzMotor);
	} while (!axis[AzMotor].MotorType);
	axis[AltMotor].MotorType = axis[AzMotor].MotorType;
	dbgSerial.println(""); dbgSerial.print("Motor Type: "); dbgSerial.print(axis[AltMotor].MotorType);

	// Handle position sensors LED current
	// -----------------------------------
	dbgSerial.println(""); dbgSerial.print("Check Calibrate LEDs");
	preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace
	if (preferences.getUChar("AzLEDI", 0) == 0) {		// If it does not exist, calibrate the LEDs
		CalibrateLEDs();
	}
// Read stored LED currents
	axis[AzMotor].HBXLEDI = preferences.getUChar("AzLEDI", 0);
	axis[AltMotor].HBXLEDI = preferences.getUChar("AltLEDI", 0);
	preferences.end();

// Set the MC LED values
	dbgSerial.println(""); dbgSerial.print("Set MC LED values");
	if (HBXSendCommand(SetLEDI, AzMotor))
		HBXSendByte(axis[AzMotor].HBXLEDI, AzMotor);
	axis[AzMotor].HBXP1 = axis[AzMotor].HBXLEDI;
	HBXPrintStatus(AzMotor);

	if (HBXSendCommand(SetLEDI, AltMotor))
		HBXSendByte(axis[AltMotor].HBXLEDI, AltMotor);
	axis[AltMotor].HBXP1 = axis[AltMotor].HBXLEDI;
	HBXPrintStatus(AltMotor);

	// Set the Offset Clear Command
	//  Send HBXP1, HBXP2    
	dbgSerial.println(""); dbgSerial.print("Reset any ETX offset bytes");
	axis[AzMotor].HBXP1 = 0;
	axis[AzMotor].HBXP2 = 0;
	axis[AltMotor].HBXP1 = 0;
	axis[AltMotor].HBXP2 = 0;
	if (HBXSendCommand(SetOffset, AzMotor))
		HBXSend2Bytes(AzMotor);
	TimerDelaymS(CMNDTIME);
	if (HBXSendCommand(SetOffset, AltMotor))
		HBXSend2Bytes(AltMotor);
	TimerDelaymS(CMNDTIME);

	// Stop the motors (RA and DEC) 
	dbgSerial.println(""); dbgSerial.print("Stop motors");
	do {
		P1 = 0;
		if (HBXSendCommand(Stop, AzMotor)) P1 += 1;
		if (HBXSendCommand(Stop, AltMotor)) P1 += 1;
	} while (P1 < 2);

	// Read status
	dbgSerial.println(""); dbgSerial.println("Read Status");
	HBXGet2Status();          // Check and read both motor states

	dbgSerial.println("Setup Complete. Listening for commands ..");

	// Print debug info every 10 s
	// ---------------------------

	// HBXTestLoop();

//	AlarmDebugPrint.attach(10, debugEQG);				// Print debug info every 10 s
//	AlarmCheckETX.attach_ms(6, CheckETXState);	// Check motor status (position), every 6mS
}


// =======================================================================================================
void loop()
{
	/**************************************************************************************************
	 *		Process EQG comms - Rx, Tx Comms are interrupt driven
	 **************************************************************************************************/

	// dbgRx();                            // Check for comms from debug port for telescope values

	// Check ETX motor status and state
	if ((micros() - StateTimer) > (STATEDELAY * 1000)) {   // ~6.55mS
		if (StateSelect) StateSelect = false;
		else StateSelect = true;
		StateTimer = micros();
		if (StateSelect) CheckETXState(AzMotor);
		else  CheckETXState(AltMotor);
	}
	
//jma	CheckAltFlipReqd();
	
	if ((micros() - StatusTimer) > (STATUSDELAY * 1000)) {   // ~50mS
		StatusTimer = micros();
/*		if (StatusSelect) StatusSelect = false;
		else StatusSelect = true;
		if (StatusSelect) CheckETXStatus(AzMotor);
		else  CheckETXStatus(AltMotor);
*/
		CheckETXStatus(AzMotor);
		CheckETXStatus(AltMotor);
	}

	// Check any incoming characters from the EQMOD serial interface
	if (HBXCheckRx())
		EQGState();                       // Check command state if data received
	if (EQGDone) {								      // EQG Rx complete, see what it is
		if (EQGErrorValue == 0) {
			EQGAction();										//  and do it
		}
		else {
			EQGError(EQGErrorValue);        // EQGAction() may set an error
		}
	}

	// Check for any characters that are ready to go to the WiFi interface
	while (EQGTxoPtr != EQGTxiPtr) {	

		// Send any characters that are ready to go to the WiFi interface
		digitalWrite(EQGLED, HIGH);
		HBXCheckTx();
		EQGTxoPtr &= EQGMASK;
		digitalWrite(EQGLED, LOW);
	}
//	TimerDelaymS(1);
//	yield();
 // HandleOTA();
}

// End loop()

