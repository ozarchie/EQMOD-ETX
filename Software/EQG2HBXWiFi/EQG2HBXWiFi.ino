#include <dummy.h>

/*
    Name:       EQG2HBXE32.ino
    Created:	2018-09-01 10:07:17 AM
    Author:     JOHNWIN10PRO\John
*/

/********************************************************
  Initialize HBX, translate EQG to HBX
  ====================================
 *********************************************************/
#define mESP32
#define mTEST

#include "HBXFileSystem.h"
#include "HBXWiFiServer.h"
#include "EQGProtocol.h"
#include "ETXProtocol.h"
#include "HBXComms.h"
#include "EQG2HBX.h"        // All the declared variables

// Function Prototypes
//
void UpdateETX(void);
void CheckETXState(void);
void TimerDelaymS(unsigned long );
void TimerDelayuS(unsigned int);


// Functions
//
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

void CheckETXStatus(unsigned char Motor) {
	/**************************************************************************************************
 *   Get ETXStatus
 **************************************************************************************************/
	HBXGetStatus(Motor);
}

void CheckETXState(unsigned char Motor) {
	/**************************************************************************************************
 *   Check ETXState
 **************************************************************************************************/
	ETXState(Motor);									// Check the motor state
}

void HandleOTA()
{
 StartOTAIfRequired();
 ArduinoOTA.handle();
}

void StartOTAIfRequired()
{
 if (ota_started)
 return;
 // Port defaults to 8266
 // ArduinoOTA.setPort(8266);
 // Hostname defaults to esp8266-[ChipID]
 //if (ArduinoOTA.getHostname() && ArduinoOTA.getHostname().length())
 
 // No authentication by default
 ArduinoOTA.setPassword((const char *)"EQG2HBX");
 ArduinoOTA.onStart([]() {
 Serial.println("OTA Start");
 });
 ArduinoOTA.onEnd([]() {
 Serial.println("\nOTA End");
 });
 ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
 Serial.printf("Progress: %u%%\r\n", (progress / (total / 100)));
 });
 ArduinoOTA.onError([](ota_error_t error) {
 Serial.printf("Error[%u]: ", error);
 if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
 else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
 else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
 else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
 else if (error == OTA_END_ERROR) Serial.println("End Failed");
 });
 ArduinoOTA.begin();
 ota_started = true;
 delay(500);
}

// =======================================================================================================
void setup()
{

#ifdef mESP32
	dbgSerial.begin(115200);											// debug
	EQGSerial.begin(9600, SERIAL_8N1, 18, 19);    // EQG via serial or WiFi
	delay(10);
#endif

	dbgSerial.println(EQ2HBX_Version);
	DelayTimer = micros();							// Initialize timers, counts
	StatusTimer = DelayTimer;
	StatusTime = DelayTimer;
	EQGErrorValue = 0;

#ifdef mESP32
	HBXWiFiSetup();
#endif

	pinMode(SCOPELED, OUTPUT);
	pinMode(EQGLED, OUTPUT);
	digitalWrite(SCOPELED, LOW);
	digitalWrite(EQGLED, LOW);
	pinMode(TESTHBX, INPUT_PULLUP);     // Initialize Mode jumpers
	digitalWrite(TESTHBX, HIGH);				// Use internal PUR, write 1 to O/P

	axis[AzMotor].PrintStatus0 = 0;         // Disable printing "status polls" with no change
	axis[AltMotor].PrintStatus0 = 0;        // Disable printing "status polls" with no change

	// Initialize EQG communications

	// **************************
	// Check for HBX Testing Mode
	// ==========================
	dbgSerial.print("digitalRead(TESTHBX)  : ");
	dbgSerial.println(digitalRead(TESTHBX));
	while (digitalRead(TESTHBX) == 0) {   // Check if test jumper installed
		dbgSerial.print("digitalRead(TESTHBX)  : ");
		dbgSerial.println(digitalRead(TESTHBX));
		HBXTestLoop();                      // Execute test code until jumper removed
	};

	dbgSerial.println("HBX Initialization");

	// Read Motor Type to determine telescope type
	// -------------------------------------------
	preferences.begin("EQG2HBX", false);								// Access EQG2HBX namespace
	telescope = 0;																			// Default ETX60
	if (!(preferences.getUChar("TELESCOPE", 0) == 0))	{	// If it exists check telescope table for a match
		telescope = (preferences.getUChar("TELESCOPE", 0));
	}
	dbgSerial.print("Telescope: ");
	dbgSerial.print(telescope);
	dbgSerial.print(", ");
	dbgSerial.println(ratio[telescope][0].Telescope);
	if (!(preferences.getUChar("PROTOCOL", 0) == 0)) {	// If it exists get protocol type (UDP, NOW)
		protocol = (preferences.getUChar("PROTOCOL", 0));
	}
	dbgSerial.print("Protocol: ");
	dbgSerial.print(protocol);
	dbgSerial.print(", ");
	if (!(preferences.getUChar("STATION", 0) == 0)) {	// If it exists get station type (AP, STA)
		protocol = (preferences.getUChar("STATION", 0));
	}
	dbgSerial.print("Station: ");
	dbgSerial.print(station);
	dbgSerial.print(", ");
	preferences.end();

	AzInitialise(telescope);
	AltInitialise(telescope);
	PrintRatioValues(telescope);
	PrintHbxValues(AzMotor);
	PrintHbxValues(AltMotor);

	// Initialize HBX communications as outputs
	// It will use H2X communications
	HCL1Talk();                 // Set for Talking on RAClk
	HCL2Talk();                 // Set for Talking on DECClk
	HDAListen();

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
  HandleOTA();
	dbgRx();                            // Check for comms from debug port for telescope values

	// Check ETX motor status and state
	if ((micros() - StateTimer) > (STATEDELAY * 1000)) {   // ~6.55mS
		if (StateSelect) StateSelect = false;
		else StateSelect = true;
		StateTimer = micros();
		if (StateSelect) CheckETXState(AzMotor);
		else  CheckETXState(AltMotor);
	}
	if ((micros() - StatusTimer) > (STATUSDELAY * 1000)) {   // ~50mS
		if (StatusSelect) StatusSelect = false;
		else StatusSelect = true;
		StatusTimer = micros();
		if (StatusSelect) CheckETXStatus(AzMotor);
		else  CheckETXStatus(AltMotor);
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
}

// End loop()
