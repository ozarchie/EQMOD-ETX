// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       EQG2HBXE32.ino
    Created:	2018-09-01 10:07:17 AM
    Author:     JOHNWIN10PRO\John
*/

/********************************************************
  Initialize HBX, translate EQG to HBX
  ====================================
 *********************************************************/
#undef m2560




#include <Preferences.h>
#include <dummy.h>
#define mESP32
#define mTEST

#include "HBXWiFiServer.h"
#include "EQGProtocol.h"
#include "ETXProtocol.h"
#include "HBXComms.h"
#include "EQG2HBX.h"						        // All the declared variables

 
 // Define User Types below here or use a .h file
//


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

void CheckETXState(void) {
	/**************************************************************************************************
 *   Check ETXState
 **************************************************************************************************/
	HBXGetStatus(AzMotor);
	HBXGetStatus(AltMotor);

	ETXState(AzMotor);									// Check the Az motor state
	ETXState(AltMotor);									// Check the Alt motor state
}

// =======================================================================================================
void setup()
{
	int	i, j, k;
	bool b;

#ifdef mESP32
	dbgSerial.begin(115200);											// debug
	EQGSerial.begin(9600, SERIAL_8N1, 18, 19);    // EQG via serial or WiFi
	delay(10);
#endif

	dbgSerial.println("ETX V2.04, ETX-EQMOD V1.03");
	DelayTimer = micros();							// Initialize timers, counts
	StatusTimer = DelayTimer;
	StatusTime = DelayTimer;
	EQGErrorValue = 0;

#ifdef mESP32
	HBXWiFiSetup();
#endif

	pinMode(AzLED, OUTPUT);
	pinMode(AltLED, OUTPUT);
	digitalWrite(AzLED, LOW);
	digitalWrite(AltLED, LOW);
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
	AzInitialise();
	AltInitialise();
	PrintRatioValues(telescope);
	PrintHbxValues(AzMotor);
	PrintHbxValues(AltMotor);

	// Initialize HBX communications as outputs
	// It will use H2X communications
	HCL1Talk();                 // Set for Talking on RAClk
	HCL2Talk();                 // Set for Talking on DECClk
	HDAListen();
	TimerDelaymS(STARTTIME);

	// Reset the motors (RA and DEC)
	//  and wait until both respond to a command 
	dbgSerial.println("Waiting for both motors to start up ..");
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

	dbgSerial.println("Check Calibrate LEDs");

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

	dbgRx();                            // Check for comms from debug port for telescope values

	// Check ETX motors status and state
	if ((micros() - StatusTimer) > (ETXDELAY * 1000)) {   // ~6.55mS
		StatusTimer = micros();
		CheckETXState();
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
		HBXCheckTx();
		EQGTxoPtr &= EQGMASK;
	}
//	TimerDelaymS(1);
//	yield();
}

// End loop()

