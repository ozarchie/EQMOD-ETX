/********************************************************
  Hardware Definitions
  ====================
 *********************************************************/
#pragma once

#define mESP32

 // Serial port definitions for HBX interface
 // =========================================

#ifdef mESP32
//BluetoothSerial SerialBT;

#define dbgSerial			Serial
#define EQGSerial			Serial2
//#define EQGBluetooth	SerialBT

#endif

// Pin definitions for HBX interface
// =================================

#ifdef  mESP32
#define HDAAz			25          // Pin7 on HBX interface
#define HCLAz			26          // Pin6 on HBX interface
#define HDAAlt		14          // Pin5 on HBX interface
#define HCLAlt		27          // Pin4 on HBX interface
#define HDAAux		18          // Pin6 on HBX interface
#define HCLAux		19          // Pin7 on HBX interface


#define H2C_INPUTPU     INPUT_PULLUP	// Set pin data input mode
#define H2C_INPUT       INPUT					// Set pin data input mode
#define H2C_OUTPUT			OUTPUT				// Set pin data output

#define SDA				21          // Pin6 on HBX interface
#define SCL				22          // Pin7 on HBX interface

// Pin definitions for LED indicators
// ==================================
#define EQGLED          21
#define SCOPELED        22

// Jumpers to run test
// ==============================
#define TESTHBX         13       // GPI35
#endif
