/********************************************************
  Hardware Definitions
  ====================
 *********************************************************/
#pragma once

#define mESP32

 // Configuration for WiFi interface
 // ================================

	// UDP allows communications with the Skywatcher SynScan app on Laptop, Tablet or phone
	// ESPNOW is  alocal area wifi net .. TODO
	// AP provides a n access point in absence of Telscope wifi network
	// STA enables use of wifi credentials to log on to Telescope wifi network

#define SelectUDP			  // Either UDP or ESPNOW
#undef SelectAP			  // Either AP  or STA

 // Serial port definitions for HBX interface
 // =========================================

#ifdef mESP32
#define dbgSerial			Serial
#define EQGSerial			Serial2

#ifdef BTSerial
  #define EQGBluetooth	SerialBT
#endif
#endif
// Pin definitions for ESP32-HBX interface
// =======================================

#ifdef  mESP32
                              // 8Pin RJ
                              // =======
                              // Pin8 Gnd
#define HDAAz			25          // Pin7 on HBX interface
#define HCLAz			26          // Pin6 on HBX interface
#define HDAAlt		14          // Pin5 on HBX interface
#define HCLAlt		27          // Pin4 on HBX interface
#define HDAAux		18          // Pin3 on HBX interface
#define HCLAux		19          // Pin2 on HBX interface
                              // Pin1 Vbat

                              // 4Pin RJ
                              // =======
                              // Pin1 Gnd
#define HDAAux		18          // Pin2 on HBX interface
#define HCLAux		19          // Pin3 on HBX interface
                              // Pin4 Vbat

#define H2C_INPUTPU     INPUT_PULLUP	// Set pin data input mode
#define H2C_INPUT       INPUT					// Set pin data input mode
#define H2C_OUTPUT			OUTPUT				// Set pin data output

// Pin definitions for LED indicators
// ==================================
#define EQGLED    21
#define ETXLED    22

#endif
