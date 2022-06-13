/**@file*/
// HBXSerialServer.ino
// https://esp-idf.readthedocs.io/en/latest/api-reference/wifi/esp_now.html
#include "Hardware.h"
#include "ETXProtocol.h"
#include "EQGProtocol.h"
#include "EQG2HBX.h"
#include "HBXComms.h"
#include "HBXFileSystem.h"
#include "HBXWiFi.h"

// Data in recvWiFi.text, length recvWiFi.len
void recvCallBack(const uint8_t *senderMac, const uint8_t *incomingData, int len) {
// Get data from WiFi buffer
  memcpy(&recvWiFi.text, incomingData, len);      // Receive data from EQMOD Tx
  recvWiFi.text[len] = 0;                         // Null terminate
  recvWiFi.len = len;
}

// Put received WiFi (UDP) data into the mount input data buffer for processing
// Data in recvWiFi.text, length recvWiFi.len
void putRxDataIntoMountInputBuffer(void) {
  uint8_t n;
  n = 0;
	while (n < recvWiFi.len) {
		EQGRxBuffer[EQGRxiPtr++] = recvWiFi.text[n];
		EQGRxiPtr &= EQGMASK;
		n += 1;
	}
  recvWiFi.len = 0;
}

// Get mount data to send to WiFi (UDP/NOW) into WiFi buffer
// Data in sendWiFi.text, length sendWiFi.len
void getTxDataFromMountOutputBuffer(void) {
	while (EQGTxoPtr != EQGTxiPtr) {
		if ((EQGTxBuffer[EQGTxoPtr] != CR) && (EQGTxBuffer[EQGTxoPtr] != LF))
		 dbgSerial.write(EQGTxBuffer[EQGTxoPtr]);
		else dbgSerial.write('.');
		sendWiFi.text[TxDIndex++] = EQGTxBuffer[EQGTxoPtr++];
		EQGTxoPtr &= EQGMASK;
  }
	// Send when a CR is detected
	if (sendWiFi.text[TxDIndex - 1] == 0x0d) {
		sendWiFi.text[TxDIndex] = 0;
		sendWiFi.len = TxDIndex;
	}
}

void HBXWiFiCommsSetup() {

	// TODO
		// Read from EEPROM

	UDPFlag = false;
#ifdef SelectUDP
	UDPFlag = true;			// Either UDP or ESPNOW
#endif
	APFlag = false;
#ifdef SelectAP
	APFlag = true;			// Either AP  or STA
#endif

	if (APFlag) { 
	// AP mode       device connects directly to EQMODWiFi (no router)
	// For AP mode:  UDP2Serial: This ESP assigns IP addresses
	// For AP mode:  ESP IP is always 192.168.88.1 (set above)

		dbgSerial.println("EQMODWiFi Access Point Mode");
		// Check preferences for ssid, pass
			preferences.begin("EQG2HBX", false);								// Access EQG2HBX namespace
			if (preferences.getString("AP_SSID", "none") == "none")
				preferences.putString("AP_SSID", "EQGWiFi");					// Default
			if (preferences.getString("AP_PASS", "none") == "none")
				preferences.putString("AP_PASS", "EQG2HBXPcl");					// Default
			APssid = preferences.getString("AP_SSID", "none");
			APpass = preferences.getString("AP_PASS", "none");
			preferences.end();
		
			WiFi.persistent(false);
		WiFi.mode(WIFI_AP);
		WiFi.softAP(APssid.c_str(), APpass.c_str());                  			// softAP ssid, pass
		delay(2000);															// Espressif wifi bug needs this
		WiFi.softAPConfig(ip, gateway, netmask);								// softAP ip
			dbgSerial.print("SoftAP IP address: "); dbgSerial.println(WiFi.softAPIP());
	}
	if (STAFlag) { 
		// STA mode       EQMODWiFi connects to network router and gets an IP
		// For STA mode:  Host software must detect that IP
		// For STA mode:  UDP2Serial router network assigns IP address
		// For STA mode:  Start webserver, mDNS and accept changes to ssid, pass, Telescope etc

		dbgSerial.println("EQMODWiFi Station Mode");
		// Check preferences for ssid, pass
		preferences.begin("EQG2HBX", false);								// Access EQG2HBX namespace
		if (preferences.getString("STA_SSID", "none") == "none")
			preferences.putString("STA_SSID", "EQGnet");					// Default Home Network
		if (preferences.getString("STA_PASS", "none") == "none")
			preferences.putString("STA_PASS", "EQGconnect");
		STAssid = preferences.getString("STA_SSID", "none");
		STApass = preferences.getString("STA_PASS", "none");
		preferences.end();

		WiFi.disconnect();
		delay(100);
		WiFi.mode(WIFI_STA);
		WiFi.begin(STAssid.c_str(), STApass.c_str());
		int i = 0;
		while ((WiFi.status() != WL_CONNECTED) && (i++ < 50)) {
			delay(1000);
			dbgSerial.print(i);	dbgSerial.print(", ssid: "); dbgSerial.print(STAssid.c_str());	dbgSerial.print(", pass: "); dbgSerial.println(STApass.c_str());
		}
		if (i >= 50) {
			WiFi.begin(STAssid.c_str(), STApass.c_str());
			while (WiFi.status() != WL_CONNECTED) {
				delay(100);
				dbgSerial.print("!");
			}
		}
		dbgSerial.print("ssid: "); dbgSerial.println(STAssid.c_str());
		dbgSerial.print("pass: "); dbgSerial.println(STApass.c_str());
		dbgSerial.print("IP:   "); dbgSerial.println(WiFi.localIP().toString().c_str());
		dbgSerial.println(" ... connected");
	}

    dbgSerial.println("UDP Mode");
    udp.begin(localUdpPort);
    dbgSerial.println("UDP2SerialServer");
    if (APFlag) 
        dbgSerial.printf("AP : ");
    if (STAFlag) 
        dbgSerial.printf("STA: ");
	    dbgSerial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

	TxDIndex = 0;
	sendWiFi.len = 0;
	recvWiFi.len = 0;
}

// -------------------------------------------------------------------------------------------

bool HBXCheckRx() {
	if (UDPFlag) {
// UDP - SynScanPro UDP
		recvWiFi.len = udp.parsePacket();
		if (recvWiFi.len > 0) {
			// receive incoming UDP packets
			recvWiFi.len = udp.read(recvWiFi.text, 250);
			if (recvWiFi.len > 0) {
				recvWiFi.text[recvWiFi.len] = 0;		// Null terminate

//				dbgSerial.print("RxUDP - len: ");
//	      dbgSerial.print(recvWiFi.len);
//				dbgSerial.print(", data: ");
 //       dbgSerial.println(recvWiFi.text);

			}
		}
	}
	if (recvWiFi.len > 0) {				// Send data to mount Input data buffer for processing
		putRxDataIntoMountInputBuffer();
		return true;								// Data received from UDP or ESP_NOW callback
	}
	else
		return false;								// No Data received
}

void HBXCheckTx() {
	if (UDPFlag) {
		// Get data to send to SynScanPro via UDP into WiFi buffer
		getTxDataFromMountOutputBuffer();
    // send back a reply, to the IP address and port we got the packet from
    if (sendWiFi.len > 0) {

//        dbgSerial.print("TxUDP - len: ");
//        dbgSerial.print(sendWiFi.len);
//        dbgSerial.print(", data: ");
//        dbgSerial.println(sendWiFi.text);

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
			memcpy(sendWiFi8, &sendWiFi.text, sendWiFi.len);
      udp.write(sendWiFi8, sendWiFi.len);
      udp.endPacket();
    }
    sendWiFi.len = 0;
		TxDIndex = 0;     
  }
}
