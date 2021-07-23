/**@file*/
// HBXSerialServer.ino
// https://esp-idf.readthedocs.io/en/latest/api-reference/wifi/esp_now.html
#include "Hardware.h"
#include "ETXProtocol.h"
#include "EQGProtocol.h"
#include "EQG2HBX.h"
#include "HBXComms.h"
#include "HBXFileSystem.h"
#include "HBXWiFiServer.h"
#include <FS.h>   // Include the SPIFFS library

// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    dbgSerial.println("ESPNow Init Success");
  }
  else {
    dbgSerial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counter and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Handle ESP_NOW WiFi data
// Data in recvWiFi.text, length recvWiFi.len
void recvCallBack(const uint8_t *senderMac, const uint8_t *incomingData, int len) {
  
// Get data from WiFi buffer
  memcpy(&recvWiFi.text, incomingData, len);      // Receive data from EQMOD Tx
  recvWiFi.text[len] = 0;                         // Null terminate
  recvWiFi.len = len;
  
// Check if serial device is requesting restart
  if ((len == 20) && ((strncmp(recvWiFi.text, "Mount, please reply", 15) == 0))) {
		// Capture the serial device mac address
		for (byte n = 0; n < ESP_NOW_ETH_ALEN; n++) {
			serial_peer.peer_addr[n] = senderMac[n];
		}
		// Discard data from reconnect request, clear flags
    recvWiFi.len = 0;
    Connected = false;
    dataSending = false;
    // Reply to reconnect request
    strcpy(sendWiFi.text, "EQMOD WiFi Mount V1.0\n");
    sendWiFi.len = sizeof("EQMOD WiFi Mount V1.0\n");
    dbgSerial.println("Reconnecting");
		sendData(&serial_peer);
		return;
  }
}

// Send data to ESP_NOW
void sendData(const esp_now_peer_info_t* s_peer) {
	// If first time, create a peer2peer connection with the sender mac address
  if (!Connected) {
    esp_now_add_peer(s_peer);						// Only one paired device - the first one to respond
  }
  
	// Send data, if not waiting for previous send to complete
  if (!dataSending) {
    memcpy(sendWiFi8, &sendWiFi, sendWiFi.len);												// Need to satisfy esp_now
    esp_now_send(serial_peer.peer_addr, sendWiFi8, sendWiFi.len);
    dataSending = true;
  }  
}

// Get Send data status
void sendCallBack(const uint8_t* mac, esp_now_send_status_t sendStatus) {
  if (sendStatus == 0) {
    sendWiFi.len = 0;         // Data successfully sent
    Connected = true;
    dataSending = false;
  }
  // Do some error checking?
}

// Put received WiFi (UDP/ESP_NOW) data into the mount input data buffer for processing
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

// Webserver functions for ssid, pass, telescope etc
// =================================================

void onRequest(AsyncWebServerRequest *request){
  //Handle Unknown Request
	request->send(404, "text/plain", "The content you are looking for was not found.");
}

void onBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
	if (!index) {
		dbgSerial.printf("BodyStart: %u B\n", total);
	}
	for (size_t i = 0; i < len; i++) {
		dbgSerial.write(data[i]);
	}
	if (index + len == total) {
		dbgSerial.printf("BodyEnd: %u B\n", total);
	}
}

void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
	if (!index) {
		dbgSerial.printf("UploadStart: %s\n", filename.c_str());
	}
	for (size_t i = 0; i < len; i++) {
		dbgSerial.write(data[i]);
	}
	if (final) {
		dbgSerial.printf("UploadEnd: %s, %u B\n", filename.c_str(), index + len);
	}
}

void AsyncServerResponseSetup(void) {

 // respond to GET requests on URL /scan
		//First request will return 0 results unless you start scan from somewhere else (loop/setup)
		//Do not request more often than 3-5 seconds

	server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request){
		String json = "[";
		int n = WiFi.scanComplete();
		if(n == -2){
			WiFi.scanNetworks(true);
		} else if(n){
			for (int i = 0; i < n; ++i) {
				if(i) json += ",";
				json += "{";
				json += "\"rssi\":"+String(WiFi.RSSI(i));
				json += ",\"ssid\":\""+WiFi.SSID(i)+"\"";
				json += ",\"bssid\":\""+WiFi.BSSIDstr(i)+"\"";
				json += ",\"channel\":"+String(WiFi.channel(i));
				json += ",\"secure\":"+String(WiFi.encryptionType(i));
//				json += ",\"hidden\":"+String(WiFi.isHidden(i)?"true":"false");
				json += "}\r\n";
			}
			WiFi.scanDelete();
			if(WiFi.scanComplete() == -2){
				WiFi.scanNetworks(true);
			}
		}
		json += "]";
		request->send(200, "application/json", json);
		json = String();
	});

  // upload a file to /upload
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200);
  }, onUpload);

  // send /index.htm file when /index is requested
  server.on("/index", HTTP_ANY, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/www/index.htm");
  });

	// send /settings.htm file when /settings is requested
	server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/www/settings.htm");
		int paramsNr = request->params();
		dbgSerial.print("Number of settings parameters: "); dbgSerial.println(paramsNr);
		if (paramsNr) {
			for (int i = 0; i < paramsNr; i++) {
				AsyncWebParameter* p = request->getParam(i);
				dbgSerial.print("Param name: "); dbgSerial.print(p->name());
				dbgSerial.print(", value: "); dbgSerial.println(p->value());
				if ((p->name()) == "ssid")
					ssid = (p->value());
				if ((p->name()) == "pass")
					pass = (p->value());
				if ((p->name()) == "scope")
					strcpy(scope, (p->value()).c_str());
			}

			preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace

			dbgSerial.print("Original - ssid: "); dbgSerial.print(preferences.getString("STA_SSID"));
			dbgSerial.print(", pass: "); dbgSerial.print(preferences.getString("STA_PASS"));
			dbgSerial.print(", Telescope: "); dbgSerial.println(preferences.getString("TELESCOPE"));

			if (strlen(ssid.c_str()) != 0) preferences.putString("STA_SSID", ssid);
			if (strlen(pass.c_str()) != 0) preferences.putString("STA_PASS", pass);
			if (strlen(scope) != 0) preferences.putString("TELESCOPE", scope);

			dbgSerial.print("Updated - ssid: ");  dbgSerial.print(preferences.getString("STA_SSID"));
			dbgSerial.print(", pass: "); dbgSerial.print(preferences.getString("STA_PASS"));
			dbgSerial.print(", Telescope: "); dbgSerial.println(preferences.getString("TELESCOPE"));
			preferences.end();
		}
	});
	
	// HTTP basic authentication
  server.on("/login", HTTP_GET, [](AsyncWebServerRequest *request){
		dbgSerial.print("B4 http_username: ");
		dbgSerial.print(http_username);
		dbgSerial.print(", http_password: ");
		dbgSerial.println(http_password);

		if(!request->authenticate(http_username, http_password))
        return request->requestAuthentication();
		else {
			dbgSerial.print("http_username: ");
			dbgSerial.print(http_username);
			dbgSerial.print(", http_password: ");
			dbgSerial.println(http_password);
		}
    request->send(200, "text/plain", "Login Success!");
		loginValid = true;
  });

	// Simple Firmware Update Form
	// send /update.htm file when /index is requested
	server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (loginValid == true) {
			request->send(SPIFFS, "/update.htm");
		}
		else {
			request->send(200, "text/plain", "Before you can update firmware, Please Login");
		}
	});

	// Process POST from Update Form
	server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
		shouldReboot = !Update.hasError();
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot?"OK":"FAIL");
    response->addHeader("Connection", "close");
    request->send(response);
  },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index){
      dbgSerial.printf("Update Start: %s\n", filename.c_str());

//      Update.runAsync(true);
//			if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
			if (!Update.begin(0x140000)) {
        Update.printError(dbgSerial);
      }
    }
    if(!Update.hasError()){
      if(Update.write(data, len) != len){
        Update.printError(dbgSerial);
      }
    }
    if(final){
      if(Update.end(true)){
				dbgSerial.printf("Update Success: %uB\n", index+len);
      } else {
        Update.printError(dbgSerial);
      }
    }
  });
}

void browseService(const char * service, const char * proto) {
	Serial.printf("Browsing for service _%s._%s.local. ... ", service, proto);
	int n = MDNS.queryService(service, proto);
	if (n == 0) {
		Serial.println("no services found");
	}
	else {
		Serial.print(n);
		Serial.println(" service(s) found");
		for (int i = 0; i < n; ++i) {
			// Print details for each service found
			Serial.print("  ");
			Serial.print(i + 1);
			Serial.print(": ");
			Serial.print(MDNS.hostname(i));
			Serial.print(" (");
			Serial.print(MDNS.IP(i));
			Serial.print(":");
			Serial.print(MDNS.port(i));
			Serial.println(")");
		}
	}
	Serial.println();
}

// ================================================================================================

void HBXWiFiSetup() {

	// TODO
		// Read from EEPROM
		// ESPNOW

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
		preferences.begin("EQG2HBX", false);											// Access EQG2HBX namespace
		if (preferences.getString("AP_SSID", "none") == "none")
			preferences.putString("AP_SSID", "EQGWiFi");						// Default
		if (preferences.getString("AP_PASS", "none") == "none")
			preferences.putString("AP_PASS", "EQG2HBXPcl");					// Default
		ssid = preferences.getString("AP_SSID", "none");
		pass = preferences.getString("AP_PASS", "none");
		preferences.end();
    
		WiFi.persistent(false);
  	WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid.c_str(), pass.c_str());                  // softAP ssid, pass
    delay(2000);																							// Espressif wifi bug needs this
  	WiFi.softAPConfig(ip, gateway, netmask);									// softAP ip
		dbgSerial.print("SoftAP IP address: "); dbgSerial.println(WiFi.softAPIP());
  }
	else {
		// STA mode       EQMODWiFi connects to network router and gets an IP
		// For STA mode:  Host software must detect that IP
		// For STA mode:  UDP2Serial router network assigns IP address
		// For STA mode:  Start webserver, mDNS and accept changes to ssid, pass, Telescope etc

		dbgSerial.println("EQMODWiFi Station Mode");
		// Check preferences for ssid, pass
		preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace
		if (preferences.getString("STA_SSID", "none") == "none")
			preferences.putString("STA_SSID", "EQGnet");							// Default Home Network
		if (preferences.getString("STA_PASS", "none") == "none")
			preferences.putString("STA_PASS", "EQGconnect");
		ssid = preferences.getString("STA_SSID", "none");
		pass = preferences.getString("STA_PASS", "none");
		preferences.end();

		WiFi.mode(WIFI_STA);
		WiFi.disconnect();
		WiFi.begin(ssid.c_str(), pass.c_str());
		int i = 0;
		while ((WiFi.status() != WL_CONNECTED) && (i++ < 50)) {
			delay(1000);
			dbgSerial.print(i);	dbgSerial.print(", ssid: "); dbgSerial.print(ssid.c_str());	dbgSerial.print(", pass: "); dbgSerial.println(pass.c_str());
		}
		if (i >= 50) {
			WiFi.begin("EQGnet", "EQGconnect");
			while (WiFi.status() != WL_CONNECTED) {
				delay(100);
				dbgSerial.print("!");
			}
		}
		dbgSerial.print("ssid: "); dbgSerial.println(ssid.c_str());
		dbgSerial.print("pass: "); dbgSerial.println(pass.c_str());
		dbgSerial.print("IP:   "); dbgSerial.println(WiFi.localIP().toString().c_str());
		dbgSerial.println(" connected");
		if (!MDNS.begin("eqmodwifi")) {
			Serial.println("Error setting up MDNS responder!");
			while (1) {
				delay(1000);
			}
		}
		Serial.println("mDNS responder started");
		dbgSerial.print("ssid: "); dbgSerial.println(ssid.c_str());
		dbgSerial.print("pass: "); dbgSerial.println(pass.c_str());
		dbgSerial.print("IP:   "); dbgSerial.println(WiFi.softAPIP().toString().c_str());
		// Load 'server.on' responses
		AsyncServerResponseSetup();
		WiFi.scanNetworks();
		dbgSerial.print("ssid: "); dbgSerial.println(ssid.c_str());
		dbgSerial.print("pass: "); dbgSerial.println(pass.c_str());
		dbgSerial.print("IP:   "); dbgSerial.println(WiFi.softAPIP().toString().c_str());
		// attach filesystem root at URL /fs
		if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
			dbgSerial.println("SPIFFS Mount Failed, SPIFF formatted");
		}
		else
			dbgSerial.println("SPIFFS Mounted .. ");
		writeFile(SPIFFS, "/www/index.htm", EQ2HBX_Version.c_str());
		listDir(SPIFFS, "/", 0);

		server.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.htm");

		// Catch-All Handlers
		// Any request that can not find a Handler that canHandle it
		// ends in the callbacks below.
		server.onNotFound(onRequest);
		server.onFileUpload(onUpload);
		server.onRequestBody(onBody);

		server.begin();
		// Add service to MDNS-SD
		MDNS.addService("_http", "_tcp", 80);
		MDNS.addService("_osc", "_udp", localUdpPort);
		//browseService("http", "tcp");
	}

  if (!UDPFlag) {
  // ESP_NOW mode   (EQMODWiFi responds to MAC protocol)
    dbgSerial.println("ESP_NOW Mode");
    InitESPNow();
    dbgSerial.println("ESPNOW2SerialServer");
    dbgSerial.print("Mount soft  mac: "); dbgSerial.println(WiFi.softAPmacAddress());
  	dbgSerial.print("Mount hard  mac: "); dbgSerial.println(WiFi.macAddress());
//  	esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
  	esp_now_register_recv_cb(recvCallBack);	
	 	esp_now_register_send_cb(sendCallBack);
  }
  else {
    dbgSerial.println("UDP Mode");
    udp.begin(localUdpPort);
    dbgSerial.println("UDP2SerialServer");
    if (APFlag) 
      dbgSerial.printf("AP : ");
    else
      dbgSerial.printf("STA: ");
		dbgSerial.printf("Now listening at IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), localUdpPort);
  }

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
  else {
		// Get data to send to ESP_NOW - EQMOD/SynScanPro Serial WiFi
  	if (Connected) {
  		getTxDataFromMountOutputBuffer();
  		if (sendWiFi.len > 0)
  			sendData(&serial_peer);
  	}
  }
}
