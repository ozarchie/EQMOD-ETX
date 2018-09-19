// HBXSerialServer.ino
// https://esp-idf.readthedocs.io/en/latest/api-reference/wifi/esp_now.html

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
  uint8_t TxD, n;
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
		dbgSerial.write(EQGTxBuffer[EQGTxoPtr]);
		sendWiFi.text[TxDIndex++] = EQGTxBuffer[EQGTxoPtr++];
		EQGTxoPtr &= EQGMASK;
  }
	// Send when a CR is detected
	if (sendWiFi.text[TxDIndex - 1] == 0x0d) {
		sendWiFi.text[TxDIndex] = 0;
		sendWiFi.len = TxDIndex;
	}
}

// ================================================================================================

void HBXWiFiSetup() {

	pinMode(PROTOCOL, INPUT_PULLUP);		// ESP_NOW(1),  UDP(0)
	pinMode(MODE, INPUT_PULLUP);				// AP(1),       STA(0)
	pinMode(SERIAL, INPUT_PULLUP);
	
	digitalWrite(SERIAL, HIGH);
	digitalWrite(PROTOCOL, HIGH);
	digitalWrite(MODE, HIGH);
  
	dbgSerial.println("");
  dbgSerial.print("digitalRead(PROTOCOL): ");
  dbgSerial.println(digitalRead(PROTOCOL));
	dbgSerial.print("digitalRead(MODE)    : ");
	dbgSerial.println(digitalRead(MODE));
	dbgSerial.print("digitalRead(SERIAL)  : ");
	dbgSerial.println(digitalRead(SERIAL));
  
  if (digitalRead(PROTOCOL))
    UDPFlag = false;
  else
    UDPFlag = true;

  if (digitalRead(MODE))
    APFlag = true;
  else
    APFlag = false;

  if (APFlag) { 
  // AP mode       device connects directly to EQMODWiFi (no router)
  // For AP mode:  UDP2Serial: This ESP assigns IP addresses
  // For AP mode:  ESP IP is always 192.168.88.1 (set above)
    dbgSerial.println("Access Point Mode");
	// Check preferences for ssid, pass
		preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace
		if (preferences.getString("AP_SSID", "none") == "none")
			preferences.putString("AP_SSID", "EQMODWiFi");							// Default EQMOD
		if (preferences.getString("AP_PASS", "none") == "none")
			preferences.putString("AP_PASS", "CShillit0");
		ssid = preferences.getString("AP_SSID", "none");
		pass = preferences.getString("AP_PASS", "none");
		preferences.end();

  	WiFi.mode(WIFI_AP);
  	WiFi.softAPConfig(ip, ip, netmask);				// softAP ip
  	WiFi.softAP(ssid.c_str(), pass.c_str());									// softAP ssid, pass
  }
  else {
  // STA mode       EQMODWiFi connects to network router and gets an IP
  // For STA mode:  Host software must detect that IP
  // For STA mode:  UDP2Serial router network assigns IP address
    dbgSerial.println("Station Mode");
	// Check preferences for ssid, pass
		preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace
		if (preferences.getString("STA_SSID", "none") == "none")
			preferences.putString("STA_SSID", "HAPInet");							// Default Home Network
		if (preferences.getString("STA_PASS", "none") == "none")
			preferences.putString("STA_PASS", "HAPIconnection");
		ssid = preferences.getString("STA_SSID", "none");
		pass = preferences.getString("STA_PASS", "none");
		preferences.end();
		
		WiFi.mode(WIFI_STA);
  	WiFi.begin(ssid.c_str(), pass.c_str());
		int i = 0;
  	while ((WiFi.status() != WL_CONNECTED) && (i++ < 100)) {
  		delay(100);
      dbgSerial.print(".");
  	}
		if (i >= 100) {
			WiFi.begin("HAPInet", "HAPIconnect");
			while (WiFi.status() != WL_CONNECTED) {
				delay(100);
				dbgSerial.print("!");
			}
		}
    dbgSerial.println(" connected");
/*
		server.on("/hello", HTTP_GET, [](AsyncWebServerRequest *request) {
			request->send(200, "text/plain", "Hello World");
		});

		server.begin();
		*/
		}

	dbgSerial.print("ssid: "); dbgSerial.println(ssid.c_str());
	dbgSerial.print("pass: "); dbgSerial.println(pass.c_str());
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
      dbgSerial.printf("Now listening at IP %s, UDP port %d\n", WiFi.softAPIP().toString().c_str(), localUdpPort);
    else
      dbgSerial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  }

	TxDIndex = 0;
	sendWiFi.len = 0;
	recvWiFi.len = 0;
}

// -------------------------------------------------------------------------------------------

bool HBXCheckRx() {
	if (UDPFlag) {																					// UDP - SynScanPro UDP
		recvWiFi.len = udp.parsePacket();
		if (recvWiFi.len > 0) {
			// receive incoming UDP packets
			recvWiFi.len = udp.read(recvWiFi.text, 250);
			if (recvWiFi.len > 0) {
				recvWiFi.text[recvWiFi.len] = 0;		// Null terminate

//				dbgSerial.print("RxUDP - len: ");
//	      dbgSerial.print(recvWiFi.len);
//				dbgSerial.print(", data: ");
//        dbgSerial.println(recvWiFi.text);

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

 //       dbgSerial.print("TxUDP - len: ");
 //       dbgSerial.print(sendWiFi.len);
 //       dbgSerial.print(", data: ");
 //       dbgSerial.println(sendWiFi.text);

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


