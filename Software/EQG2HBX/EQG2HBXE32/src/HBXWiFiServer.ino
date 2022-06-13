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
#include <FS.h>   // Include the SPIFFS library
#include "SPIFFS.h"

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
  server.on("/index", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/www/index.htm");
  });

	// send /settings.htm file when /settings is requested
	server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(SPIFFS, "/www/settings.htm");
		int paramsNr = request->params();
		int i, j;
		dbgSerial.print("Number of settings parameters: "); dbgSerial.println(paramsNr);

		if (paramsNr) {
			preferences.begin("EQG2HBX", false);						// Access EQG2HBX namespace

			dbgSerial.print("Original -");
			dbgSerial.print(" AP_ssid: "); dbgSerial.print(preferences.getString("AP_SSID"));
			dbgSerial.print(", AP_pass: "); dbgSerial.print(preferences.getString("AP_PASS"));
			dbgSerial.print(", STA_ssid: "); dbgSerial.print(preferences.getString("STA_SSID"));
			dbgSerial.print(", STA_pass: "); dbgSerial.print(preferences.getString("STA_PASS"));
			dbgSerial.print(", WiFi Mode: "); dbgSerial.print(preferences.getString("WIFIMODE"));
			dbgSerial.print(", Protocol: "); dbgSerial.print(preferences.getString("PROTOCOL"));
			dbgSerial.print(", Telescope: "); dbgSerial.print(preferences.getString("TELESCOPE"));
			dbgSerial.print(", Mount: "); dbgSerial.println(preferences.getString("MOUNT"));

			for (j = 0; j < paramsNr; j++) {
				AsyncWebParameter* p = request->getParam(j);
				dbgSerial.print("Param name: "); dbgSerial.print(p->name());
				dbgSerial.print(", value: "); dbgSerial.println(p->value());
				if ((p->name()) == "AP_ssid")		APssid = (p->value());
				if ((p->name()) == "AP_pass")		APpass = (p->value());
				if ((p->name()) == "STA_ssid")		STAssid = (p->value());
				if ((p->name()) == "STA_pass")		STApass = (p->value());
				if ((p->name()) == "mode")		mode = (p->value());
				if ((p->name()) == "protocol")	protocol = (p->value());
				if ((p->name()) == "scope")		scope = (p->value());
				if ((p->name()) == "mount")		mount = (p->value());
			}

			scopetype = scopedefault;
			modetype = modedefault;
			protocoltype = protocoldefault;
			mounttype = mountdefault;

			for (i=0;i<16;i++) {
				dbgSerial.printf(" %d: ",i);dbgSerial.print(ratio[i][0].Telescope);
				if (strcmp(ratio[i][0].Telescope, scope.c_str()) == 0) {
					scopetype = i;
					dbgSerial.printf(" %d: ",i);dbgSerial.print(ratio[i][0].Telescope);
					dbgSerial.printf(" %d: ",i);dbgSerial.println(scope.c_str());
				}
			}

			for (i=0;i<3;i++) {
				if (strcmp(modeDesc[i], mode.c_str()) == 0) { modetype = i;
				dbgSerial.printf(" %d: ",i);dbgSerial.print(modeDesc[i]);
				dbgSerial.printf(" %d: ",i);dbgSerial.println(mode.c_str());}
			}

			for (i=0;i<3;i++) {
				if (strcmp(protocolDesc[i], protocol.c_str()) == 0) protocoltype = i;
			}

			for (i=0;i<3;i++) {
				if (strcmp(mountDesc[i], mount.c_str()) == 0) mounttype = i;
			}

			if (strlen(STAssid.c_str()) != 0) preferences.putString("AP_SSID", APssid);
			if (strlen(STApass.c_str()) != 0) preferences.putString("AP_PASS", APpass);
			if (strlen(STAssid.c_str()) != 0) preferences.putString("STA_SSID", STAssid);
			if (strlen(STApass.c_str()) != 0) preferences.putString("STA_PASS", STApass);
			if (strlen(mode.c_str()) != 0) preferences.putString("WIFIMODE", mode);
			if (strlen(protocol.c_str()) != 0) preferences.putString("PROTOCOL", protocol);
			if (strlen(scope.c_str()) != 0) preferences.putString("TELESCOPE", scope);
			if (strlen(mount.c_str()) != 0) preferences.putString("MOUNT", mount);

			preferences.putUChar("MODETYPE", modetype);
			preferences.putUChar("PROTOCOLTYPE", protocoltype);
			preferences.putUChar("SCOPETYPE", scopetype);
			preferences.putUChar("MOUNTTYPE", mounttype);

			dbgSerial.println("");
			dbgSerial.print("Updated  - ");
			dbgSerial.print("APssid: "); dbgSerial.print(preferences.getString("AP_SSID"));
			dbgSerial.print(", APpass: "); dbgSerial.print(preferences.getString("AP_PASS"));
			dbgSerial.print("STAssid: "); dbgSerial.print(preferences.getString("STA_SSID"));
			dbgSerial.print(", STApass: "); dbgSerial.print(preferences.getString("STA_PASS"));
			dbgSerial.print(", WiFi Mode: "); dbgSerial.print(preferences.getString("WIFIMODE"));dbgSerial.printf(" %d",modetype);
			dbgSerial.print(", Protocol: "); dbgSerial.print(preferences.getString("PROTOCOL"));dbgSerial.printf(" %d",protocoltype);
			dbgSerial.print(", Telescope: "); dbgSerial.print(preferences.getString("TELESCOPE"));dbgSerial.printf(" %d",scopetype);
			dbgSerial.print(", Mount: "); dbgSerial.print(preferences.getString("MOUNT"));dbgSerial.printf(" %d",mounttype);dbgSerial.println("");
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

void HBXWiFiServerSetup() {
	uint8_t numSsid = 0;
		
		if (!MDNS.begin("EQGWiFi")) {
			Serial.println("Error setting up MDNS responder!");
			while (1) {
				delay(1000);
			}
		}
		Serial.println("mDNS responder started");
		// Load 'server.on' responses
		AsyncServerResponseSetup();
		numSsid = WiFi.scanNetworks();
		Serial.print("Number of available WiFi networks discovered:");
		Serial.println(numSsid);

/*        for (int i = 0; i < numSsid; ++i) {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
        }
*/
		// attach filesystem root at URL /fs
		if (!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
			dbgSerial.println("SPIFFS Mount Failed, SPIFF formatted");
		}
		else
			dbgSerial.println("SPIFFS Mounted .. updating Version");
		writeFile(SPIFFS, "/www/index.htm", EQ2HBX_Version.c_str());
		listDir(SPIFFS, "/", 0);

		server.serveStatic("/", SPIFFS, "/www/").setDefaultFile("index.htm");

		// Catch-All Handlers
		// Any request that can not find a Handler that can Handle it
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
