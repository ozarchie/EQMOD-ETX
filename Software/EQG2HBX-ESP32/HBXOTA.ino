#include <ArduinoOTA.h>
bool ota_started;

const char* ssid = "YOUR_ROUTER_SSID";
const char* password = "YOUR_ROUTER_WIFI_PASSWORD";
void setup() 
{
 Serial.begin(115200);
 ConnectToWiFi();
 StartOTAIfRequired();
 PrintWifiStatus();
 Serial.println("Connected to wifi");
}
void loop() 
{
 //Serial.println("Hello world");
 HandleOTA();
}

void ConnectToWiFi()
{

 Serial.println("Booting");
 WiFi.mode(WIFI_STA);
 Serial.println("Mode set");
 WiFi.begin(ssid, password);
 Serial.println("Begin complete");
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
 ArduinoOTA.setPassword((const char *)"123");
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
void PrintWifiStatus() 
{
 // print the SSID of the network you're attached to:
 Serial.print("SSID: ");
 Serial.println(WiFi.SSID());
 //using dhcp? wait for ip or ip not set!
 if (WiFi.localIP()[0] == 0)
 {
 Serial.println("DHCP: Waiting for IP Address ...");
 while (WiFi.localIP()[0] == 0)
 {
 yield();
 }
 }
 // print your WiFi shield's IP address:
 IPAddress ip = WiFi.localIP();
 Serial.print("IP Address: ");
 Serial.println(ip);
 //Serial.println(WiFi.status());
}