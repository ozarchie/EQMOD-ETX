//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
// Modified John Archbold - February2019
//This code creates a bridge between Serial2 and Classical Bluetooth (SPP)
// It has been modified for the EQMOD serial device based on the ESP32
// Debug data is sent to the USB port
// Serial data is sent to/from the Serial2 prot from the BT port
// The EQG is connected directly to the Serial2 port

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

#define dbgSerial     Serial
#define EQGSerial     Serial2
#define EQGBluetooth  SerialBT

void setup() {
  Serial.begin(115200);
  EQGSerial.begin(9600, SERIAL_8N1, 18, 19);    // EQG via serial, bluetooth or WiFi
  EQGBluetooth.begin("EQGBlue"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
}

void loop() {
  if (EQGSerial.available()) {
    EQGBluetooth.write(Serial.read());
  }
  if (EQGBluetooth.available()) {
    EQGSerial.write(SerialBT.read());
  }
  delay(5);
}
