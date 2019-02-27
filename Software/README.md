THIS CODE IS NOT FULLY TESTED.  
ONLY USE IT IF YOU WANT TO HELP IN THE DEBUG PROCESS.  
<hr />
<h1>Installation</h1>

<hr />
<ol>

<h2>ESP32 Interface</h2>
<hr />
<ol>
<h3>Download the github EQMOD-ETX and configure</h3>
<li>https://github.com/ozarchie/EQMOD-ETX/tree/master/Software</li>
<li>Make sure you have the espressif ESP32 system and libraries installed.    
[Download ESP32](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/windows.md)</li>
<li>Compile and upload to your hardware</li>  

<ol>

<h2>SYNSCAN</h2>
<h3>Run on Windows - ASCOM only supports Windows</h3>
<li>Download and install Synscan Appn (Windows)</li>
[Download SynScan](http://skywatcher.com/download/software/synscan-app/ "Title")  
<li>Download and install ASCOM driver for Synscan (Windows) to interface to CdC (or other ASCOM-based program)</li>
[Download SynScan ASCOM](http://skywatcher.com/download/software/ascom-driver/ "Title")
<li>Search and join WiFi network EQMODWiFi, Password is CShillit0</li>
<li>Start SynScan and press Connect</li>
<li>Select EQ6</li>
<li>Start CdC</li>
<li>Telescope>TelescopeSettings>Telescope: Check ASCOM</li>
<li>Telescope>ConnectTelescope>Select:SynScanMobile Telescope</li>
<li>Telescope>ConnectTelescope>Connect</li>  

<h3>Run using phone or tablet App</h3>
<li>Download and install Synscan Appn (iOS/Android)</li>
[Download SynScan](http://skywatcher.com/download/software/synscan-app/ "Title")
<li>Search and join WiFi network EQMODWiFi, Password is CShillit0</li>
<li>Start SynScan and press Connect</li>
<h2>EQMOD</h2>
EQMOD does not support WiFi communications natively.  

It needs a Serial<->WiFi bridge.  

If you want to use EQMOD, you need to build the  
<li> [EQMOD-WiFi](https://github.com/ozarchie/EQMOD-WiFi)  
 
    interface and code.  
</li>
<li>Plug the EQMOD-WiFi device into a USB port</li>
<li>Configure CdC/EQMOD to use the USB serial port associated with the EQMOD-WiFi</li>
<li>The EQMOD-WiFi and EQMOD-ETX will handle the WiFi communications using ESP-NOW</li>
