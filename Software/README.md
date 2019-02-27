THIS CODE IS NOT FULLY TESTED.  
ONLY USE IT IF YOU WANT TO HELP IN THE DEBUG PROCESS.  
<hr />
<h1>Installation</h1>
<h2>EQMOD</h2>
<hr />
<ol>
EQMOD does not support WiFi communications natively.  
It needs a Serial<->WiFi bridge.  
If you want to use EQMOD, you need to build the  
<li> [EQMOD-WiFi](https://github.com/ozarchie/EQMOD-WiFi), or  
<li> [EQMOD-BT](https://github.com/ozarchie/EQMOD-Bluetooth)  
    interface and code.  
</li>
If you are OK with SynScan, see below.  
<h2>ESP32</h2>
<hr />
<ol>
<h3>Download the github image and configure</h3>
<li>https://github.com/ozarchie/EQMOD-ETX/tree/master/Software</li>
<li>Make sure you have the espressif ESP32 system and libraries installed.    
[Download ESP32](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/windows.md)</li>
<li>Compile and upload to your hardware  
<h3>Download and install Synscan (Windows/Android) to drive the scope</h3>  
[Download SynScan](http://skywatcher.com/download/software/synscan-app/ "Title")  
Download and install ASCOM driver for Synscan (Windows) to interface to CdC etc  
[Download SynScan ASCOM](http://skywatcher.com/download/software/ascom-driver/ "Title")  
</li>
<li>Search and join WiFi network EQMODWiFi, Password is CShillit0</li>
<h3>Run on Windows - ASCOM only supports Windows</h3>
<li>Start SynScan and press Connect</li>
<li>Select EQ6</li>
<li>THEN Start CdC</li>
<li>Telescope>TelescopeSettings>Telescope: Check ASCOM</li>
<li>Telescope>ConnectTelescope>Select:SynScanMobile Telescope</li>
<li>Telescope>ConnectTelescope>Connect</li>