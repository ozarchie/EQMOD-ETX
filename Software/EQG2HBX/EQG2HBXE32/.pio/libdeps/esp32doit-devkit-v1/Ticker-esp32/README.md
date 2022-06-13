# Deprecated

This Ticker library has just been merged into the Arduino core for ESP32. This repo will not be maintained anymore.

# Ticker library for esp32

[![Build Status](https://travis-ci.org/bertmelis/Ticker-esp32.svg?branch=master)](https://travis-ci.org/bertmelis/Ticker-esp32)

This library acts as a compatibility layer to add Ticker functionality as seen in the ESP8266 core for the Arduino environment. As such the same function calls can be used on ESP32 as on ESP8266.

There are a number of differences and usage hints:
* This lib uses the SDK timer API
* Callbacks are not handled in interrupts so an interrupt can still interfere in the callback-code
* Although there are no interrupts involved, blocking code in the callback is not supported
* Switching from a one-time timer to a repeating timer is not supported. Trying to will not work

## Usage
See the examples for usage.

## To do
As this is a quick and dirty version, there will probably be some things I overlooked.
* optimize code?
* did I actually implement every use case?


## License
As the purpose of this lib was to create esp8266-compatible interface, the header file was almost completely taken from [igrr's Ticker library for esp8266](https://github.com/esp8266/Arduino/blob/master/libraries/Ticker/Ticker.h), credits go to him.

Hence, the implementation is also released under LGPL 2.1 (or later).
