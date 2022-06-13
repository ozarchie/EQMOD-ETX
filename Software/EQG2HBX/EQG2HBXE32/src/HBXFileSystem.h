#pragma once
/**************************************************************
 *	SPIFFS filesystem
 *	Only invoked in STA mode
*/

#include <EEPROM.h>
#include <FS.h>
#include <SPIFFS.h>
//#include <SPIFFSEditor.h>

#define FORMAT_SPIFFS_IF_FAILED true
#pragma once

void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
void readFile(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void renameFile(fs::FS &fs, const char * path1, const char * path2);
void deleteFile(fs::FS &fs, const char * path);
