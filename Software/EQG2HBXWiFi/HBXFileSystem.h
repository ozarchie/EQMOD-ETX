#pragma once
/**************************************************************
 *	SPIFFS filesystem
 *	Only invoked in STA mode
*/

#include <FS.h>
#include <SPIFFS.h>
#include <SPIFFSEditor.h>
#include <Preferences.h>

#define FORMAT_SPIFFS_IF_FAILED true
