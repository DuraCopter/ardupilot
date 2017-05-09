#pragma once

#include "ap_version.h"

#define THISFIRMWARE "DuraCopter V3.5.0-rc8"
#define FIRMWARE_VERSION 3,5,0,FIRMWARE_VERSION_TYPE_RC

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
