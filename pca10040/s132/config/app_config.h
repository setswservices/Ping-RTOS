/////////////////////////////////////////////////////////////////////////////////////////////
//
//	File Name:		app_config.h
//	Author(s):		Jeffery Bahr
//	Copyright Notice:	Copyright, 2019, Ping LLC
//
//	Purpose/Functionality:	Version information and overrides to sdk_config.h, as per Nordic convention
//
//	Because this is always the first, mandatory header file in any .c file, we'll place some other header
//	files at the front that are potentially used by everybody.
//
/////////////////////////////////////////////////////////////////////////////////////////////


#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
//#include "nordic_common.h"
//#include "nrf.h"

///////////////////////////////////////////////////////////////////////////////////////////////
// Version
///////////////////////////////////////////////////////////////////////////////////////////////

// use BCD for the digits here, hex makes no sense

#define FW_MAJ_REV       		1 		 // major rev for substantial changes or releases
#define FW_MIN_REV       		1  		// less major revisions
#define FW_BUILD_NUMBER 	0 	// build numbers used for test revisions within a minor rev

#define STRING_IT(x) # x
#define VERSION_TO_STRING(major, minor, build) "v" STRING_IT(major) "." STRING_IT(minor) "." STRING_IT(build)
#define FIRMWARE_VERSION_STRING \
    VERSION_TO_STRING(FW_MAJ_REV, FW_MIN_REV, FW_BUILD_NUMBER)


///////////////////////////////////////////////////////////////////////////////////////////////
// Defines to override those in sdk_config.h
///////////////////////////////////////////////////////////////////////////////////////////////

#define	TWI_ENABLED				1

#define	NRFX_TWIM_ENABLED 		1
#define	NRFX_TWIM1_ENABLED		1

//#define	BSP_SIMPLE					1
//#define	BUTTONS_NUMBER			0

#endif // APP_CONFIG_H
