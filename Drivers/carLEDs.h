/*
 * carLEDs.h
 *
 *  Created on: Feb 12, 2022
 *      Author: swade
 */

#ifndef DRIVERS_CARLEDS_H_
#define DRIVERS_CARLEDS_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "msp.h"
#include <stdbool.h>

extern void initCarLEDs(bool debug  );

extern void headlightsToggle(bool status);

extern void brakelightsToggle(bool status);

extern void turnSignalToggle(bool status, bool side);

extern void underLightsToggle(bool status);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif



#endif /* DRIVERS_CARLEDS_H_ */
