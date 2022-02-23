/*
 * relay.h
 *
 *  Created on: Feb 15, 2022
 *      Author: swade
 */

#ifndef DRIVERS_RELAY_H_
#define DRIVERS_RELAY_H_

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

extern void setupRelay(void);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_RELAY_H_ */
