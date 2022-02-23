/*
 * tachometer.h
 *
 *  Created on: Feb 10, 2022
 *      Author: swade
 */

#ifndef DRIVERS_TACHOMETER_H_
#define DRIVERS_TACHOMETER_H_
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

#define SW_PORT     P1

extern void initTachometer(void);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif



#endif /* DRIVERS_TACHOMETER_H_ */
