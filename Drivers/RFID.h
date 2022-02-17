/*
 * RFID.h
 *
 *  Created on: Feb 14, 2022
 *      Author: ciratasb
 */

#ifndef DRIVERS_RFID_H_
#define DRIVERS_RFID_H_

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

/* SPI pin definitions */
#define CS_PORT P3
#define CS_BIT  BIT6
#define SPI_PORT P1
#define CLK_BIT BIT5
#define MOSI_BIT BIT6
#define MISO_BIT BIT7

/*!
 * \brief This function configures LED2 pins as output pins
 *
 * This function configures P2.0, P2.1, and P2.2 as output pins
 *  for the RGB LED2, and initializes LED to 'off'
 *
 * Modified bits 0 to 2 of \b P2DIR register and \b P2SEL registers.
 *
 * \return None
 */
extern void initTachometer(void);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif




#endif /* DRIVERS_RFID_H_ */
