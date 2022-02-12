/*
 * gyro.h
 *
 *  Created on: Feb 11, 2022
 *      Author: swade
 */

#ifndef DRIVERS_GYRO_H_
#define DRIVERS_GYRO_H_
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

#define PWR_MGMT_ADDR           0x6B        // address of power management register
#define GY521_ADDRESS           0x68        // I2C address of GY-521 sensor
#define ACCEL_BASE_ADDR         0x3B        // base address of accelerometer data registers

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
extern void initGyro(void);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif



#endif /* DRIVERS_GYRO_H_ */
