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
