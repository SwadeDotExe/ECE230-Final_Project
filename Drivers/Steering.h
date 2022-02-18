/*
 * Steering.h
 *
 *  Created on: Feb 11, 2022
 *      Author: Bryce Bejlovec
 */

#ifndef STEERING_H_
#define STEERING_H_

#include <stdint.h>
#include "msp.h"

#define MAXJOY 2^12 -1;
#define MINJOY 0;
#define MAXPWM 2000;

/*!
 * \This function maps the joystick Y value to an appropriate value to be used for motor PWM
 *
 * Translates the 12bit ADC value to one that is able
 * to be transmitted to the RC car for PWM usage.
 *
 * \return int32_t
 *
 */
extern int32_t mapStickY(int16_t stickValue);

/*!
 * \This function maps the joystick X value to an appropriate value to be used for left side motor PWM
 *
 * Translates the 12bit ADC value to one that is able
 * to be transmitted to the RC car for PWM usage.
 *
 * \return int32_t
 *
 */
extern int32_t mapStickXL(int16_t stickValue);

/*!
 * \This function maps the joystick X value to an appropriate value to be used for right side motor PWM
 *
 * Translates the 12bit ADC value to one that is able
 * to be transmitted to the RC car for PWM usage.
 *
 * \return int32_t
 *
 */
extern int32_t mapStickXR(uint16_t stickValue);

#endif /* STEERING_H_ */
