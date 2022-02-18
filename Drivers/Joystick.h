/*
 * Joystick.h
 *
 *  Created on: Feb 11, 2022
 *      Author: bejlovba
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_
#include <stdint.h>
#include "msp.h"

/*!
 * \brief This function configures pins and timer for Joystick driver
 *
 * This function configures P5.4/5 as input pins for Joystick ADC signals
 *  initializes ADC14 for MEM0/1
 *
 * Modified bits 0-3 of \b P3DIR register and \b P2SEL registers.
 * Modified \b TA0CTL, \b ADC14CTL1 and CCR registers.
 *
 * \return None
 */
extern void initJoystick(void);



/*!
 * \brief This function returns the X value of the joystick.
 *
 * This function returns the value of the X joystick read by ADC
 *
 * \return uint16_t
 */
extern int16_t readJoystickX();

/*!
 * \brief This function returns the Y value of the joystick.
 *
 * This function returns the value of the Y joystick read by ADC
 *
 * \return uint16_t
 */
extern int16_t readJoystickY();


#endif /* JOYSTICK_H_ */
