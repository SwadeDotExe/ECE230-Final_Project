/*
 * L293D.h
 *
 *  Created on: Feb 7, 2022
 *      Author: Bryce Bejlovec
 */

#ifndef L293D_H_
#define L293D_H_
#include <stdint.h>




#endif /* L293D_H_ */
#define PWM_PERIOD                  0x0800      // approx 50Hz signal

/*!
 * \brief This function configures pins and timer for L293D driver
 *
 * This function configures P2.4/5/6/7 as output pins for L293D signals
 *  initializes Timer_A0 CCR1/2/3/4 for PWM output
 *
 * Modified bits 0-3 of \b P3DIR register and \b P2SEL registers.
 * Modified \b TA0CTL, \b TA0CCTL1 and CCR registers.
 *
 * \return None
 */
extern void initL293D(void);

/*!
 * \brief This function sets color of RGB LED
 *
 * This function sets PWM to signed 32-bit values \a speed and direction
 *
 *  \param left and right signed 32-bit values
 *
 * Modified \b TA0CCR1/2/3/4 register.
 *
 * \return None
 */
extern void setMotorPWM(int16_t left, int16_t right);
