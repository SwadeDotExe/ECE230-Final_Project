/*! \file */
/*!
 * pwmRGBLED.h
 *
 * Description: RGB LED full-color driver using PWM for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured for 12MHz.
 *              Uses Timer_A0 and P2.4/5/6 (TA0.1/2/3)
 *
 *  Created on: 1/31/2022
 *      Author: Swade Cirata
 */

#ifndef PWMRGBLED_H_
#define PWMRGBLED_H_

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

#define LED_PWM_PERIOD                  0x0800      // approx 50Hz signal
#define SHIFT_MULTIPLIER                3           // for 0x800 period and 8-bit color
#define RED_OFFSET                      16
#define GREEN_OFFSET                    8
#define BLUE_OFFSET                     0


/*!
 * \brief This function configures pins and timer for PWM RGB LED driver
 *
 * This function configures P2.4/5/6 as output pins for RGB LED signals and
 *  initializes Timer_A0 CCR1/2/3 for PWM output
 *
 * Modified bits 4-6 of \b P2DIR register and \b P2SEL registers.
 * Modified \b TA0CTL, \b TA0CCTL1 and CCR registers.
 *
 * \return None
 */
extern void initPWMRGBLED(void);


/*!
 * \brief This function sets color of RGB LED
 *
 * This function sets color of RGB LED to 24-bit RGB value \a color
 *
 *  \param color 24-bit RGB value
 *
 * Modified \b TA0CCR1/2/3 register.
 *
 * \return None
 */
extern void setPWMRGBColor(uint32_t color);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* PWMRGBLED_H_ */
