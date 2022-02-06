/*! \file */
/*!
 * pwmRGBLED.c
 *
 * Description: RGB LED full-color driver using PWM for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured for 12MHz.
 *              Uses Timer_A0 and P2.4-6 (TA0.1-3)
 *
 *  Created on: 1/31/2022
 *      Author: Swade Cirata
 */

#include <Drivers/pwmRGBLED.h>
#include "msp.h"


void initPWMRGBLED(void) {
    /* Configure RGB LED pins */
    P2->SEL0 |= BIT4 | BIT5 | BIT6;         // set 3 pins as GPIO
    P2->SEL1 &= ~(BIT4 | BIT5 | BIT6);
    P2->OUT &= ~(BIT4 | BIT5 | BIT6);       // clear output (LED off)
    P2->DIR |= BIT4 | BIT5 | BIT6;          // set as output pins


    /* Configure Timer_A0 and CCR1/2/3 */
    // Set period of Timer_A0 in CCR0 register for Up Mode
    TIMER_A0->CCR[0] = LED_PWM_PERIOD;
    // Set initial positive pulse-width of PWM in CCR1/2/3 register
    TIMER_A0->CCR[1] = 0;
    TIMER_A0->CCR[2] = 0;
    TIMER_A0->CCR[3] = 0;
    // Configure CCR1/2/3 for Compare mode, Reset/Set output mode, with interrupt disabled
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    // Configure Timer_A0 in Up Mode, with source SMCLK, prescale 28:1, and
    //  interrupt disabled  -  tick rate will be (12MHz/28)
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |   // SMCLK
            TIMER_A_CTL_ID__4 |                 // /4 divider
            TIMER_A_CTL_MC__UP |                // Up mode
            TIMER_A_CTL_CLR;                    // Clear TAR
    TIMER_A0->EX0 = TIMER_A_EX0_IDEX__7;        // /7 divider
}

void setPWMRGBColor(uint32_t color) {
    // Update CCR1/2/3 registers to set new pulse-width for red, green,
    //  blue intensity, respectively. \a color is a 24-bit value:
    //          upper 8 bits represent red intensity (0 to 255)
    //          middle 8 bits represent green intensity (0 to 255)
    //          lower 8 bits represent blue intensity (0 to 255)
    TIMER_A0->CCR[1] = ((color >> RED_OFFSET) & 0xFF) << SHIFT_MULTIPLIER;    // set red intensity
    TIMER_A0->CCR[2] = ((color >> GREEN_OFFSET) & 0xFF) << SHIFT_MULTIPLIER;  // set green intensity
    TIMER_A0->CCR[3] = ((color >> BLUE_OFFSET) & 0xFF) << SHIFT_MULTIPLIER;   // set blue intensity
}
