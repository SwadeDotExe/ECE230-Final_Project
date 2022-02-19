/*! \file */
/*!
 * L293D.c
 *
 * Description: L293D driver using PWM for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured for 12MHz.
 *              Uses Timer_A1 and P2.4-7 (TA1.0-4)
 *
 *  Created on:
 *      Author:Bryce Bejlovec
 */

#include "Drivers/L293D.h"
#include "msp.h"
#include <stdint.h>

void initL293D(void) {
    /* Configure RGB LED pins */
    P2->SEL0 |=   BIT4 | BIT5 | BIT6 | BIT7;         // set 4 pins as GPIO
    P2->SEL1 &= ~(BIT4 | BIT5 | BIT6 | BIT7);
    P2->OUT  &= ~(BIT4 | BIT5 | BIT6 | BIT7);       // clear output
    P2->DIR  |=   BIT4 | BIT5 | BIT6 | BIT7;          // set as output pins


    /* Configure TIMER_A0 and CCR1/2/3 */
    // Set period of TIMER_A0 in CCR0 register for Up Mode
    TIMER_A0->CCR[0] = PWM_PERIOD;
    // Set initial positive pulse-width of PWM in CCR1/2/3/4 register
    TIMER_A0->CCR[1] = 0;
    TIMER_A0->CCR[2] = 0;
    TIMER_A0->CCR[3] = 0;
    TIMER_A0->CCR[4] = 0;
    // Configure CCR1/2/3/4 for Compare mode, Reset/Set output mode, with interrupt disabled
    TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_7;       // Reset/Set output mode
    // Configure Timer_A1 in Up Mode, with source SMCLK, prescale 28:1, and
    //  interrupt disabled  -  tick rate will be (12MHz/28)
    TIMER_A0->CTL = TIMER_A_CTL_SSEL__SMCLK |   // SMCLK
            TIMER_A_CTL_ID__4 |                 // /4 divider
            TIMER_A_CTL_MC__UP |                // Up mode
            TIMER_A_CTL_CLR;                    // Clear TAR
    TIMER_A0->EX0 = TIMER_A_EX0_IDEX__7;        // /7 divider
}

void setMotorPWM(int16_t left, int16_t right) {
    // Update CCR1/2/3/4 registers to set new pulse-width for right and left
    //  speed respectively.
    //          "left" represents the rate at which the left side motors would rotate
    //          "right" represents the rate at which the right side motors would rotate
    if(left >= 0){
        TIMER_A0->CCR[3] = 0;  // disable reverse
        TIMER_A0->CCR[1] = (uint16_t)(left);    // set left speed
    }else {
        TIMER_A0->CCR[1] = 0;  // disable forward
        TIMER_A0->CCR[3] = ((uint16_t)(left*(-1)-1));    // set left speed
    }
    if(right >= 0){
        TIMER_A0->CCR[2] = 0;  // disable reverse
        TIMER_A0->CCR[4] = (uint16_t)(right);    // set right speed
    } else {
        TIMER_A0->CCR[3] = 0;  // disable forward
        TIMER_A0->CCR[2] = ((uint16_t)(right*(-1)-1));    // set right speed
    }
}
