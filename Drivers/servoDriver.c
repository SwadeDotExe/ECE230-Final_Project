/*! \file */
/*!
 * servoDriver.c
 *
 * Description: Servo motor driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A2 and P5.6 (TA2.1)
 *
 *  Created on: 1/22/2022
 *      Author: Swade Cirata
 */

#include "servoDriver.h"
#include "msp.h"
#include <stdio.h>

/* Global Variables  */
uint16_t pulseWidthTicks = SERVO_MIN_ANGLE;
float    scaledADC = 0.0;

#define SERVO_MASK                    (0x0040)
#define SERVO_PORT                    P5

void initServoMotor(void) {
    // Configure servo pin (P5.6) for primary module function (TA2.1),
    //  output, initially LOW
    SERVO_PORT->SEL0 |=  SERVO_MASK;
    SERVO_PORT->SEL1 &= ~SERVO_MASK;
    SERVO_PORT->DIR  |=  SERVO_MASK;
    SERVO_PORT->OUT  &= ~SERVO_MASK;


    /* Configure Timer_A2 and CCR1 */
    // Set period of Timer_A2 in CCR0 register for Up Mode
    TIMER_A2->CCR[0] = SERVO_TMR_PERIOD;
    // Set initial positive pulse-width of PWM in CCR1 register
    TIMER_A2->CCR[1] = SERVO_MIN_ANGLE;

    // Configure TA2CCR1 for Compare mode, Reset/Set output mode, with interrupt disabled
    TIMER_A2->CCTL[1] = 0b0000000011100100;

    // Configure Timer_A2 in Up Mode, with source SMCLK, prescale 24:1, and
    //     interrupt disabled  -  tick rate will be 2MHz (for SMCLK = 48MHz)
    // Configure Timer_A2 (requires setting control AND expansion register)
    TIMER_A2->CTL = 0b1011010000;   // Divide by 8
    TIMER_A2->EX0 = 0b010;          // Divide by 3
}

void incrementTenDegree(void) {
    // update pulse-width for <current angle> + <10 degrees>
    pulseWidthTicks += TEN_DEGREE_TICKS;
    if (pulseWidthTicks > SERVO_MAX_ANGLE) {
        pulseWidthTicks = SERVO_MIN_ANGLE;
    }
    // Update CCR1 register to set new positive pulse-width
    TIMER_A2->CCR[1] = pulseWidthTicks;
}

void setServoAngle(uint16_t angle) {

    /* Scale input ADC for servo range */
    scaledADC = angle * 0.733;

    /* Update register with converted value */
    pulseWidthTicks = (scaledADC) + SERVO_MIN_ANGLE;

    /* Update CCR1 register to set new positive pulse-width */
    TIMER_A2->CCR[1] = pulseWidthTicks;
}
