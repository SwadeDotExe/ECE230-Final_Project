/*! \file */
/*!
 * stepperMotor.c
 *
 * Description: Stepper motor ULN2003 driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A3 and P2.7, P2.6, P2.5, P2.4
 *
 *  Created on: 1/22/2022
 *      Author: Swade Cirata
 */

#include "stepperMotor.h"
#include "msp.h"
#include "csHFXT.h"

/* Global Variables  */
// Fill in array with 4-bit binary sequence for wave drive (1-phase full step)
const uint8_t stepperSequence[STEP_SEQ_CNT] = {0b1100, 0b0110, 0b0011, 0b1001};
uint16_t stepPeriod = INIT_PERIOD;
uint8_t currentStep = 0;
uint32_t conversion;


void initStepperMotor(void) {
    // set stepper port pins as GPIO outputs
    STEPPER_PORT->SEL0 = (STEPPER_PORT->SEL0) & ~STEPPER_MASK;
    STEPPER_PORT->SEL1 = (STEPPER_PORT->SEL1) & ~STEPPER_MASK;
    STEPPER_PORT->DIR = (STEPPER_PORT->DIR) | STEPPER_MASK;

    // initialize stepper outputs to LOW
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT) & ~STEPPER_MASK;

    /* Configure Timer_A3 and CCR0 */
    // Set period of Timer_A3 in CCR0 register for Up Mode
    TIMER_A3->CCR[0] = INIT_PERIOD;
    // Configure CCR0 for Compare mode with interrupt enabled (no output mode - 0)
    TIMER_A3->CCTL[0] = 0x0010;
    // Configure Timer_A3 in Stop Mode, with source SMCLK, prescale 24:1, and
    //  interrupt disabled  -  tick rate will be 2MHz (for SMCLK = 48MHz)
    TIMER_A3->CTL = 0x280;
    TIMER_A3->EX0 = 0x5;

    /* Configure global interrupts and NVIC */
    // Enable TA3CCR0 compare interrupt by setting IRQ bit in NVIC ISER0 register
    NVIC->ISER[0] |= 0b100000000000000;
    __enable_irq();                             // Enable global interrupt

}

void enableStepperMotor(void) {
    // Configure Timer_A3 in Up Mode (leaving remaining configuration unchanged)
    TIMER_A3->CTL |= 0b0000000000010000;
}

void disableStepperMotor(void) {
    // Configure Timer_A3 in Stop Mode (leaving remaining configuration unchanged)
    TIMER_A3->CTL ^= 0b0000000000010000;
}

void stepClockwise(void) {
    currentStep = (currentStep + 1) % STEP_SEQ_CNT;  // increment to next step position
    //  do this as a single assignment to avoid transient changes on driver signals
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT & 0x0F) + (stepperSequence[currentStep] << 4);
}

void stepCounterClockwise(void) {
    currentStep = (currentStep - 1) & 0x03;  // decrement to previous step position (counter-clockwise)
    //  update output port for current step pattern
    //  do this as a single assignment to avoid transient changes on driver signals
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT & 0x0F) + (stepperSequence[currentStep] << 4);
}

void setSpeed(uint16_t speed) {
    conversion = (2000000) / ((speed / 440 + 1)/60.0 * 2048);     // Convert to clock cycles (clock: 2MHz)
    TIMER_A3->CCR[0] = conversion;                  // Update register
}

// Timer A3 CCR0 interrupt service routine
void TA3_0_IRQHandler(void)
{
    /* Not necessary to check which flag is set because only one IRQ
     *  mapped to this interrupt vector     */
    stepClockwise();
    // Clear timer compare flag in TA3CCTL0
    TIMER_A3->CCTL[0] &= ~0x1;

}
