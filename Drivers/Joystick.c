/*! \file */
/*!
 * Joystick.c
 *
 * Description: Joystick driver using two ADC modules for MSP432P4111 Launchpad.
 *              Assumes MODCLK configured for 12MHz.
 *              Uses ADC14 and P5.4-5 (ADC14MEM0-1)
 *
 *  Created on:
 *      Author:Bryce Bejlovec
 */

#include <Joystick.h>
#include "msp.h"
#include <stdint.h>

static volatile uint16_t XResult;
static volatile uint16_t YResult;

void initJoystick(){
    //  Configure P5.4/5.5 for ADC (tertiary module function)
        P5->SEL0 |= BIT4 | BIT5;
        P5->SEL1 |= BIT4 | BIT5;
        P5->DIR &= ~(BIT4 | BIT5);

        /* Configure ADC (CTL0 and CTL1) registers for:
         *      clock source - default MODCLK, clock prescale 1:1,
         *      sample input signal (SHI) source - software controlled (ADC14SC),
         *      Pulse Sample mode with sampling period of 16 ADC14CLK cycles,
         *      Multi-channel, continuous-conversion mode, 12-bit resolution,
         *      ADC14 conversion start address ADC14MEM1, and Low-power mode
         */
        ADC14->CTL0 = ADC14_CTL0_SHP                // Pulse Sample Mode
                        | ADC14_CTL0_SHT0__16       // 16 cycle sample-and-hold time (for ADC14MEM1)
                        | ADC14_CTL0_PDIV__1        // Predivide by 1
                        | ADC14_CTL0_DIV__1         // /1 clock divider
                        | ADC14_CTL0_SHS_0          // ADC14SC bit sample-and-hold source select
                        | ADC14_CTL0_SSEL__MODCLK   // clock source select MODCLK
                        | ADC14_CTL0_CONSEQ_3       // Dual-channel, sequence-conversion mode
                        | BIT7                      // Enables multi-sample conversion
                        | ADC14_CTL0_ON;            // ADC14 on

        ADC14->CTL1 = ADC14_CTL1_RES__12BIT         // 12-bit conversion results
                | ADC14_CTL1_CSTARTADD_OFS // ADC14MEM0 - conversion start address
                | BIT3                          // ADC14 returns a signed 12-bit value (-2048-2047)
                | ADC14_CTL1_PWRMD_2;               // Low-power mode

        // Configure ADC14MCTL0/1 as storage register for result
        //          Single-ended mode with Vref+ = Vcc and Vref- = Vss,
        //          Input channels - A0/1, and comparator window disabled
        ADC14->MCTL[0] |= 0b0000;
        ADC14->MCTL[1] |= 0b0001;

        // Enable ADC conversion complete interrupt for ADC14MEM1
//        ADC14->IER0 = 0b10;

        // Enable ADC interrupt in NVIC module
//        NVIC->ISER[0] = (1 << ADC14_IRQn);

        // Enable and start sampling/conversion by ADC
        ADC14->CTL0 |= 0b11;
}

int16_t readJoystickX(){
    return ADC14->MEM[0];
}

int16_t readJoystickY(){
    return ADC14->MEM[1];
}

//void ADC14_IRQHandler(void) {
//
//}
