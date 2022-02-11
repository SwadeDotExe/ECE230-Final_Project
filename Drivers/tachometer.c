/*
 * tachometer.c
 *
 *  Created on: Feb 10, 2022
 *      Author: swade
 */

#include "msp.h"
#include "tachometer.h"
#include <stdint.h>
#include <stdbool.h>
static volatile uint16_t curADCResult;

void initTachometer() {
    // GPIO Setup
    P1->SEL0 &= ~BIT0;                      // Set LED1 pin to GPIO function
    P1->SEL1 &= ~BIT0;
    P1->OUT &= ~BIT0;                       // Clear LED1 to start
    P1->DIR |= BIT0;                        // Set P1.0/LED1 to output

    // DONE Configure P5.4 for ADC (tertiary module function)
    P5->SEL0 |= BIT4;      // set P1.1 for GPIO
    P5->SEL1 |= BIT4;
    P5->DIR &= ~BIT4;       // set P1.1 as input

    /* Configure ADC (CTL0 and CTL1) registers for:
     *      clock source - default MODCLK, clock prescale 1:1,
     *      sample input signal (SHI) source - software controlled (ADC14SC),
     *      Pulse Sample mode with sampling period of 16 ADC14CLK cycles,
     *      Single-channel, single-conversion mode, 12-bit resolution,
     *      ADC14 conversion start address ADC14MEM1, and Low-power mode
     */
    ADC14->CTL0 = ADC14_CTL0_SHP                // Pulse Sample Mode
                    | ADC14_CTL0_SHT0__16       // 16 cycle sample-and-hold time (for ADC14MEM1)
                    | ADC14_CTL0_PDIV__1        // Predivide by 1
                    | ADC14_CTL0_DIV__1         // /1 clock divider
                    | ADC14_CTL0_SHS_0          // ADC14SC bit sample-and-hold source select
                    | ADC14_CTL0_SSEL__MODCLK   // clock source select MODCLK
                    | ADC14_CTL0_CONSEQ_0       // Single-channel, single-conversion mode
                    | ADC14_CTL0_ON;            // ADC14 on

    ADC14->CTL1 = ADC14_CTL1_RES__12BIT         // 12-bit conversion results
            | (0x1 << ADC14_CTL1_CSTARTADD_OFS) // ADC14MEM1 - conversion start address
            | ADC14_CTL1_PWRMD_2;               // Low-power mode

    // DONE Configure ADC14MCTL1 as storage register for result
    //          Single-ended mode with Vref+ = Vcc and Vref- = Vss,
    //          Input channel - A1, and comparator window disabled
    ADC14->MCTL[1] = 0x1;

    // DONE Enable ADC conversion complete interrupt for ADC14MEM1
    ADC14->IER0 = 0x2;

    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = (1 << ADC14_IRQn);

    // Enable and start sampling/conversion by ADC
    ADC14->CTL0 |= 0b11;

}

// ADC14 interrupt service routine
void ADC14_IRQHandler(void) {
    // Check if interrupt triggered by ADC14MEM1 conversion value loaded
    //  Not necessary for this example since only one ADC channel used
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG1) {
        curADCResult = ADC14->MEM[1];
        if (curADCResult >= 0x7FF)              // ADC12MEM1 = A1 > 0.5AVcc?
          P1->OUT |= BIT0;                      // Turn LED1 on
        else
          P1->OUT &= ~BIT0;                     // Turn LED1 off
        // Restart sampling/conversion by ADC
        ADC14->CTL0 |= 0b11;
    }
}
