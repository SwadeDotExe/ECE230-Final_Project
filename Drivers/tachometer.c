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

void initTachometer() {

    // Tachometer
    P5->SEL0 |= BIT4;
    P5->SEL1 |= BIT4;
    P5->DIR &= ~BIT4;

    // Shunt Output
    P5->SEL0 |= BIT2;
    P5->SEL1 |= BIT2;
    P5->DIR &= ~BIT2;

    // Volt Meter
    P5->SEL0 |= BIT1;
    P5->SEL1 |= BIT1;
    P5->DIR &= ~BIT1;

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
                    | ADC14_CTL0_CONSEQ_1       // Single-channel, single-conversion mode
                    | ADC14_CTL0_ON             // ADC14 on
                    | ADC14_CTL0_MSC;

    ADC14->CTL1 = ADC14_CTL1_RES__12BIT         // 12-bit conversion results
            | (0x1 << ADC14_CTL1_CSTARTADD_OFS); // ADC14MEM1 - conversion start address

    // Configure ADC14MCTL1 as storage register for result
    //       Single-ended mode with Vref+ = Vcc and Vref- = Vss,
    //       Input channel - A1, and comparator window disabled
    ADC14->MCTL[1] = 0x1;         // P5.4 (Tachometer)
    ADC14->MCTL[2] = 0x3;         // P5.2 (Shunt Current)
    ADC14->MCTL[3] = 0b10000100;  // P5.1 (System Voltage) + stop bit

    // Enable ADC conversion complete interrupt for ADC14MEM1
    ADC14->IER0 = 0b10;

    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = (1 << ADC14_IRQn);

    // Enable and start sampling/conversion by ADC
    ADC14->CTL0 |= 0b11;

}
