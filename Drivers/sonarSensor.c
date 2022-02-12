/*
 * sonarSensor.c
 *
 *  Created on: Feb 6, 2022
 *      Author: Swade
 */
#include "msp.h"
#include "Drivers/sysTickDelays.h"
#include <stdint.h>

int sonarTime = 0;
int distance = 0;
long sensor = 0;

void initalizeSonar() {

    /* Setup P4.1 as Input for Sonar Echo */
    P4->DIR &= ~BIT1;
    P4->REN |=  BIT1;
    P4->OUT &= ~BIT1;
    P4->SEL0 = 0;
    P4->SEL1 = 0;
    P4->IFG  = 0;
    P4->IE  |=  BIT1;
    P4->IES &= ~BIT1;

    /* Setup Timer A0 in upmode */
    TIMER_A0->CCTL[0]= TIMER_A_CCTLN_CCIE;
    TIMER_A0->CCR[0] = 1000;
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;

    /* Enable Interrupts */
    NVIC->ISER[1] = 1 << ((PORT4_IRQn) & 31);
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    // Initialize timer used in sysTickDelay.c
    initDelayTimer(48000000);
}

int getSonarDistance() {

    /* Setup P4.0 as Output for Sonar Trigger */
    P4->DIR |= BIT0;

    /* Generate Sonar Pulse */
    P4->OUT |= BIT0;

    /* Delay */
    delayMicroSec(20);

    /* Set Pin Low */
    P4->OUT &= ~BIT0;

    /* Reset Interrupt Flag */
    P4->IFG = 0;

    /* Rising Edge Interrupt */
    P4->IES &= ~BIT1;

    /* Delay While Reading */
    delayMilliSec(30);

    /* Return length */
    return sensor;
}

// Interrupt for Sonar Clock (for counting length of response)
void PORT4_IRQHandler(void) {

    /* Check for Interrupt */
    if(P4->IFG & BIT1) {

        /* Rising Edge Detect */
        if(!(P4->IES & BIT1)) {

            // Clear Timer
            TIMER_A0->CTL |= TIMER_A_CTL_CLR;

            // Starts Counter at 0
            sonarTime = 0;

            // Set Falling Edge
            P4->IES |=  BIT1;
        }

        /* Falling Edge Detect */
        else {

            // Calculate distance from length
            sensor = (long) sonarTime*1000 + (long) TIMER_A0->R;

            // Set Falling Edge
            P4->IES &= ~BIT1;
        }

        /* Clear Interrupt */
        P4->IFG &= ~BIT1;
    }
}
// Timer_A0 ISR
void TA0_0_IRQHandler(void) {

    /* Add to Sonar Counter */
    sonarTime++;

    /* Update Register */
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

