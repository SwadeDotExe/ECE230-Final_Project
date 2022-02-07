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

    /* Setup P2.7 as Input for Sonar Echo */
    P2->DIR &= ~BIT5;
    P2->REN |=  BIT5;
    P2->OUT &= ~BIT5;
    P2->SEL0 = 0;
    P2->SEL1 = 0;
    P2->IFG  = 0;
    P2->IE  |=  BIT5;
    P2->IES &= ~BIT5;

    /* Setup Timer A0 in upmode */
    TIMER_A0->CCTL[0]= TIMER_A_CCTLN_CCIE;
    TIMER_A0->CCR[0] = 1000;
    TIMER_A0->CTL = TIMER_A_CTL_TASSEL_2 | TIMER_A_CTL_MC__UP | TIMER_A_CTL_CLR;

    /* Enable Interrupts */
    NVIC->ISER[1] = 1 << ((PORT2_IRQn) & 31);
    NVIC->ISER[0] = 1 << ((TA0_0_IRQn) & 31);

    // Initialize timer used in sysTickDelay.c
    initDelayTimer(48000000);
}

int getSonarDistance() {

    /* Setup P2.6 as Output for Sonar Trigger */
    P2->DIR |= BIT4;

    /* Generate Sonar Pulse */
    P2->OUT |= BIT4;

    /* Delay */
    delayMicroSec(20);

    /* Set Pin Low */
    P2->OUT &= ~BIT4;

    /* Reset Interrupt Flag */
    P2->IFG = 0;

    /* Rising Edge Interrupt */
    P2->IES &= ~BIT5;

    /* Delay While Reading */
    delayMilliSec(30);

    /* Return length */
    return sensor;
}

// Timer_A0 ISR
void PORT2_IRQHandler(void) {

    /* Check for Interrupt */
    if(P2->IFG & BIT5) {

        /* Rising Edge Detect */
        if(!(P2->IES & BIT5)) {

            // Clear Timer
            TIMER_A0->CTL |= TIMER_A_CTL_CLR;

            // Starts Counter at 0
            sonarTime = 0;

            // Set Falling Edge
            P2->IES |=  BIT5;
        }

        /* Falling Edge Detect */
        else {

            // Calculate distance from length
            sensor = (long) sonarTime*1000 + (long) TIMER_A0->R;

            // Set Falling Edge
            P2->IES &=  ~BIT5;
        }

        /* Clear Interrupt */
        P2->IFG &= ~BIT5;
    }
}

// Interrupt for Sonar Clock (for counting length of response)
void TA0_0_IRQHandler(void) {

    /* Add to Sonar Counter */
    sonarTime++;

    /* Update Register */
    TIMER_A0->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
}

