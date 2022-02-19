/*
 * carLEDs.c
 *
 *  Created on: Feb 12, 2022
 *      Author: swade
 */



#include "msp.h"
#include "carLEDs.h"
#include <stdint.h>
#include <stdbool.h>


void initCarLEDs(bool debug) {

    /* Debug = False -->   Car Lights */
    /* Debug = True  --> Debug Lights */

    // Debug LED Setup
    P1->SEL0 &= ~BIT0;                      // Set LED1 pin to GPIO function
    P1->SEL1 &= ~BIT0;
    P1->OUT  &= ~BIT0;                       // Clear LED1 to start
    P1->DIR  |=  BIT0;                        // Set P1.0/LED1 to output

    // Red LEDs
    P4->SEL0 &= ~BIT4;                      // Set LED1 pin to GPIO function
    P4->SEL1 &= ~BIT4;
    P4->OUT &= ~BIT4;                       // Clear LED1 to start
    P4->DIR |= BIT4;                        // Set P1.0/LED1 to output

    // White LEDs
    P2->SEL0 &= ~BIT3;                      // Set LED1 pin to GPIO function
    P2->SEL1 &= ~BIT3;
    P2->OUT &= ~BIT3;                       // Clear LED1 to start
    P2->DIR |= BIT3;                        // Set P1.0/LED1 to output

    // Left Orange LEDs
    P4->SEL0 &= ~BIT5;                      // Set LED1 pin to GPIO function
    P4->SEL1 &= ~BIT5;
    P4->OUT &= ~BIT5;                       // Clear LED1 to start
    P4->DIR |= BIT5;                        // Set P1.0/LED1 to output

    // Right Orange LEDs
    P4->SEL0 &= ~BIT3;                      // Set LED1 pin to GPIO function
    P4->SEL1 &= ~BIT3;
    P4->OUT &= ~BIT3;                       // Clear LED1 to start
    P4->DIR |= BIT3;                        // Set P1.0/LED1 to output

    // Relay Setup
    P4->SEL0 &= ~BIT7;
    P4->SEL1 &= ~BIT7;
    P4->DIR  &= ~BIT7;
    P4->REN  |=  BIT7;
    P4->OUT  |=  BIT7;      // off
}



void headlightsToggle(bool status) {
    if(status) {
        P4->OUT |= BIT2;
        P2->OUT |= BIT3;
        P4->OUT |= BIT7;      // on
    }
    else {
        P4->OUT &= ~BIT2;
        P2->OUT &= ~BIT3;
        P4->OUT &= ~BIT7;
    }
}

void brakelightsToggle(bool status) {
    if(status) {
        P4->OUT |= BIT4;
    }
    else {
        P4->OUT &= ~BIT4;
    }
}

void turnSignalToggle(bool status, bool side) {
    // false = left, true = right
    if (!side && status) {
        P4->OUT |= BIT3;
    }
    else if (!side && !status) {
        P4->OUT &= ~BIT3;
    }
    else if (side && status) {
        P4->OUT |= BIT5;
    }
    else if (side && !status) {
        P4->OUT &= ~BIT5;
    }
}




