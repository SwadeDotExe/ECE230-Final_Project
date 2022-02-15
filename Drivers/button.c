
/*
 * switchDriver.c
 *
 *  Created on: Jan 2, 2022
 *      Author: swade
 */

#include "msp.h"
#include "button.h"
#include <stdint.h>
#include <stdbool.h>


void SW_init(void) {

    // Switch Setup
    P1->SEL0 &= ~BIT5;
    P1->SEL1 &= ~BIT5;
    P1->DIR  &= ~BIT5;
    P1->REN  |= BIT5;
    P1->OUT  |= BIT5;

    // LED Setup
    P1->SEL0 &= ~BIT0;                      // Set LED1 pin to GPIO function
    P1->SEL1 &= ~BIT0;
    P1->OUT &= ~BIT0;                       // Clear LED1 to start
    P1->DIR |= BIT0;                        // Set P1.0/LED1 to output



}

bool checkSW(int SwitchNumber){

    // Switch 1
    if(P1->IN & BIT5 && SwitchNumber == 1) {
//        P1->OUT |= BIT0;
        return true;
    }
    else {
//        P1->OUT &= ~BIT0;
    }
    return false;

}
