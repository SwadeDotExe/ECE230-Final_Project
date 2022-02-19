
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
    P5->SEL0 &= ~BIT6;
    P5->SEL1 &= ~BIT6;
    P5->DIR  &= ~BIT6;
    P5->REN  |= BIT6;
    P5->OUT  |= BIT6;

}

bool checkSW(int SwitchNumber){

    // Switch 1
    if(P5->IN & BIT6 && SwitchNumber == 1) {
        return true;
    }
    else {
        return false;
    }

}
