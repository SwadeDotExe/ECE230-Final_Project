
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

    // DONE set LED2 pins as GPIO outputs
    P1->SEL0 &= ~18;
    P1->SEL1 &= ~18;
    P1->DIR  &= ~18;
    P1->REN  |= 18;
    P1->OUT  |= 18;
    // DONE set LED2 outputs to LOW

}

bool checkSW(int SwitchNumber){

    // Switch 1
    if(P1->IN == 252 && SwitchNumber == 1) {
        return true;
    }

    // Switch 2
    if(P1->IN == 238 && SwitchNumber == 2 || P1->IN == 230 && SwitchNumber == 2) {
        return true;
    }
    return false;
}
