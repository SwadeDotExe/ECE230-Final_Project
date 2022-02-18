/*
 * relay.c
 *
 *  Created on: Feb 15, 2022
 *      Author: swade
 */

#include "msp.h"
#include "button.h"
#include <stdint.h>
#include <stdbool.h>


void setupRelay(void) {

    // Relay Setup
    P4->SEL0 &= ~BIT6;
    P4->SEL1 &= ~BIT6;
    P4->DIR  &= ~BIT6;
    P4->REN  |=  BIT6;
    P4->OUT  |=  BIT6;      // off
//    P4->OUT  &= ~BIT6;      // on

}
