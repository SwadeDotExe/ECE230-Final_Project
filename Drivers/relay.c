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
    P3->SEL0 &= ~BIT0;
    P3->SEL1 &= ~BIT0;
    P3->DIR  &= ~BIT0;
    P3->REN  |=  BIT0;
    P3->OUT  |=  BIT0;      // on
    P3->OUT  &= ~BIT0;      // off

}
