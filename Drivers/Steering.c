/*! \file */
/*!
 * Steering.c
 *
 * Description: Contains one function to convert a 12-bit
 * PWM value to one usable by motor PWM.
 *
 *  Created on:
 *      Author:Bryce Bejlovec
 */

#include "Drivers/Joystick.h"
#include "msp.h"
#include <stdint.h>

int32_t mapStickY(int16_t stickYValue){
    if(stickYValue > 50 || stickYValue < -50){
           return stickYValue;
        }
    return 0;
}

int32_t mapStickXL(int16_t stickXValue){
    if(stickXValue > 50){
       return stickXValue;
    }else if(stickXValue < -50){
       return stickXValue;
    }
    return 0;
}

int32_t mapStickXR(int16_t stickXValue){
    if(stickXValue < -50){
       return -1 * stickXValue;
    }else if(stickXValue > 50){
       return -1 * stickXValue;
    }
    return 0;
}
