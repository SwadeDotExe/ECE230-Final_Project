/*! \file */
/******************************************************************************
 * Final Project
 *
 * Description: A RC car controlled via Bluetooth with a variety of sensors
 *
 * Authors: Swade and Bryce
 * Last-modified: 2/6/2022
 *
 *
 *                                   ___  ___
 *                                    |    |
 *                  MSP432P411x      10k  10k     GY-521
 *             -------------------    |    |    -----------
 *         /|\|      P1.6/UCB0SDA |<--|----|-->| SDA
 *          | |                   |   |        |
 *          --|RST                |   |        |
 *            |      P1.7/UCB0SCL |<--|------->| SCL
 *            |                   |             -----------
 *            |                   |
 *            |                   |          LCD
 *            |                   |       --------
 *            |              P2.7 |----->| RS
 *            |                   |      |
 *            |              P2.6 |----->| En
 *            |                   |      |
 *            |                   |  8   |
 *            |               P4  |--\-->| DB
 *            |                   |      |
 *            |                   |       --------
 *            |                   |
 *            |              PJ.2 |------
 *            |                   |     |
 *            |                   |    HFXT @ 48MHz
 *            |                   |     |
 *            |              PJ.3 |------
 *            |                   |
 *            |      P1.3/UCA0TXD |----> PC (echo)
 *            |      P1.2/UCA0RXD |<---- PC
 *            |                   |
 *             -------------------
 *
 * An external HF crystal between HFXIN & HFXOUT is required for MCLK,SMCLK
 *
*******************************************************************************/
#include "msp.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/csHFXT.h"
#include "Drivers/lcd.h"
#include "Drivers/sysTickDelays.h"
#include "Drivers/sonarSensor.h"


/**
 * main.c
 */
void main(void)
{
    /* Stop Watchdog timer */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;


    initalizeSonar();

    while(1)
    {
        int test = getSonarDistance();

    }
}
