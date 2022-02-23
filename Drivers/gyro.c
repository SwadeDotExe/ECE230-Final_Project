/*
 * gyro.c
 *
 *  Created on: Feb 11, 2022
 *      Author: swade
 */
#include "msp.h"
#include "gyro.h"
#include <stdint.h>
#include <stdbool.h>

void initGyro() {
    /* Configure UART pins */
        P1->SEL0 |= BIT6 | BIT7;                // set I2C pins as secondary function
        P1->SEL1 &= ~(BIT6 | BIT7);
        /* Configure eUSCI_B0 for I2C mode
         *  I2C master mode, synchronous, 7-bit address, SMCLK clock source,
         *  transmit mode, with automatic STOP condition generation
         */
        EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Software reset enabled
        EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset mode
                EUSCI_B_CTLW0_MODE_3 |          // I2C mode
                EUSCI_B_CTLW0_MST |             // Master mode
                EUSCI_B_CTLW0_SYNC |            // Sync mode
                EUSCI_B_CTLW0_TR |              // Transmitter mode
                EUSCI_B_CTLW0_SSEL__SMCLK;      // SMCLK

        /* I2C clock calculation
         * Refer to Section 26.3.6 of Technical Reference manual
         * BRCLK = 3MHz, I2C bit clock rate = 100kbps
        */
        // DONE configure eUSCI_B0 bit rate control for 100 kbps
        EUSCI_B0->BRW = 30;

        /* Configure I2C to communicate with GY-521 */
        EUSCI_B0->I2CSA = GY521_ADDRESS;            // I2C peripheral address
        EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Release eUSCI from reset

        /* Initialize GY-521 by writing to Power Management Register
         *
         *  format for Write operations
         *  _________________________________________________________________
         *  |       |          |                 |                  |       |
         *  | Start |  Addr  W | <Register Addr> | <Value to write> | Stop  |
         *  |_______|__________|_________________|__________________|_______|
         */
        // Ensure stop condition not pending
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
        do {
            // Send I2C start condition and address frame with W
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
            // wait for TX buffer to be ready
            while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
            // load 1st data byte into TX buffer
            EUSCI_B0->TXBUF = PWR_MGMT_ADDR;            // send register address
            // wait for ACK/NACK after address frame
            while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT);
        } while(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG);   // resend address frame if ACK not received
        // wait for TX buffer to be ready
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // load 2nd data byte into TX buffer
        EUSCI_B0->TXBUF = 0;                // write value to register
        // wait for 2nd data byte to begin transmit
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // Send I2C stop condition
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

        // Ensure stop condition got sent
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
        // ensure flags are cleared before enabling interrupts
        EUSCI_B0->IFG &= ~(EUSCI_B_IFG_TXIFG0 | EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_NACKIFG);

        EUSCI_B0->IE |= EUSCI_A_IE_RXIE |       // Enable receive interrupt
                EUSCI_A_IE_TXIE |               // Enable transmit interrupt
                EUSCI_B_IE_NACKIE;              // Enable NACK interrupt
}
