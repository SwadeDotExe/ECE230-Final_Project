///*! \file */
///******************************************************************************
// * Final Project
// *
// * Description: A RC car controlled via Bluetooth with a variety of sensors
// *
// * Authors: Swade and Bryce
// * Last-modified: 2/6/2022
// *
// *
// *                                   ___  ___
// *                                    |    |
// *                  MSP432P411x      10k  10k     GY-521
// *             -------------------    |    |    -----------
// *         /|\|      P1.6/UCB0SDA |<--|----|-->| SDA
// *          | |                   |   |        |
// *          --|RST                |   |        |
// *            |      P1.7/UCB0SCL |<--|------->| SCL
// *            |                   |             -----------
// *            |                   |
// *            |                   |          LCD
// *            |                   |       --------
// *            |              P2.7 |----->| RS
// *            |                   |      |
// *            |              P2.6 |----->| En
// *            |                   |      |
// *            |                   |  8   |
// *            |               P4  |--\-->| DB
// *            |                   |      |
// *            |                   |       --------
// *            |                   |
// *            |              PJ.2 |------
// *            |                   |     |
// *            |                   |    HFXT @ 48MHz
// *            |                   |     |
// *            |              PJ.3 |------
// *            |                   |
// *            |      P1.3/UCA0TXD |----> PC (echo)
// *            |      P1.2/UCA0RXD |<---- PC
// *            |                   |
// *             -------------------
// *
// * An external HF crystal between HFXIN & HFXOUT is required for MCLK,SMCLK
// *
// *
// * Transmit String: sonar=0000,gyro=0000,power=0000,volt=0000
// *          Length: 41
// *
//*******************************************************************************/
//#include "msp.h"
//
///* Standard Includes */
//#include <stdint.h>
//#include <stdbool.h>
//#include "Drivers/csHFXT.h"
//#include "Drivers/lcd.h"
//#include "Drivers/sysTickDelays.h"
//
///* Defines */
//#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT
//#define NUM_OF_REC_BYTES        6           // number of bytes to receive from sensor read
//
///* Variables for Serial Input */
//char inputChar;
//bool recievingData = false;
//int  recievedIndex = 0;
//char recievedMessage[48];
//bool messageDone = false;
//
///* Variables to hold Sensor Data */
//int sonarSensor;
//int accelSensor;
//int currentSensor;
//int voltageSensor;
//int tachoSensor;
//
//
///**
// * main.c
// */
//void main(void)
//{
//    /* Stop Watchdog timer */
//    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;
//
//    /* Configure Peripherals */
//    configHFXT();
//    initDelayTimer(CLK_FREQUENCY);
//    setupBluetooth();
//    configLCD(CLK_FREQUENCY);
//    initLCD();
//
//    int i = 0;
//
//    // Enable eUSCIB0 interrupt in NVIC module
//    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);
//
//    // Enable eUSCIA0 interrupt in NVIC module
//    NVIC->ISER[0] = (1 << EUSCIA0_IRQn );
//
//    // Enable eUSCIA2 interrupt in NVIC module
//    NVIC->ISER[0] = (1 << EUSCIA2_IRQn );
//
//    // Enable global interrupt
//    __enable_irq();
//
//    while(1)
//    {
//        // Wait for complete message
//        while(!messageDone);
//
//        // Sonar Sensor
//        sonarSensor = recievedMessage[1] * 1000 +
//                      recievedMessage[2] * 100  +
//                      recievedMessage[3] * 10   +
//                      recievedMessage[4] * 1;
//
//        // Done reading message
//        messageDone = false;
//    }
//}
//
//// UART interrupt service routine (Received data)
//void EUSCIA2_IRQHandler(void)
//{
//
//    /* Recieve String: sonar=0.000,gyro=0.000,power=0.000,volt=0.000,tach=0.000 */
//
//    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
//    {
//        // Check if the TX buffer is empty first
//        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
//
//        // Capture recieved byte
//        inputChar = EUSCI_A2->RXBUF;
//
//        // End of Transmission
//        if(inputChar == '>') {
//            recievingData = false;
//            recievedIndex = 0;
//            messageDone = true;
//        }
//
//        // Capture Data
//        if(recievingData) {
//            recievedMessage[recievedIndex] = inputChar;
//            recievedIndex++;
//        }
//
//        // Start of Transmission
//        if(inputChar == '<') {
//            recievingData = true;
//        }
//
//        // Echo the received character back
//        //  Note that reading RX buffer clears the flag and removes value from buffer
////        EUSCI_A2->TXBUF = EUSCI_A2->RXBUF;
//
//    }
//}
//
//void setupBluetooth() {
//    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
//    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
//    CS->CTL0 = 0;                           // Reset tuning parameters
//    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
//    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
//            CS_CTL1_SELS_3 |                // SMCLK = DCO
//            CS_CTL1_SELM_3;                 // MCLK = DCO
//    CS->KEY = 0;                            // Lock CS module from unintended accesses
//
//    /* Configure UART pins */
//    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
//    P3->SEL1 &= ~(BIT2 | BIT3);
//
//    /* Configure UART
//     *  Asynchronous UART mode, 8N1 (8-bit data, no parity, 1 stop bit),
//     *  LSB first, SMCLK clock source
//     */
//    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
//    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
//            EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK
//
//    /* Baud Rate calculation
//     * Refer to Section 24.3.10 of Technical Reference manual
//     * BRCLK = 12000000, Baud rate = 38400
//     *
//     * DONE calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
//     *          values used in next two TODOs
//     */
//    // DONE set clock prescaler in EUSCI_A2 baud rate control register
//    EUSCI_A2->BRW = 19;
//    // DONE configure baud clock modulation in EUSCI_A2 modulation control register
//    EUSCI_A2->MCTLW = 0b110010110000001;
//
//    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
//    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
//    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt
//}
//
