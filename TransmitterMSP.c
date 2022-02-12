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
///* State Data */
//bool headLightState = false;
//bool brakeLightState = false;
//bool leftTurnSignalState = false;
//bool rightTurnSignalState = false;
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
//        sonarSensor   = (recievedMessage[0] - '0') * 1000 +
//                        (recievedMessage[1] - '0') * 100  +
//                        (recievedMessage[2] - '0') * 10   +
//                        (recievedMessage[3] - '0') * 1;
//
//        // Set LCD cursor to first line
//        commandInstruction(CLEAR_DISPLAY_MASK);
//
//        printChar(recievedMessage[0]);
//        printChar(recievedMessage[1]);
//        printChar(recievedMessage[2]);
//        printChar(recievedMessage[3]);
//        printChar('|');
//
//        // Gyro Sensor
//        accelSensor   = (recievedMessage[5] - '0') * 1000 +
//                        (recievedMessage[6] - '0') * 100  +
//                        (recievedMessage[7] - '0') * 10   +
//                        (recievedMessage[8] - '0') * 1;
//
//        printChar(recievedMessage[5]);
//        printChar(recievedMessage[6]);
//        printChar(recievedMessage[7]);
//        printChar(recievedMessage[8]);
//        printChar('|');
//
//        // Current Sensor
//        currentSensor = (recievedMessage[10] - '0') * 1000 +
//                        (recievedMessage[11] - '0') * 100  +
//                        (recievedMessage[12] - '0') * 10   +
//                        (recievedMessage[13] - '0') * 1;
//
//        printChar(recievedMessage[10]);
//        printChar(recievedMessage[11]);
//        printChar(recievedMessage[12]);
//        printChar(recievedMessage[13]);
//        printChar(' ');
//        printChar(' ');
//
//        // Set LCD cursor to second line
//        commandInstruction(DISPLAY_CTRL_MASK | 0b0010100000);
//
//        // Voltage Sensor
//        voltageSensor = (recievedMessage[15] - '0') * 1000 +
//                        (recievedMessage[16] - '0') * 100  +
//                        (recievedMessage[17] - '0') * 10   +
//                        (recievedMessage[18] - '0') * 1;
//
//        printChar(recievedMessage[15]);
//        printChar(recievedMessage[16]);
//        printChar(recievedMessage[17]);
//        printChar(recievedMessage[18]);
//        printChar('|');
//
//        // Tachometer Sensor
//        tachoSensor   = (recievedMessage[20] - '0') * 1000 +
//                        (recievedMessage[21] - '0') * 100  +
//                        (recievedMessage[22] - '0') * 10   +
//                        (recievedMessage[23] - '0') * 1;
//
//        printChar(recievedMessage[20]);
//        printChar(recievedMessage[21]);
//        printChar(recievedMessage[22]);
//        printChar(recievedMessage[23]);
//
//
//            /* Variables */
//            int i;
//            int a;
////            char tempResults[20];
//            char messageSent[34];
//
//            /* Create Message */
////            snprintf(messageSent, sizeof messageSent, "<%c%c%c%c,%c%c%c%c,%c%c%c%c,%c%c%c%c,%c%c%c%c>\r\n",
////                     tempResults[0],  tempResults[1],  tempResults[2],  tempResults[3],
////                     tempResults[4],  tempResults[5],  tempResults[6],  tempResults[7],
////                     tempResults[8],  tempResults[9],  tempResults[10], tempResults[11],
////                     tempResults[12], tempResults[13], tempResults[14], tempResults[15],
////                     tempResults[16], tempResults[17], tempResults[18], tempResults[19]);
//
//                /* Recieve String: leftsteering,rightsteering,headlights,brakelights,leftturnsig,rightturnsig, */
//                /*             <   xxxx        ,xxxx         ,x         ,x          ,x          ,x           > */
//
//            snprintf(messageSent, sizeof messageSent, "<%c%c%c%c,%c%c%c%c,%c,%c,%c,%c>\r\n",
//                     '6', '9', '6', '7',
//                     '1', '2', '3', '4',
//                     '1',
//                     '1',
//                     '1',
//                     '1');
//
//            /* Transmit Message */
//            for (a = 0; a < strlen(messageSent); a++) {
//
//                // Send next character of message
//                //  Note that writing to TX buffer clears the flag
//                EUSCI_A2->TXBUF = messageSent[a];
//
//                for (i = 200; i > 0; i--);        // lazy delay
//            }
//
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
