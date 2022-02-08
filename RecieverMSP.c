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
//#include "Drivers/sysTickDelays.h"
//
///* Defines */
//#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT
//#define NUM_OF_REC_BYTES        6           // number of bytes to receive from sensor read
//#define GY521_ADDRESS           0x68        // I2C address of GY-521 sensor
//#define ACCEL_BASE_ADDR         0x3B        // base address of accelerometer data registers
//#define PWR_MGMT_ADDR           0x6B        // address of power management register
//
///* Raw Storage for Gyro Sensor */
//uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
//uint8_t RXDataPointer, TXDataPointer;
//
///* Variables for Serial Input */
//int inputCount = 0;
//char messageSent[48];
//bool messageComplete = false;
//
///* Processed Readings from Gyro Sensor */
//volatile int16_t accel_x, accel_y, accel_z;
//bool isNegative = false;
//char sensorThirdDec;
//char sensorSecondDec;
//char sensorFirstDec;
//char sensorFirstNum;
//
///* System Start Flag */
//bool hasStarted = false;
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
//    setupSerial();
//    configLCD(CLK_FREQUENCY);
//    initLCD();
//
//    int i = 0;
//
//    // Enable eUSCIB0 interrupt in NVIC module
//    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);
//
//      // Enable eUSCIA0 interrupt in NVIC module
//      NVIC->ISER[0] = (1 << EUSCIA0_IRQn );
//
//      // Enable global interrupt
//      __enable_irq();
//
//    while(1)
//    {
//        while(!messageComplete);
//        printToLCD();
//
//    }
//}
//
//void printToLCD() {
//
//    parseSensors(gyroReading, gyroSelection);
//
//    // Begin writing characters on first line
//    printChar('A');
//    printChar('c');
//    printChar('c');
//    printChar('e');
//    printChar('l');
//    printChar('e');
//    printChar('r');
//    printChar('o');
//    printChar('m');
//    printChar('e');
//    printChar('t');
//    printChar('e');
//    printChar('r');
//    printChar(' ');
//    printChar('8');
//    printChar('g');
//
//    // Set LCD cursor to second line
//    commandInstruction(DISPLAY_CTRL_MASK | 0b0010100000);
//
//    // Check which gyro is selected and print correct letter
//    switch(gyroSelection) {
//        case 'X' :
//            printChar('X');
//            break;
//        case 'Y' :
//            printChar('Y');
//            break;
//        case 'Z' :
//            printChar('Z');
//            break;
//        default:
//            printChar('?');
//    }
//
//    // Print some more
//    printChar(':');
//    printChar(' ');
//
//    // Check for negative sign
//    if(isNegative) {
//        printChar('-');
//    }
//    else {
//        printChar(' ');
//    }
//
//    /* Print Actual Reading */
//    printChar(firstNum);
//    printChar('.');
//    printChar(firstDec);
//    printChar(secondDec);
//    printChar(thirdDec);
//
//    // Print suffix
//    printChar(' ');
//    printChar('g');
//    printChar(' ');
//    printChar(' ');
//    printChar(' ');
//    printChar(' ');
//    printChar(' ');
//}
//
//// UART (Bluetooth) Interrupt Handler
//void EUSCIA0_IRQHandler(void)
//{
//    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
//    {
//        // Check if the TX buffer is empty first
//        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));
//
//        // New message in progress
//        messageComplete = false;
//
//        // Saves message to array
//        messageSent[inputCount] = EUSCI_A0->RXBUF
//
//        // Increases message index
//        inputCount++;
//
//        // Checks for beginning of next message
//        if(inputCount >= 47) {
//            inputCount = 0;
//            messageComplete = true;
//        }
//
//    }
//}
//
//void setupSerial() {
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
//    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
//    P1->SEL1 &= ~(BIT2 | BIT3);
//
//    /* Configure UART
//     *  Asynchronous UART mode, 8O1 (8-bit data, odd parity, 1 stop bit),
//     *  LSB first, SMCLK clock source
//     */
//    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
//    // DONE complete configuration of UART in eUSCI_A0 control register
//    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | 0b1100000010000000;
//
//    /* Baud Rate calculation
//     * Refer to Section 24.3.10 of Technical Reference manual
//     * BRCLK = 12000000, Baud rate = 38400
//     *
//     * DONE calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
//     *          values used in next two TODOs
//     */
//    // DONE set clock prescaler in eUSCI_A0 baud rate control register
//    EUSCI_A0->BRW = 19;
//    // DONE configure baud clock modulation in eUSCI_A0 modulation control register
//    EUSCI_A0->MCTLW = 0b110010110000001;
//
//    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
//    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
//    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt
//}
//
