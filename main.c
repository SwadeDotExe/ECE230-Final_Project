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
 *
 *                  MSP432P411x
 *             -------------------
 *         /|\|                   |
 *          | |                   |
 *          --|RST                |
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
 *            |      P1.3/UCA0TXD |----> PC (echo)
 *            |      P1.2/UCA0RXD |<---- PC
 *            |                   |
 *             -------------------
 *
*******************************************************************************/
#include "msp.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/csHFXT.h"
#include "Drivers/lcd.h"
#include "Drivers/sysTickDelays.h"
#include "Drivers/button.h"

/* Defines */
#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT

/* Variables for Serial Input */
char inputChar;
bool recievingData = false;
int  recievedIndex = 0;
char recievedMessage[48];
bool messageDone = false;

/* Variables to hold Sensor Data */
int sonarSensor;
int accelSensor;
int currentSensor;
int voltageSensor;
int tachoSensor;

/* Wheel Speed Data */
int16_t rightWheelSpeed;
int16_t leftWheelSpeed;

/* State Data */
bool headLightState = false;
bool brakeLightState = false;
bool leftTurnSignalState = false;
bool rightTurnSignalState = false;
bool headLightButtPress = false;

/**
 * main.c
 */
void main(void)
{
    /* Stop Watchdog timer */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /* Configure Peripherals */
    SW_init();
    configHFXT();
    initDelayTimer(CLK_FREQUENCY);
    setupBluetooth();
    configLCD(CLK_FREQUENCY);
    initLCD();
    initCarLEDs();

    int i = 0;

//    // Enable eUSCIB0 interrupt in NVIC module
//    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);
//
//    // Enable eUSCIA0 interrupt in NVIC module
//    NVIC->ISER[0] = (1 << EUSCIA0_IRQn );

    // Enable global interrupt
    __enable_irq();

    while(1)
    {
        // Wait for complete message
        while(!messageDone) {
            headLightButtPress = checkSW(1);
        }

        updateLCD();

        sendMessage();

        updateDebugLEDs();

        headLightButtPress = false;
    }
}

void updateDebugLEDs(void) {

        headlightsToggle(headLightState);

        headlightsToggle(brakeLightState);

        turnSignalToggle(leftTurnSignalState, false);

        turnSignalToggle(rightTurnSignalState, true);
}

void sendMessage(void) {
    /* Variables */
       int i;
       int a;
       char tempResults[12];
       char messageSent[20];

       /* Right Wheel Speed */
       tempResults[0] = '1';
       tempResults[1] = '2';
       tempResults[2] = '3';
       tempResults[3] = '4';

       /* Left Wheel Speed */
       tempResults[4] = '5';
       tempResults[5] = '6';
       tempResults[6] = '7';
       tempResults[7] = '8';

       /* Head Lights */
       if(headLightButtPress) {    // Button Pressed
           tempResults[8] = '1';
       }
       else {
           tempResults[8] = '0';   // Button Not Pressed
       }

       /* Brake Lights */
       if(rightWheelSpeed < 0 && leftWheelSpeed < 0) {     // If wheel speeds are less than zero
           tempResults[9] = '1';
       }
       else {
           tempResults[9] = '0';
       }

       /* Left Turn Signal */
       if(rightWheelSpeed > leftWheelSpeed) {     // If wheel speeds are less than zero
           tempResults[10] = '1';
       }
       else {
           tempResults[10] = '0';
       }

       /* Right Turn Signal */
       if(rightWheelSpeed < leftWheelSpeed) {     // If wheel speeds are less than zero
           tempResults[11] = '1';
       }
       else {
           tempResults[11] = '0';
       }

       /* Receive String: leftsteering,rightsteering,headlights,brakelights,leftturnsig,rightturnsig, */
       /*             <   xxxx        ,xxxx         ,x         ,x          ,x          ,x           > */


       /* Create Message */
       snprintf(messageSent, sizeof messageSent, "<%c%c%c%c,%c%c%c%c,%c,%c,%c,%c>\r\n",
                tempResults[0],  tempResults[1],  tempResults[2],  tempResults[3],
                tempResults[4],  tempResults[5],  tempResults[6],  tempResults[7],
                tempResults[8],
                tempResults[9],
                tempResults[10],
                tempResults[11]);

       /* Transmit Message */
       for (a = 0; a <= strlen(messageSent); a++) {

           // Send next character of message
           //  Note that writing to TX buffer clears the flag
           EUSCI_A2->TXBUF = messageSent[a];

           for (i = 200; i > 0; i--);        // lazy delay
       }
}

void updateLCD() {
    // Sonar Sensor
    sonarSensor   = (recievedMessage[0] - '0') * 1000 +
                    (recievedMessage[1] - '0') * 100  +
                    (recievedMessage[2] - '0') * 10   +
                    (recievedMessage[3] - '0') * 1;

    // Set LCD cursor to first line
    commandInstruction(CLEAR_DISPLAY_MASK);

    printChar(recievedMessage[0]);
    printChar(recievedMessage[1]);
    printChar(recievedMessage[2]);
    printChar(recievedMessage[3]);
    printChar('|');

    // Gyro Sensor
    accelSensor   = (recievedMessage[5] - '0') * 1000 +
                    (recievedMessage[6] - '0') * 100  +
                    (recievedMessage[7] - '0') * 10   +
                    (recievedMessage[8] - '0') * 1;

    printChar(recievedMessage[5]);
    printChar(recievedMessage[6]);
    printChar(recievedMessage[7]);
    printChar(recievedMessage[8]);
    printChar('|');

    // Current Sensor
    currentSensor = (recievedMessage[10] - '0') * 1000 +
                    (recievedMessage[11] - '0') * 100  +
                    (recievedMessage[12] - '0') * 10   +
                    (recievedMessage[13] - '0') * 1;

    printChar(recievedMessage[10]);
    printChar(recievedMessage[11]);
    printChar(recievedMessage[12]);
    printChar(recievedMessage[13]);
    printChar(' ');
    printChar(' ');

    // Set LCD cursor to second line
    commandInstruction(DISPLAY_CTRL_MASK | 0b0010100000);

    // Voltage Sensor
    voltageSensor = (recievedMessage[15] - '0') * 1000 +
                    (recievedMessage[16] - '0') * 100  +
                    (recievedMessage[17] - '0') * 10   +
                    (recievedMessage[18] - '0') * 1;

    printChar(recievedMessage[15]);
    printChar(recievedMessage[16]);
    printChar(recievedMessage[17]);
    printChar(recievedMessage[18]);
    printChar('|');

    // Tachometer Sensor
    tachoSensor   = (recievedMessage[20] - '0') * 1000 +
                    (recievedMessage[21] - '0') * 100  +
                    (recievedMessage[22] - '0') * 10   +
                    (recievedMessage[23] - '0') * 1;

    printChar(recievedMessage[20]);
    printChar(recievedMessage[21]);
    printChar(recievedMessage[22]);
    printChar(recievedMessage[23]);

    // Done reading message
    messageDone = false;
}

// UART interrupt service routine (Received data)
void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));

        // Capture recieved byte
        inputChar = EUSCI_A2->RXBUF;

        // End of Transmission
        if(inputChar == '>') {
            recievingData = false;
            recievedIndex = 0;
            messageDone = true;
        }

        // Capture Data
        if(recievingData) {
            recievedMessage[recievedIndex] = inputChar;
            recievedIndex++;
        }

        // Start of Transmission
        if(inputChar == '<') {
            recievingData = true;
        }
    }
}

void setupBluetooth() {
    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Configure UART pins */
    P3->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
    P3->SEL1 &= ~(BIT2 | BIT3);

    /* Configure UART
     *  Asynchronous UART mode, 8N1 (8-bit data, no parity, 1 stop bit),
     *  LSB first, SMCLK clock source
     */
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset
            EUSCI_A_CTLW0_SSEL__SMCLK;      // Configure eUSCI clock source for SMCLK

    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 38400
     *
     * DONE calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
     *          values used in next two TODOs
     */
    // DONE set clock prescaler in EUSCI_A2 baud rate control register
    EUSCI_A2->BRW = 19;
    // DONE configure baud clock modulation in EUSCI_A2 modulation control register
    EUSCI_A2->MCTLW = 0b110010110000001;

    EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A2->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt

    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIA2_IRQn );
}

