/*! \file */
/******************************************************************************
 * Final Project - Transmitter
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
#include "Drivers/Joystick.h"
#include "Drivers/Steering.h"

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

/* Processed Readings from parseSensors() */
bool isNegative = false;
char sensorThirdDec;
char sensorSecondDec;
char sensorFirstDec;
char sensorFirstNum;

/* Message Variables */
char tempResults[12];
char messageSent[21];

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
    initCarLEDs(false);
    initJoystick();

    int i = 0;
    int z = 0;

    // Enable global interrupt
    __enable_irq();

    while(1)
    {
        /* Send Message to Car */
        sendMessage();

        /* Update debug LEDs to match car */
        updateDebugLEDs();

        /* Delay before next message */
        for(z = 0; z < 75000; z++);
    }
}

void readJoystick(void) {
    //       parse and save a (signed) 4 digit value to
    //       rightWheelSpeed and leftWheelSpeed variables
    leftWheelSpeed = mapStickY((readJoystickY() - 2000)/16);
    rightWheelSpeed = leftWheelSpeed + mapStickXR(readJoystickX()/16);
    leftWheelSpeed += mapStickXL(readJoystickX()/16);

}

void updateDebugLEDs(void) {

        headlightsToggle(headLightState);

        headlightsToggle(brakeLightState);

        turnSignalToggle(leftTurnSignalState, false);

        turnSignalToggle(rightTurnSignalState, true);
}

void parseSensor(int16_t sensorReading) {

    /* Check for negative before regex */
    if(sensorReading < 0) {
        // Set negative flag
        isNegative = true;
        // Convert to positive
        sensorReading *= -1;
    }
    else {
        isNegative = false;
    }

    /* Do regex on raw reading */
    sensorReading  /= 4;
    sensorThirdDec      = (sensorReading % 10) + '0';          // Find 3rd Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorSecondDec     = (sensorReading % 10) + '0';          // Find 2nd Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorFirstDec      = (sensorReading % 10) + '0';          // Find 1st Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorFirstNum      = (sensorReading) + '0';               // Find whole number
}

void createMessage(void) {
    /* Right Wheel Speed */
    parseSensor(rightWheelSpeed);
    tempResults[0] = sensorFirstNum;
    tempResults[1] = sensorFirstDec;
    tempResults[2] = sensorSecondDec;
    tempResults[3] = sensorThirdDec;

    /* Left Wheel Speed */
    parseSensor(leftWheelSpeed);
    tempResults[4] = sensorFirstNum;
    tempResults[5] = sensorFirstDec;
    tempResults[6] = sensorSecondDec;
    tempResults[7] = sensorThirdDec;

    /* Head Lights */
    if(checkSW(1)) {    // Button Pressed

       /* Toggle Headlight State */
       headLightState != headLightState;

       /* Parse Message */
       if (headLightState) {
           tempResults[8] = '1';    // Headlights On
       }
       else {
           tempResults[8] = '0';    // Headlights Off
       }
    }

    /* Brake Lights */
    if(rightWheelSpeed < 0 && leftWheelSpeed < 0) {     // If wheel speeds are less than zero
       tempResults[9] = '1';
       brakeLightState = true;
    }
    else {
       tempResults[9] = '0';
       brakeLightState = false;
    }

    /* Left Turn Signal */
    if(rightWheelSpeed > leftWheelSpeed) {
       tempResults[10] = '1';
       leftTurnSignalState = true;
    }
    else {
       tempResults[10] = '0';
       leftTurnSignalState = false;
    }

    /* Right Turn Signal */
    if(rightWheelSpeed < leftWheelSpeed) {
       tempResults[11] = '1';
       rightTurnSignalState = true;
    }
    else {
       tempResults[11] = '0';
       rightTurnSignalState = false;
    }
}

void sendMessage(void) {

    /* Variables */
    int i;                      // Transmit delay
    int a;                      // Message char loop

    /* Gather sensor data */
    readJoystick();

    /* Parse Sensors and create message */
    createMessage();

    /* Turn LED1 on */
    P1->OUT |= BIT0;

    /* Receive String: leftsteering,rightsteering,headlights,brakelights,leftturnsig,rightturnsig, */
    /*             <   xxxx        ,xxxx         ,x         ,x          ,x          ,x           > */

    /* Create Message */
    snprintf(messageSent, sizeof messageSent, "<%c%c%c%c,%c%c%c%c,%c,%c,%c,%c,>",
            tempResults[0],  tempResults[1],  tempResults[2],  tempResults[3],
            tempResults[4],  tempResults[5],  tempResults[6],  tempResults[7],
            tempResults[8],
            tempResults[9],
            tempResults[10],
            tempResults[11]);

    /* Transmit Message */
    for (a = 0; a <= strlen(messageSent); a++) {

       // Send next character of message
       EUSCI_A2->TXBUF = messageSent[a];

       for (i = 1000; i > 0; i--);        // lazy delay
    }

    /* Turn LED1 off */
    P1->OUT &= ~BIT0;
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
            updateLCD();
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

