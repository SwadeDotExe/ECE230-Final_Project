/*! \file */
/******************************************************************************
 * Final Project
 *
 * Description: A RC car controlled via Bluetooth with a variety of sensors
 *
 * Authors: Swade and Bryce
 * Last-modified: 2/22/2022
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
 *            |                   |    220 Ohm       ---------
 *            |              P4.4 |----/\/\/\------>| Red LED |
 *            |                   |                  ---------
 *            |                   |
 *            |                   |    220 Ohm       -----------
 *            |              P4.2 |----/\/\/\------>| White LED |
 *            |                   |                  -----------
 *            |                   |
 *            |                   |    220 Ohm       ----------------
 *            |              P4.3 |----/\/\/\------>| Orange LED (L) |
 *            |                   |                  ----------------
 *            |                   |
 *            |                   |    220 Ohm       ----------------
 *            |              P4.5 |----/\/\/\------>| Orange LED (R) |
 *            |                   |                  ----------------
 *            |                   |
 *            |                   |
 *            |                   |
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
#include "Drivers/sysTickDelays.h"
#include "Drivers/sonarSensor.h"
#include "Drivers/tachometer.h"
#include "Drivers/gyro.h"
#include "Drivers/carLEDs.h"
#include "Drivers/relay.h"
#include "Drivers/L293D.h"

/* Delay Timer */
#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT
#define NUM_OF_REC_BYTES        6           // number of bytes to receive from sensor read

/* Raw Storage for Gyro Sensor */
uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
uint8_t RXDataPointer, TXDataPointer;
volatile int16_t accel_x, accel_y, accel_z;

/* Parsed Readings */
bool isNegative = false;
char sensorThirdDec;
char sensorSecondDec;
char sensorFirstDec;
char sensorFirstNum;

/* Sonar Sensor */
int sonarReading;

/* Variable to hold Tachometer interrupt count */
volatile uint32_t tachometerTicks = 0;
static volatile uint16_t curADCResult;
bool locked = false;
bool adcInterruptEnabled = false;

/* Variables for Serial Input */
char inputChar;
bool recievingData = false;
const int messageRecievedLength = 21;
int  recievedIndex = 0;
char recievedMessage[messageRecievedLength];
bool messageDone = false;

/* Variables to hold Recieved Data */
int16_t leftWheelSpeed;
int16_t rightWheelSpeed;
bool headLightState = false;
bool brakeLightState = false;
bool leftTurnSignalState = false;
bool rightTurnSignalState = false;
bool underLightsState = false;

/* Car State */
int carState = 0;

/* Variables for Message Transmission */
char tempResults[25];
char messageSent[500];

/**
 * main.c
 */
void main(void)
{
    /* Stop Watchdog timer */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /* Configure Peripherals */
    initGyro();
    initDelayTimer(CLK_FREQUENCY);
    setupBluetooth();
//    initalizeSonar();
    initTachometer();
    initCarLEDs(true);
    setupRelay();
    initL293D();

    // Initialize data variable
    RXDataPointer = 0;
    TXDataPointer = 0;

    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIA0_IRQn );

    // Enable eUSCIA2 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIA2_IRQn );

    // Enable global interrupt
    __enable_irq();

    // Main loop delay
    int i = 0;

    while(1)
    {
        /* Send message to base station */
        sendMessage();

        /* Delay for next transmission (while sampling tachometer) */
        adcInterruptEnabled = true;
        for (i = 50000; i > 0; i--);        // lazy delay
        adcInterruptEnabled = false;
    }
}

void driveCar(int direction) {
    /* 1 = forward
     * 2 = backward
     * 3 = left
     * 4 = right
     */
    switch(direction) {

    // Forward
    case 1:
//        if((getSonarDistance() / 6 * 10) < 400) {    // Hitting Wall
//            setMotorPWM(0, 0);
//            brakeLightState = true;
//            leftTurnSignalState = true;
//            rightTurnSignalState = true;
//        }
//        else {                            // Not hitting wall
            setMotorPWM(2000, 2000);
            brakeLightState = false;
            leftTurnSignalState = false;
            rightTurnSignalState = false;
//        }
        break;

    // Backward
    case 2:
        setMotorPWM(-1500, -2000);
        brakeLightState = true;
        leftTurnSignalState = false;
        rightTurnSignalState = false;
        break;

    // Left
    case 3:
        setMotorPWM(250, 2000);
        brakeLightState = false;
        leftTurnSignalState = true;
        rightTurnSignalState = false;
        break;

    // Right
    case 4:
        setMotorPWM(2000, 250);
        brakeLightState = false;
        leftTurnSignalState = false;
        rightTurnSignalState = true;
        break;

    // Stop Car
    case 0:
        setMotorPWM(0, 0);
        brakeLightState = true;
        leftTurnSignalState = false;
        rightTurnSignalState = false;
        break;
    }
}

void createMessage(void) {
    /* Get Sonar Reading */

    sonarReading = getSonarDistance() / 6 * 10;
    if(sonarReading >= 1000) { // Larger than 10, display something else
        tempResults[0] = ' ';
        tempResults[1] = '>';
        tempResults[2] = '1';
        tempResults[3] = '0';
    }
    else {
        parseSensor(sonarReading);
        tempResults[0] = sensorFirstDec;
        tempResults[1] = '.';
        tempResults[2] = sensorSecondDec;
        tempResults[3] = sensorThirdDec;
    }

    /* Get Gyro Reading */
    readGyroSensor();
    parseSensor(accel_y);
    if(isNegative) {
        tempResults[4] = '-';
    }
    else {
        tempResults[4] = ' ';
    }
    tempResults[5] = sensorFirstNum;
    tempResults[6] = sensorFirstDec;
    tempResults[7] = sensorSecondDec;

    /* Get Power (Shunt) Reading */
    parseSensor(ADC14->MEM[2] / 4);
    tempResults[8]  = sensorFirstNum;
    tempResults[9]  = sensorFirstDec;
    tempResults[10] = sensorSecondDec;
    tempResults[11] = sensorThirdDec;

    /* Get Voltage Reading */
    parseSensor(ADC14->MEM[3] * 2 + 1000);
    tempResults[12] = sensorFirstNum;
    tempResults[13] = sensorFirstDec;
    tempResults[14] = sensorSecondDec;
    tempResults[15] = sensorThirdDec;

    /* Get Tachometer Reading */
    parseSensor(tachometerTicks * 10000 / 150);
    tachometerTicks = 0;
    tempResults[16] = sensorFirstNum;
    tempResults[17] = sensorFirstDec;
    tempResults[18] = sensorSecondDec;
    tempResults[19] = sensorThirdDec;

    /* Create Message */
    snprintf(messageSent, sizeof messageSent, "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "\r\n"
                                              "******************************\r\n"
                                              "*       RC Car Stats         *\r\n"
                                              "*                            *\r\n"
                                              "* Sonar Sensor:  %c%c%c%c in     *\r\n"
                                              "*    Gyroscope: %c%c.%c%c G      *\r\n"
                                              "*      Current:  %c.%c%c%c Amps  *\r\n"
                                              "*      Voltage:  %c.%c%c%c Volts *\r\n"
                                              "*   Tachometer:  %c%c%c.%c RPM   *\r\n"
                                              "******************************\r\n",
             tempResults[0],  tempResults[1],  tempResults[2],  tempResults[3],
             tempResults[4],  tempResults[5],  tempResults[6],  tempResults[7],
             tempResults[8],  tempResults[9],  tempResults[10], tempResults[11],
             tempResults[12], tempResults[13], tempResults[14], tempResults[15],
             tempResults[16], tempResults[17], tempResults[18], tempResults[19]);
}

/* Send Message to Receiver */
void sendMessage() {

    /* Transmit String: sonar=0.000,gyro=0.000,power=0.000,volt=0.000,tach=0.000 */

    /* Variables */
    int i;
    int a;

    /* Parse Sensors and create message */
    createMessage();

    /* Transmit Message */
    for (a = 0; a <= strlen(messageSent); a++) {

       // Send next character of message
       EUSCI_A2->TXBUF = messageSent[a];

       for (i = 250; i > 0; i--);        // lazy delay
    }

    // Restart sampling/conversion by ADC
    ADC14->CTL0 |= 0b11;
}

void updateLEDs() {
    headlightsToggle(headLightState);
    brakelightsToggle(brakeLightState);
    turnSignalToggle(leftTurnSignalState, false);
    turnSignalToggle(rightTurnSignalState, true);
    underLightsToggle(underLightsState);
}


void readGyroSensor() {

    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

    /* Read register values from sensor by sending register address and restart
     *
     *  format for Write-Restart-Read operation
     *  _______________________________________________________________________
     *  |       | Periph | <Register |       | Periph |               |       |
     *  | Start |  Addr  |  Address> | Start |  Addr  | <6 Byte Read> | Stop  |
     *  |_______|____W___|___________|_______|____R___|_______________|_______|
     *
     *
     *  Initiated with start condition - completion handled in ISR
     */
    // change to transmitter mode (Write)
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
    // send I2C start condition with address frame and W bit
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    // wait for sensor data to be received
    while (RXDataPointer < NUM_OF_REC_BYTES) ;
    /* combine bytes to form 16-bit accel_ values  */
    accel_y = (RXData[2] << 8) + (RXData[3]);

    RXDataPointer = 0;
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
    sensorReading  /= 1;
    sensorThirdDec      = (sensorReading % 10) + '0';          // Find 3rd Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorSecondDec     = (sensorReading % 10) + '0';          // Find 2nd Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorFirstDec      = (sensorReading % 10) + '0';          // Find 1st Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorFirstNum      = (sensorReading) + '0';               // Find whole number
}

// Tachometer Interrupt Service Routine
void ADC14_IRQHandler(void) {
    // Check if interrupt triggered by ADC14MEM1 conversion value loaded
    //  Not necessary for this example since only one ADC channel used
    if (ADC14->IFGR0 & ADC14_IFGR0_IFG1) {

        curADCResult = ADC14->MEM[1];
        // First detection of open encoder wheel
        if (curADCResult >= 0x7FF & !locked) {    // Encoder wheel is open
            tachometerTicks++;
            locked = true;
        }

        // Detect beginning of closed encoder wheel
        else if (curADCResult < 0x7FF & locked) { // Encoder wheel is closed
            locked = false;
        }
        if(adcInterruptEnabled) {
            ADC14->CTL0 |= 0b11;
        }
    }
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

        /* Switch Case for Input */
        switch(inputChar) {

            // Forward
            case 'w':
                carState = 1;
                break;

            // Backward
            case 's':
                carState = 2;
                break;

            // Left
            case 'a':
                carState = 3;
                break;

            // Right
            case 'd':
                carState = 4;
                break;

            // Stop Car
            case ' ':
                carState = 0;
                break;

            // Under Lights
            case 'l':
                underLightsState = !underLightsState;
                break;

            // Head Lights
            case 'h':
                headLightState = !headLightState;
                break;

        } // end case

        /* Update Car LEDs */
        updateLEDs();

        /* Drive Car */
        driveCar(carState);
    }
}
// I2C interrupt service routine
void EUSCIB0_IRQHandler(void)
{
    // Handle if ACK not received for address frame
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // resend I2C start condition and address frame
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        TXDataPointer = 0;
        RXDataPointer = 0;
    }
    // When TX buffer is ready, load next byte or Restart for Read
    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
        if (TXDataPointer == 0) {
            // load 1st data byte into TX buffer (writing to buffer clears the flag)
            EUSCI_B0->TXBUF = ACCEL_BASE_ADDR;      // send register address
            TXDataPointer = 1;
        } else {
            // change to receiver mode (Read)
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            // send Restart and address frame with R bit
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
            TXDataPointer = 0;
            RXDataPointer = 0;
            // need to clear flag since not writing to buffer
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
        }
    }
    // When new byte is received, read value from RX buffer
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) {
        // Get RX data
        if (RXDataPointer < NUM_OF_REC_BYTES) {
            // reading the buffer clears the flag
            RXData[RXDataPointer++] = EUSCI_B0->RXBUF;
        }
        else {  // in case of glitch, avoid array out-of-bounds error
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;
        }

        // check if last byte being received - if so, initiate STOP (and NACK)
        if (RXDataPointer == (NUM_OF_REC_BYTES-1)) {
            // Send I2C stop condition
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
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
}
