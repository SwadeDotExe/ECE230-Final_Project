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
 *
 * Transmit String: sonar=0000,gyro=0000,power=0000,volt=0000
 *          Length: 41
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

/* Defines */
#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT
#define NUM_OF_REC_BYTES        6           // number of bytes to receive from sensor read
#define GY521_ADDRESS           0x68        // I2C address of GY-521 sensor
#define ACCEL_BASE_ADDR         0x3B        // base address of accelerometer data registers
#define PWR_MGMT_ADDR           0x6B        // address of power management register

/* Raw Storage for Gyro Sensor */
uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
uint8_t RXDataPointer, TXDataPointer;

/* Variables for Serial Input */
bool inputRecieved = false;
char inputChar;

/* Processed Readings from Gyro Sensor */
volatile int16_t accel_x, accel_y, accel_z;
bool isNegative = false;
char sensorThirdDec;
char sensorSecondDec;
char sensorFirstDec;
char sensorFirstNum;

/* System Start Flag */
bool hasStarted = false;

///* Used in Loops */
//volatile uint32_t i;
//volatile uint32_t a;

/**
 * main.c
 */
void main(void)
{
    /* Stop Watchdog timer */
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    /* Configure Peripherals */
    configHFXT();
    configGyro();
    initDelayTimer(CLK_FREQUENCY);
    setupSerial();
    configLCD(CLK_FREQUENCY);
    initLCD();
    initalizeSonar();

    int i = 0;

    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);

      // Enable eUSCIA0 interrupt in NVIC module
      NVIC->ISER[0] = (1 << EUSCIA0_IRQn );

      // Enable global interrupt
      __enable_irq();

    while(1)
    {
        sendMessage();
        for (i = 20000; i > 0; i--);        // lazy delay

    }
}

/* Send Message to Receiver */
void sendMessage() {

    /* Transmit String: sonar=0000,gyro=0000,power=0000,volt=0000 */

    /* Variables */
    int i;
    int a;
    char tempResults[16];
    char messageSent[48];

    /* Get Sonar Reading */
    parseSensor(getSonarDistance());
    tempResults[0] = sensorFirstNum;
    tempResults[1] = sensorFirstDec;
    tempResults[2] = sensorSecondDec;
    tempResults[3] = sensorThirdDec;

    /* Get Gyro Reading */
    readGyroSensor();
    parseSensor(accel_x);
    tempResults[4] = sensorFirstNum;
    tempResults[5] = sensorFirstDec;
    tempResults[6] = sensorSecondDec;
    tempResults[7] = sensorThirdDec;

    /* Get Power (Shunt) Reading */
//    parseSensor(accel_x);
    tempResults[8]  = sensorFirstNum;
    tempResults[9]  = sensorFirstDec;
    tempResults[10] = sensorSecondDec;
    tempResults[11] = sensorThirdDec;

    /* Get Voltage Reading */
//    parseSensor(accel_x);
    tempResults[12] = sensorFirstNum;
    tempResults[13] = sensorFirstDec;
    tempResults[14] = sensorSecondDec;
    tempResults[15] = sensorThirdDec;

    /* Create Message */
    snprintf(messageSent, sizeof messageSent, "sonar=%c.%c%c%c,gyro=%c.%c%c%c,power=%c.%c%c%c,volt=%c.%c%c%c\r\n",
             tempResults[0],  tempResults[1],  tempResults[2],  tempResults[3],
             tempResults[4],  tempResults[5],  tempResults[6],  tempResults[7],
             tempResults[8],  tempResults[9],  tempResults[10], tempResults[11],
             tempResults[12], tempResults[13], tempResults[14], tempResults[15]);

    /* Transmit Message */
    for (a = 0; a < strlen(messageSent); a++) {

        // Send next character of message
        //  Note that writing to TX buffer clears the flag
        EUSCI_A0->TXBUF = messageSent[a];

        for (i = 2000; i > 0; i--);        // lazy delay
    }
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
    accel_x = (RXData[0] << 8) + (RXData[1]);

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
    sensorReading  /= 4;
    sensorThirdDec      = (sensorReading % 10) + '0';          // Find 3rd Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorSecondDec     = (sensorReading % 10) + '0';          // Find 2nd Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorFirstDec      = (sensorReading % 10) + '0';          // Find 1st Decimal
    sensorReading  /= 10;                                   // Shift Bits
    sensorFirstNum      = (sensorReading) + '0';               // Find whole number
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

        // Tell system there is an input
        inputRecieved = true;

        // Sanitize Input
        if(EUSCI_A0->RXBUF == 'x' | EUSCI_A0->RXBUF == 'y' | EUSCI_A0->RXBUF == 'z' |
           EUSCI_A0->RXBUF == 'X' | EUSCI_A0->RXBUF == 'Y' | EUSCI_A0->RXBUF == 'Z') {
            inputChar = EUSCI_A0->RXBUF;
        }

        // Echo the received character back
        //  Note that reading RX buffer clears the flag and removes value from buffer
        EUSCI_A0->TXBUF = EUSCI_A0->RXBUF;


        // Check for debug menu (p)
        if(EUSCI_A0->RXBUF == 'p' | EUSCI_A0->RXBUF == 'P') {
//            printDebug();
        }

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

void configGyro() {
    /* Configure UART pins */
    P1->SEL0 |= BIT6 | BIT7;                // set I2C pins as secondary function
    P1->SEL1 &= ~(BIT6 | BIT7);

    // Initialize data variable
    RXDataPointer = 0;
    TXDataPointer = 0;

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

void setupSerial() {
    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
            CS_CTL1_SELS_3 |                // SMCLK = DCO
            CS_CTL1_SELM_3;                 // MCLK = DCO
    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Configure UART pins */
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
    P1->SEL1 &= ~(BIT2 | BIT3);

    /* Configure UART
     *  Asynchronous UART mode, 8O1 (8-bit data, odd parity, 1 stop bit),
     *  LSB first, SMCLK clock source
     */
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    // DONE complete configuration of UART in eUSCI_A0 control register
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | 0b1100000010000000;

    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 12000000, Baud rate = 38400
     *
     * DONE calculate N and determine values for UCBRx, UCBRFx, and UCBRSx
     *          values used in next two TODOs
     */
    // DONE set clock prescaler in eUSCI_A0 baud rate control register
    EUSCI_A0->BRW = 19;
    // DONE configure baud clock modulation in eUSCI_A0 modulation control register
    EUSCI_A0->MCTLW = 0b110010110000001;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt
}

