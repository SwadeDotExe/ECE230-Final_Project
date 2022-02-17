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
//#include "Drivers/sonarSensor.h"
//#include "Drivers/tachometer.h"
//#include "Drivers/gyro.h"
//#include "Drivers/carLEDs.h"
//#include "Drivers/relay.h"
//
///* Defines */
//#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT
//#define NUM_OF_REC_BYTES        6           // number of bytes to receive from sensor read
//
///* Raw Storage for Gyro Sensor */
//uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
//uint8_t RXDataPointer, TXDataPointer;
//
///* Processed Readings from Gyro Sensor */
//volatile int16_t accel_x, accel_y, accel_z;
//bool isNegative = false;
//char sensorThirdDec;
//char sensorSecondDec;
//char sensorFirstDec;
//char sensorFirstNum;
//
///* Variable to hold Tachometer interrupt count */
//volatile uint32_t tachometerTicks = 0;
//static volatile uint16_t curADCResult;
//
///* System Start Flag */
//bool hasStarted = false;
//
//bool locked = false;
//bool adcInterruptEnabled = false;
//
///* Variables for Serial Input */
//char inputChar;
//bool recievingData = false;
//const int messageRecievedLength = 21;
//int  recievedIndex = 0;
//char recievedMessage[messageRecievedLength];
//bool messageDone = false;
//
///* Variables to hold Recieved Data */
//int leftWheelSpeed;
//int rightWheelSpeed;
//bool headLightState = false;
//bool brakeLightState = false;
//bool leftTurnSignalState = false;
//bool rightTurnSignalState = false;
//
///* Variables for Message Transmission */
//char tempResults[20];
//char messageSent[60];
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
//    initGyro();
//    initDelayTimer(CLK_FREQUENCY);
//    setupBluetooth();
//    initalizeSonar();
//    initTachometer();
//    initCarLEDs(false);
//    setupRelay();
//
//    // Initialize data variable
//    RXDataPointer = 0;
//    TXDataPointer = 0;
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
//    // Main loop delay
//    int i = 0;
//
//    while(1)
//    {
//        /* Send message to base station */
//        sendMessage();
//
//        /* Delay for next transmission (while sampling tachometer) */
//        adcInterruptEnabled = true;
//        for (i = 10000; i > 0; i--);        // lazy delay
//        adcInterruptEnabled = false;
//    }
//}
//
//void createMessage(void) {
//    /* Get Sonar Reading */
//    parseSensor(getSonarDistance());
//    tempResults[0] = sensorFirstNum;
//    tempResults[1] = sensorFirstDec;
//    tempResults[2] = sensorSecondDec;
//    tempResults[3] = sensorThirdDec;
//
//    /* Get Gyro Reading */
//    readGyroSensor();
//    parseSensor(accel_x);
//    tempResults[4] = sensorFirstNum;
//    tempResults[5] = sensorFirstDec;
//    tempResults[6] = sensorSecondDec;
//    tempResults[7] = sensorThirdDec;
//
//    /* Get Power (Shunt) Reading */
//    parseSensor(ADC14->MEM[2]);
//    tempResults[8]  = sensorFirstNum;
//    tempResults[9]  = sensorFirstDec;
//    tempResults[10] = sensorSecondDec;
//    tempResults[11] = sensorThirdDec;
//
//    /* Get Voltage Reading */
//    parseSensor(ADC14->MEM[3]);
//    tempResults[12] = sensorFirstNum;
//    tempResults[13] = sensorFirstDec;
//    tempResults[14] = sensorSecondDec;
//    tempResults[15] = sensorThirdDec;
//
//    /* Get Tachometer Reading */
//    parseSensor(tachometerTicks * 1000);
//    tachometerTicks = 0;
//    tempResults[16] = sensorFirstNum;
//    tempResults[17] = sensorFirstDec;
//    tempResults[18] = sensorSecondDec;
//    tempResults[19] = sensorThirdDec;
//
//    /* Create Message */
//    snprintf(messageSent, sizeof messageSent, "<%c%c%c%c,%c%c%c%c,%c%c%c%c,%c%c%c%c,%c%c%c%c>\r\n",
//             tempResults[0],  tempResults[1],  tempResults[2],  tempResults[3],
//             tempResults[4],  tempResults[5],  tempResults[6],  tempResults[7],
//             tempResults[8],  tempResults[9],  tempResults[10], tempResults[11],
//             tempResults[12], tempResults[13], tempResults[14], tempResults[15],
//             tempResults[16], tempResults[17], tempResults[18], tempResults[19]);
//}
//
///* Send Message to Receiver */
//void sendMessage() {
//
//    /* Transmit String: sonar=0.000,gyro=0.000,power=0.000,volt=0.000,tach=0.000 */
//
//    /* Variables */
//    int i;
//    int a;
//
//    /* Parse Sensors and create message */
//    createMessage();
//
//    /* Transmit Message */
//    for (a = 0; a <= strlen(messageSent); a++) {
//
//       // Send next character of message
//       EUSCI_A2->TXBUF = messageSent[a];
//
//       for (i = 1000; i > 0; i--);        // lazy delay
//    }
//
//    // Restart sampling/conversion by ADC
//    ADC14->CTL0 |= 0b11;
//}
//
//void updateLEDs() {
//    headlightsToggle(headLightState);
//    brakelightsToggle(brakeLightState);
//    turnSignalToggle(leftTurnSignalState, false);
//    turnSignalToggle(rightTurnSignalState, true);
//}
//
//void recieveMessage() {
//
//    /* Recieve String: leftsteering,rightsteering,headlights,brakelights,leftturnsig,rightturnsig, */
//    /*             <   xxxx        ,xxxx         ,x         ,x          ,x          ,x           > */
//
//
//    /* Check Message for Errors */
//    if(recievedMessage[4]  == ',' &&
//       recievedMessage[9]  == ',' &&
//       recievedMessage[11] == ',' &&
//       recievedMessage[13] == ',' &&
//       recievedMessage[15] == ',')
//
//    /* Continue with message if no errors */
//    {
//            // Left Wheels Speed
//            leftWheelSpeed   = (recievedMessage[0] - '0') * 1000 +
//                               (recievedMessage[1] - '0') * 100  +
//                               (recievedMessage[2] - '0') * 10   +
//                               (recievedMessage[3] - '0') * 1;
//
//            // Right Wheels Speed
//            rightWheelSpeed  = (recievedMessage[5] - '0') * 1000 +
//                               (recievedMessage[6] - '0') * 100  +
//                               (recievedMessage[7] - '0') * 10   +
//                               (recievedMessage[8] - '0') * 1;
//
//            // Headlights
//            if(recievedMessage[10] == '1') {
//                headLightState = true;
//            }
//            else {
//                headLightState = false;
//            }
//
//            // Brake Lights
//            if(recievedMessage[12] == '1') {
//                brakeLightState = true;
//            }
//            else {
//                brakeLightState = false;
//            }
//
//            // Left Turn Signal
//            if(recievedMessage[14] == '1') {
//                leftTurnSignalState = true;
//            }
//            else {
//                leftTurnSignalState = false;
//            }
//
//            // Right Turn Signal
//            if(recievedMessage[16] == '1') {
//                rightTurnSignalState = true;
//            }
//            else {
//                rightTurnSignalState = false;
//            }
//
//            /* Update Car LEDs */
//            updateLEDs();
//
//    } // End of error check loop
//}
//
//void readGyroSensor() {
//
//    // Ensure stop condition got sent
//    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
//
//    /* Read register values from sensor by sending register address and restart
//     *
//     *  format for Write-Restart-Read operation
//     *  _______________________________________________________________________
//     *  |       | Periph | <Register |       | Periph |               |       |
//     *  | Start |  Addr  |  Address> | Start |  Addr  | <6 Byte Read> | Stop  |
//     *  |_______|____W___|___________|_______|____R___|_______________|_______|
//     *
//     *
//     *  Initiated with start condition - completion handled in ISR
//     */
//    // change to transmitter mode (Write)
//    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
//    // send I2C start condition with address frame and W bit
//    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
//
//    // wait for sensor data to be received
//    while (RXDataPointer < NUM_OF_REC_BYTES) ;
//    /* combine bytes to form 16-bit accel_ values  */
//    accel_x = (RXData[0] << 8) + (RXData[1]);
//
//    RXDataPointer = 0;
//}
//
//void parseSensor(int16_t sensorReading) {
//
//    /* Check for negative before regex */
//    if(sensorReading < 0) {
//        // Set negative flag
//        isNegative = true;
//        // Convert to positive
//        sensorReading *= -1;
//    }
//    else {
//        isNegative = false;
//    }
//
//    /* Do regex on raw reading */
//    sensorReading  /= 4;
//    sensorThirdDec      = (sensorReading % 10) + '0';          // Find 3rd Decimal
//    sensorReading  /= 10;                                   // Shift Bits
//    sensorSecondDec     = (sensorReading % 10) + '0';          // Find 2nd Decimal
//    sensorReading  /= 10;                                   // Shift Bits
//    sensorFirstDec      = (sensorReading % 10) + '0';          // Find 1st Decimal
//    sensorReading  /= 10;                                   // Shift Bits
//    sensorFirstNum      = (sensorReading) + '0';               // Find whole number
//}
//
//// Tachometer Interrupt Service Routine
//void ADC14_IRQHandler(void) {
//    // Check if interrupt triggered by ADC14MEM1 conversion value loaded
//    //  Not necessary for this example since only one ADC channel used
//    if (ADC14->IFGR0 & ADC14_IFGR0_IFG1) {
//
//        curADCResult = ADC14->MEM[1];
//        // First detection of open encoder wheel
//        if (curADCResult >= 0x7FF & !locked) {    // Encoder wheel is open
//            tachometerTicks++;
//            locked = true;
//        }
//
//        // Detect beginning of closed encoder wheel
//        else if (curADCResult < 0x7FF & locked) { // Encoder wheel is closed
//            locked = false;
//        }
//        if(adcInterruptEnabled) {
//            ADC14->CTL0 |= 0b11;
//        }
//    }
//}
//
//// UART interrupt service routine (Received data)
//void EUSCIA2_IRQHandler(void)
//{
//    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
//    {
//        // Check if the TX buffer is empty first
//        while(!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG));
//
//        // Capture recieved byte
//        inputChar = EUSCI_A2->RXBUF;
//
//        // End of Transmission
//        if((char)inputChar == '>') {
//            recievingData = false;
//            recievedIndex = 0;
//            messageDone = true;
//            P1->OUT &= ~BIT0;                     // Turn LED1 off
//            recieveMessage();
//        }
//
//        // Capture Data
//        if(recievingData) {
//            recievedMessage[recievedIndex] = inputChar;
//            recievedIndex++;
//        }
//
//        // Start of Transmission
//        if((char)inputChar == '<') {
//            P1->OUT |= BIT0;                     // Turn LED1 off
//            recievingData = true;
//            messageDone = false;
//            recievedIndex = 0;
//        }
//    }
//}
//// I2C interrupt service routine
//void EUSCIB0_IRQHandler(void)
//{
//    // Handle if ACK not received for address frame
//    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
//        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;
//
//        // resend I2C start condition and address frame
//        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
//        TXDataPointer = 0;
//        RXDataPointer = 0;
//    }
//    // When TX buffer is ready, load next byte or Restart for Read
//    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
//        if (TXDataPointer == 0) {
//            // load 1st data byte into TX buffer (writing to buffer clears the flag)
//            EUSCI_B0->TXBUF = ACCEL_BASE_ADDR;      // send register address
//            TXDataPointer = 1;
//        } else {
//            // change to receiver mode (Read)
//            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
//            // send Restart and address frame with R bit
//            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
//            TXDataPointer = 0;
//            RXDataPointer = 0;
//            // need to clear flag since not writing to buffer
//            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
//        }
//    }
//    // When new byte is received, read value from RX buffer
//    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) {
//        // Get RX data
//        if (RXDataPointer < NUM_OF_REC_BYTES) {
//            // reading the buffer clears the flag
//            RXData[RXDataPointer++] = EUSCI_B0->RXBUF;
//        }
//        else {  // in case of glitch, avoid array out-of-bounds error
//            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;
//        }
//
//        // check if last byte being received - if so, initiate STOP (and NACK)
//        if (RXDataPointer == (NUM_OF_REC_BYTES-1)) {
//            // Send I2C stop condition
//            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
//        }
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
