# ECE230 Final Project
## _Group Members: Swade and Bryce_
![ECE230 - C](https://img.shields.io/badge/ECE230-C-red)
![Winter - 2022](https://img.shields.io/badge/Winter-2022-blue)
## Basic Overview
For the arcade game project, we made a classic arcade style Bomb Jack game. Using what we learned in CSSE220 and in some cases going beyond what was taught in class, we designed a game based on the 1984 classic aracade game Bomb Jack using Java.

## MSP to MSP Bluetooth Communication

A problem we faced was trying to setup our HC-06 Bluetooth modules to communicate with each other. The goal was to have one in master mode and the other in slave, however the HC-06 can only support slave mode. We had to order a HC-05 which supports both of these modes and set it up with the commands below:

**Wiring**

| Arduino | HC-05/06 |
|:-------:|:--------:|
| 3.3V    | VCC      |
| 3.3V    | EN       |
| GND     | GND      |
| TXD     | TXD      |
| RXD     | RXD      |

**Slave Configuration:**
- "AT"              --> OK
- "AT+UART?"        --> 38400,0,0
- "AT+ROLE?"        --> 0
- "AT+ADDR?"        --> Slave's Address
- "AT+NAME=RCSlave" --> OK

**Master Configuration:**
- "AT"                       --> OK
- "AT+UART?"                 --> 38400,0,0
- "AT+ROLE=1"                --> OK
- "AT+CMODE=0"               --> OK
- "AT+BIND=*Slave's Address* --> OK
- "AT+NAME=RCMaster"         --> OK


## Sonar Sensor
