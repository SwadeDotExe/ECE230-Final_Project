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
- 
- **Master Configuration:**
- "AT"                       --> OK
- "AT+UART?"                 --> 38400,0,0
- "AT+ROLE=1"                --> OK
- "AT+CMODE=0"               --> OK
- "AT+BIND=*Slave's Address* --> OK
- "AT+NAME=RCMaster"         --> OK


## Essential Features:
- Bomb Jack who moves left, right and up with arrow keys, stands on platforms, falls (due to gravity), and collects cherry bombs, and starts out with 4 number of lives
- Two types of Aliens: initial aliens just move back and forth on the platforms.
- When the hero and an alien collide, the hero dies and loses a life.
- There are cherry bombs placed at various locations on a level, and are picked by the hero when he collides with them. Each time a bomb is collected, the hero scores points.
- If the hero collects all the bombs on a level, then the player moves on to the next level.
- Each level has its own configuration of bombs, aliens, platforms.
- When the hero loses all 4 lives then the game is over.
- The game loada each level from a pre-created text-based data file.
- The game displays the current score and the number of lives the hero currently has.
- Includes **U** (up) and **D** (down) keystrokes to go the next level or go back a level.

## Extra Features:
- The hero has shooting capability (shoot little mario bullets), and if an alien is hit by the hero’s bullet, then the alien is destroyed and disappears from the screen and the hero scores points. [Keystrokes: **W** (to shoot to left of hero) and **E** (to shoot to right of hero)

![Bullets](https://github.com/swadewhite/CSSE220-Final_Project/blob/main/GIFs/Shooting.gif)

- Custom edited backgrounds and images to look like the actual game.

![Background](https://github.com/swadewhite/CSSE220-Final_Project/blob/main/images/Background3.png)

- Gameover and game win screens

![GameOver](https://github.com/swadewhite/CSSE220-Final_Project/blob/main/GIFs/GameOver.gif)

- There is a second type of alien that is designed to track the hero and move towards him depending on his location in relation to the alien.

![Tracking](https://github.com/swadewhite/CSSE220-Final_Project/blob/main/GIFs/Tracking.gif)

- Boss Level: Consists of a giant tracking alien. Requires a bunch of bullets to kill the alien, and you have to beat the alien in order to win the game.

![BossLevel](https://github.com/swadewhite/CSSE220-Final_Project/blob/main/GIFs/BossLevel.gif)

- Debug Stats

![DebugStats](https://github.com/swadewhite/CSSE220-Final_Project/blob/main/images/DebugStats.png)

# UML Class Diagrams
- Original UML Diagram:
![oldUML](https://user-images.githubusercontent.com/44556609/141491167-7debf709-0de0-40ee-b5ba-89e5b4adf57e.png)

- Final UML Diagram:
![newUML](https://user-images.githubusercontent.com/44556609/141490950-75b9b740-93ee-41d9-8c7f-0ee461fa609e.png)
