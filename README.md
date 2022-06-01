# Maze-Solver

## Code
The code was written on the Atmega328P board. All low-level drivers were implemented by us, to achieve the best performance and the least code size.

## Components & cost
| Name           | Use & description          | Number | market Cost per Part                                                                                                         | Notes                |
| -------------- | -------------------------- | ------ | ---------------------------------------------------------------------------------------------------------------------------- | -------------------- |
| Arduino Uno    | The main development board | 1      | [160 EGP](https://free-electronic.com/product/arduino-uno-r3-ch340-usb-cable/)                                               | We already had it    |
| TCRT 5000      | The IR sensor              | 5      | [10 EGP](https://store.fut-electronics.com/products/tcrt5000-reflective-ir-sensor?_pos=1&_sid=65bee30f3&_ss=r)               | \-                   |
| Gearbox Motors | The used motors            | 2      | [25 EGP](https://store.fut-electronics.com/products/dc-geared-motors-for-robots-straight-shaft?_pos=33&_sid=eb26e25ca&_ss=r) | We already had them  |
| Motor Driver   | The L298N H-bridge         | 1      | [70 EGP](https://store.fut-electronics.com/products/l298-dual-motor-driver-module-2a?_pos=16&_sid=eb26e25ca&_ss=r)           | We already had it    |
| Car chassis    | Car’s Main body            | 1      | 190 EGP                                                                                                                      | We had 3d-printed it |
| Car batteries  | Lithium Rechargeable ones  | 3      | 35 EGP                                                                                                                       | \-                   |
| Battery Holder | To hold the batteries      | 1      | 15 EGP                                                                                                                       | \-                   |

## Schematic diagrams
Our Car consists of 5 IR sensors and 2 DC motors. In this schematic, we show the connections of the sensors, motors, and power connections.

![Copy of Line Follower](https://user-images.githubusercontent.com/56788883/171509716-32decac7-9016-49d1-b802-5e7c3174b95f.png)
>> Note that:
We have used the L293N motor driver, but since it is not supported in the simulation, we have shown the connections of the L293D H-bridge with a voltage regulator. 
We have used the TCRT 5000 IR sensors, but since it is not supported in the simulation, we have used another module.

## Challenges
- **The Tight Range of the IR Sensor**: One of the most crucial issues we have faced was the tight range of the IR sensor, we weren’t able to agree on a cutting threshold between the white and black colors that was robust to other color and heat changes. We have solved this problem by using some resistors connected with the sensor and tuning its value until we reached the best range and threshold. That came with the cost of slower readings, but a few milliseconds longer wasn’t very critical to our application.
- **Varying Voltage Level**: Simply, we were somehow tuning the optimum velocity profile for our car. But, the velocity was affected by how much the batteries are charged. We managed to overcome this issue by using an AC/DC power adapter during the testing and adding fully charged batteries during the competition.
- **Number of IR Sensors**: We managed to tackle the line following problem using only 3 sensors but, that wasn't enough to do the same in solving the maze. We had to use extra two sensors; three for following the line and the other two to detect the turns.

