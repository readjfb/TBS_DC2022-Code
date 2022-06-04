# TBS_DC2022-Code
Team TBS (McCormick Design Competition 2022)  
&rarr; Jackson Bremen, Garrett Short, Evan Waite  
Development using PlatformIO in VSCode || Code to be executed on Teensy 4.0

## Current Status
Robot is working with dead reckoning via encoders
We were hoping to use gyroscopes/ IMU, but didn't recieve the parts
Robot dispenses tokens continuously while moving 
Due to time constraints, we were not able to execute the robot as completely as we would have liked, but completed the task to a satisfactory level

## Demo


https://user-images.githubusercontent.com/15671813/172027598-4a8e7318-7596-4adb-a696-1f8902d849cd.mov


**note - Tokens were not loaded on recorded run*

## Features
Motor speeds are controlled via PID loop, allowing programmer to specify a speed in m/s, and the wheels will maintain speed to a high degree of fidelity
Robot uses PID for driving, both linearly and for turning; programmer specifies a distance to drive or an angle to turn at, and the robot executes the turn pretty well  
Code is completely nonblocking and runs fast  
Robot continuously dispenses tokens while robot moves

## Basic Structure
Code is written as object oriented C++. Motors, Dispenser are base classes. Robot holds instances of them as needed, and uses their functions (see [MotorClass.h](https://github.com/readjfb/TBS_DC2022-Code/blob/103447f442ba9b6c0e5b22dbf0b4158de8f8f7a0/PID_Robot/include/MotorClass.h)). Within Robot.cpp, the large switch statement in update_robot acts as a basic state machine, allowing us to step through the steps of the robot

## Unused features
Code to use MPU6050 was written but not integrated, as sensor precision was lacking

## Next steps
- [ ] Add sensors, such as laser rangefinders, IMU/Gyro/Magnetometer Sensor Suite, or bump sensors
- [ ] Refine hadware
- [ ] Create built-in model of the course; allow robot to truly autonomously solve the course using TSP, Djikstra's, A*, or other
- [ ] Add a stop/ reset button to electronics stack 




