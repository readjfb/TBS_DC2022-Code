# TBS_DC2022-Code
Teensy 4.0 code for Team TBS (McCormick Design Competition 2022)
Team members: Jackson Bremen, Garrett Short, Evan Waite
Development using PlatformIO in VSCode

## Current Status
Robot is working with dead reckoning via encoders
We were hoping to use gyroscopes/ IMU, but didn't recieve the parts
Robot dispenses tokens continuously while moving 

## Features
Motor speeds are controlled via PID loop, allowing programmer to specify a speed in m/s, and the wheels will maintain speed to a high degree of fidelity
Robot uses PID for driving, both linearly and for turning; programmer specifies a distance to drive or an angle to turn at, and the robot executes the turn pretty well

## Potential next steps
Add sensors, such as laser rangefinders, IMU/Gyro/Magnetometer Sensor Suite, or bump sensors
Refine hadware
Create built-in model of the course; allow robot to truly autonomously solve the course using TSP, Djikstra's, A*, or other

