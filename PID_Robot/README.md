# Overview
This is code for a Teensy 4.0, written in C++ for the 2022 DC competition
There are several layers of control logic that will be implemented

At the Motor level (src/MotorClass MOTOR class), the idea is to specify a targeted velocity, and a tuned PID loop will ensure that the wheels turn at a precise velocity

At the Robot level (to be implemented), the idea is to use two types of command
1. Linear move
 - Specify a heading to maintain and a distance, and robot will drive the distance
 - Heading keeping is the foremost concern, with distance a second major objective
2. Rotate
 - Specify a heading to rotate to (need to decide if it will be relative or absolute)
 - Robot will use a tuned PID loop to turn to the precise heading

 At the Control level (to be implemented), the objective is to specify a sequence of Robot move commands, which robot will follow
 - Need to decide if it will have a fixed path, dynamic, what