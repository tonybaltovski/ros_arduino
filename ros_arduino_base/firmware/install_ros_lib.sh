#!/bin/bash
cd "$(dirname "$0")"/libraries
rm -rf *
rosrun rosserial_arduino make_libraries.py .
git clone https://github.com/tonybaltovski/PololuMC33926MotorDriver.git
git clone https://github.com/tonybaltovski/DFRobotL298PShieldDriver.git
git clone https://github.com/PaulStoffregen/Encoder.git
