#!/bin/bash
cd "$(dirname "$0")"/libraries
rm -rf *
rosrun rosserial_arduino make_libraries.py .
git clone https://github.com/tonybaltovski/PololuMC33926MotorDriver.git
wget http://www.pjrc.com/teensy/arduino_libraries/Encoder.zip
unzip Encoder.zip
rm Encoder.zip
