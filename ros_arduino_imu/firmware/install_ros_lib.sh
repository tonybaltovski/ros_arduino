#!/bin/bash
cd "$(dirname "$0")"/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
