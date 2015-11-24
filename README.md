ros_arduino[![Build Status](https://travis-ci.org/tonybaltovski/ros_arduino.svg?branch=indigo-devel)](https://travis-ci.org/tonybaltovski/ros_arduino)
===========
## Overview ##
ros_arduino provides a simple interface for a mobile robot using an Arduino(-like) mircocontroller using [rosserial](https://github.com/ros-drivers/rosserial).

### ros_arduino_base
This package provides an interface to a two wheel differential mobile base with encoders to produce an odometry estimate.  You can copy this to your sketchbook or set your sketchbook location to the firmware folder.  The firmware folder contains a script `install_ros_lib.sh` to generate the ros_lib and other libraries that are needed.  In `two_wheel_base.ino` you can configure your base's settings.

### ros_arduino_imu
This package provides an interface for various I2C IMUs to ROS.  The firmware that needs to uploaded to the Arduino(-like) mircocontroller is in the firmware folder.  You can copy this to your sketchbook or set your sketchbook location to the firmware folder.  The firmware folder contains a script `install_ros_lib.sh` to generate the ros_lib that is needed.  In `imu_configuration.h` you can configure your IMU.

