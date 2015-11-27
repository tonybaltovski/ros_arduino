ros_arduino[![Build Status](https://travis-ci.org/tonybaltovski/ros_arduino.svg?branch=indigo-devel)](https://travis-ci.org/tonybaltovski/ros_arduino)
===========
## Overview ##
ros_arduino provides a simple interface for a mobile robot using an Arduino(-like) mircocontroller using [rosserial](https://github.com/ros-drivers/rosserial).

### udev rules for Arduino

Add this udev rule to your system:

```
KERNEL=="ttyACM*", SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", MODE="666", SYMLINK+="arduino arduino_$attr{serial}", GROUP="dialout"
```

or, for the Arduino Mega:

```
KERNEL=="ttyACM*", SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="2a03", ATTRS{idProduct}=="0042", MODE="666", SYMLINK+="arduino arduino_mega_$attr{serial}", GROUP="dialout"
```

### Building

Clone this repo into the src folder of a catkin workspace, run catkin_make, and then source your devel/setup.sh .

### ros_arduino_base
This package provides an interface to a two wheel differential mobile base with encoders to produce an odometry estimate.  You can copy this to your sketchbook or set your sketchbook location to the firmware folder.  The firmware folder contains a script `install_ros_lib.sh` to generate the ros_lib and other libraries that are needed.  In `two_wheel_base.ino` you can configure your base's settings.

### ros_arduino_imu
This package provides an interface for various I2C IMUs to ROS.  The firmware that needs to uploaded to the Arduino(-like) mircocontroller is in the firmware folder.  You can copy this to your sketchbook or set your sketchbook location to the firmware folder.  The firmware folder contains a script `install_ros_lib.sh` to generate the ros_lib that is needed.  In `imu_configuration.h` you can configure your IMU.

### ros_arduino_sonar

This package provides an interface for the HC-SR04 ultrasonic ranging module. The firmware that needs to uploaded to the Arduino(-like) mircocontroller is in the firmware folder.  You can copy this to your sketchbook or set your sketchbook location to the firmware folder.

To launch the ROS interface, run:

```
roslaunch ros_arduino_sonar sonar_ahrs.launch
```

This will publish topics /sonar1 ... /sonarn of type [sensor_msgs/Range](http://docs.ros.org/jade/api/sensor_msgs/html/msg/Range.html) with links to /sonar1_link ... /sonarn_link.


