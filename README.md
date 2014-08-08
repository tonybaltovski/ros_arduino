# Overview #

ros_arduino_base provide a basic interface two wheel differential mobile base.  
## On-board (the micro-controller) ##
- Velocity control is done on-board using encoders based on a differential velocity command. 
- Also, the encoder data is sent to an off-board node for processing.  

## Off-board (the PC) ##
- Off-board, the encoder data is processed to determine the pose of base.  
- In addition, the standard velocity topic (twist) is converted to left and right velocities.

## Current motor drivers
- Dual MC33926 Motor Drivers Carrier
- DFRobot L298P Motor Shield (2A)