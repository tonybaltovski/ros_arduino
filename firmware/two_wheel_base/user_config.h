#ifndef _USER_CONFIG_H_
#define _USER_CONFIG_H_

// Select your motor driver here
//#define PololuMC33926
#define DFRobotL298PShield
// Define your encoder pins here.
// Try to use pins that have interrupts
// Left side encoders pins
#define LEFT_ENCODER_A 14  // Interrupt on Teensy 3.0
#define LEFT_ENCODER_B 15  // Interrupt on Teensy 3.0
// Right side encoders pins
#define RIGHT_ENCODER_A 6  // Interrupt on Teensy 3.0
#define RIGHT_ENCODER_B 7  // Interrupt on Teensy 3.0

#endif  // _USER_CONFIG_H_
