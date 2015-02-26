#ifndef _PololuMC33926_CONFIG_H_
#define _PololuMC33926_CONFIG_H_

#include "motor_driver_config.h"

// Left and Right motor driver objects
MC33926 left_motor(2,3,5,4);
MC33926 right_motor(12,11,10,9);

void setupMotors()
{
  // Initalize Motors
  left_motor.init();
  right_motor.init();
}
void commandLeftMotor(int16_t cmd)
{
  left_motor.set_pwm(cmd);
}
void commandRightMotor(int16_t cmd)
{
  right_motor.set_pwm(cmd);
}

#endif  // _PololuMC33926_CONFIG_H_

