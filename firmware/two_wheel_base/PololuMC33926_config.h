#ifndef _PololuMC33926_CONFIG_H_
#define _PololuMC33926_CONFIG_H_

#include "motor_driver_config.h"
#include <PololuMC33926.h>

// Left and Right motor driver objects
MC33926 left_motor(2,3,4,5);
MC33926 right_motor(12,11,10,9);
void setupMotors()
{
  // Initalize Motors
  left_motor.init();
  right_motor.init();
}
void commandLeftMotor(int cmd)
{
  left_motor.set_pwm(cmd);
}
void commandRightMotor(int cmd)
{
  right_motor.set_pwm(cmd);
}

#endif  // _PololuMC33926_CONFIG_H_

