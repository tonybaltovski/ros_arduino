#ifndef _DFRobotL298PShield_CONFIG_H_
#define _DFRobotL298PShield_CONFIG_H_

#include "motor_driver_config.h"

// Motor Driver Pins
//Left Motor
#define left_motor_pwm 7
#define left_motor_enable 6
//Right Motor
#define right_motor_enable 5
#define right_motor_pwm 4
// Left and Right motor driver objects

void setupMotors()
{

}
void commandLeftMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(left_motor_pwm, 1);
  }
  else
  {
    digitalWrite(left_motor_pwm, 0);
  }
  analogWrite(left_motor_enable, abs(cmd));
}
void commandRightMotor(int16_t cmd)
{
  if (cmd >= 0)
  {
    digitalWrite(right_motor_pwm, 1);
  }
  else
  {
    digitalWrite(right_motor_pwm, 0);
  }
  analogWrite(right_motor_enable, abs(cmd));
}


#endif  // _DFRobotL298PShield_CONFIG_H_

