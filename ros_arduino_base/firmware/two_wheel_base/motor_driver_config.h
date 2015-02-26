#ifndef _MOTOR_DRIVER_CONFIG_H_
#define _MOTOR_DRIVER_CONFIG_H_

void setupMotors();
void commandLeftMotor(int16_t cmd);
void commandRightMotor(int16_t cmd);

#if defined(PololuMC33926)
  #include "PololuMC33926_config.h"
#elif defined(DFRobotL298PShield)
  #include "DFRobotL298PShield_config.h"
#endif

#endif  // _MOTOR_DRIVER_CONFIG_H_
