#ifndef _MAGNETOMETER_H_
#define _MAGNETOMETER_H_

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>

uint8_t mag_reads = 0;
byte mag_buffer[6];
geometry_msgs::Vector3 raw_magnetic_field;

bool check_magnetometer();
geometry_msgs::Vector3 measure_magnetometer();

#if defined(HMC5883L)
  #include "magnetometer_HMC5883L.h"
#endif

#endif  // _MAGNETOMETER_H_

