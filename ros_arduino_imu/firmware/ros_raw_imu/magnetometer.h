#ifndef _MEGNETOMETER_H_
#define _MEGNETOMETER_H_

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>

#if defined(HMC5883L)
#include "magnetometer_HMC5883L.h"
#endif

bool check_magnetometer();
geometry_msgs::Vector3 measure_magnetometer();


#endif

