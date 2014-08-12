#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>

float acceleration_bias[3]= {
  0.0,0.0,0.0};
float acceleration_samples[3] = {
  0.0,0.0,0.0};

bool remove_acceleration_bias();
bool check_accelerometer();
geometry_msgs::Vector3 measure_acceleration();

#if defined(ADXL345)
#include "accelerometer_ADXL345.h"
#endif

#endif

