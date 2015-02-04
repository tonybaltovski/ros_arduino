#ifndef _ACCELEROMETER_H_
#define _ACCELEROMETER_H_

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>

float acceleration_bias[3]= {0.0,0.0,0.0};
float acceleration_samples[3] = {0.0,0.0,0.0};
uint16_t acceleration_total_samples = 500;

uint8_t acc_reads = 0;
byte acc_buffer[6];

bool remove_acceleration_bias();
bool check_accelerometer();

geometry_msgs::Vector3 measure_acceleration();
geometry_msgs::Vector3 raw_acceleration;
  
#if defined(ADXL345)
  #include "accelerometer_ADXL345.h"
#endif

#endif  // _ACCELEROMETER_H_

