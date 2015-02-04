#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_

#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3.h>

float gyroscope_offset[3] = {0.0,0.0,0.0};
float gyroscope_samples[3] = {0.0,0.0,0.0};
int gyroscope_total_samples = 500;

int reads = 0;
byte buffer[6];

bool check_gyroscope();
bool remove_gyroscope_bias();

geometry_msgs::Vector3 raw_rotation;
geometry_msgs::Vector3 measure_gyroscope();

#if defined(ITG3205)
  #include "gyroscope_ITG3205.h"
#elif defined(L3G4200D)
  #include "gyroscope_L3G4200D.h"
#endif

#endif  // _GYROSCOPE_H_

