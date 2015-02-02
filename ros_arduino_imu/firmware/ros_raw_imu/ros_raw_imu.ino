/*

 Copyright (c) 2013, Tony Baltovski 
 All rights reserved. 
 
 Redistribution and use in source and binary forms, with or without 
 modification, are permitted provided that the following conditions are met: 
 
 * Redistributions of source code must retain the above copyright notice, 
 this list of conditions and the following disclaimer. 
 * Redistributions in binary form must reproduce the above copyright 
 notice, this list of conditions and the following disclaimer in the 
 documentation and/or other materials provided with the distribution. 
 * Neither the name of  nor the names of its contributors may be used to 
 endorse or promote products derived from this software without specific 
 prior written permission. 
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 POSSIBILITY OF SUCH DAMAGE. 
 
 */

#include <Wire.h>

#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

#include "imu_configuration.h"

ros::NodeHandle nh;

uint32_t last_time = 0;
uint32_t update_rate = 200; //Hz

bool is_first = true;
bool is_accelerometer_calibrated = false;
bool is_gyroscope_calibrated = false;

ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

void setup()
{
  nh.getHardware()->setBaud(BAUD);
  nh.initNode();
  nh.advertise(raw_imu_pub);
  // Wait for ROSserial to connect
  while (!nh.connected()) 
  {
    nh.spinOnce();
  }
  nh.loginfo("ROS Arduino IMU started.");
  Wire.begin();                  
  delay(15);
}


void loop()
{ 
  if (nh.connected())
  {
    if (is_first)
    {
      nh.logwarn("Calibrating IMU!");
      nh.logwarn("Sync may be lost.");
      
      raw_imu_msg.accelerometer = check_accelerometer();
      raw_imu_msg.gyroscope = check_gyroscope();
      raw_imu_msg.magnetometer = check_magnetometer();
      is_first = false;
    }
    else if (millis() - last_time >= 1000/update_rate)
    {
      if(!is_accelerometer_calibrated && !is_gyroscope_calibrated)
      {
        is_accelerometer_calibrated = remove_acceleration_bias();
        is_gyroscope_calibrated = remove_gyroscope_bias();
        nh.logwarn("IMU Calibration Complete");
      }
      else
      {
        raw_imu_msg.header.stamp = nh.now();
        raw_imu_msg.header.frame_id = "imu_link";
        if (raw_imu_msg.accelerometer)
        {
          raw_imu_msg.raw_linear_acceleration = measure_acceleration();
        }
        else
        {
          nh.logerror("Accelerometer NOT FOUND!");
        }
        
        if (raw_imu_msg.gyroscope)
        {
          raw_imu_msg.raw_angular_velocity = measure_gyroscope();
        }
        else
        {
          nh.logerror("Gyroscope NOT FOUND!");
        }
        
        if (raw_imu_msg.magnetometer)
        {
          raw_imu_msg.raw_magnetic_field = measure_magnetometer();
        }
        else
        {
          nh.logerror("Magnetometer NOT FOUND!");
        }

        raw_imu_pub.publish(&raw_imu_msg);
      }
      last_time = millis();
    }
  }
  nh.spinOnce();
}







