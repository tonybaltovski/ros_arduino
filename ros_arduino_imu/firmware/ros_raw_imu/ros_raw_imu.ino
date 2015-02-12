/*

 Copyright (c) 2013-2015, Tony Baltovski 
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

#include <Arduino.h>

#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

#if defined(WIRE_T3)
  #include <i2c_t3.h>
#else
  #include <Wire.h>
#endif

#include "imu_configuration.h"

uint32_t last_time = 0;
uint8_t update_rate = 50; //Hz

bool is_first = true;

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
  #if defined(WIRE_T3)
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  #else
    Wire.begin();
  #endif

  delay(5);
}


void loop()
{ 
  if (nh.connected())
  {
    if (is_first)
    { 
      raw_imu_msg.accelerometer = check_accelerometer();
      raw_imu_msg.gyroscope = check_gyroscope();
      raw_imu_msg.magnetometer = check_magnetometer();
      
      if (!raw_imu_msg.accelerometer)
      {
        nh.logerror("Accelerometer NOT FOUND!");
      }
      
      if (!raw_imu_msg.gyroscope)
      {
        nh.logerror("Gyroscope NOT FOUND!");
      }
      
      if (!raw_imu_msg.magnetometer)
      {
        nh.logerror("Magnetometer NOT FOUND!");
      }
      
      is_first = false;
    }
    else if (millis() - last_time >= 1000/update_rate)
    {
      raw_imu_msg.header.stamp = nh.now();
      raw_imu_msg.header.frame_id = "imu_link";
      if (raw_imu_msg.accelerometer)
      {
        measure_acceleration();
        raw_imu_msg.raw_linear_acceleration = raw_acceleration;
      }
      
      if (raw_imu_msg.gyroscope)
      {
        measure_gyroscope();
        raw_imu_msg.raw_angular_velocity = raw_rotation;
      }
      
      if (raw_imu_msg.magnetometer)
      {
        measure_magnetometer();
        raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
      }

      raw_imu_pub.publish(&raw_imu_msg);

      last_time = millis();
    }
  }
  nh.spinOnce();
}







