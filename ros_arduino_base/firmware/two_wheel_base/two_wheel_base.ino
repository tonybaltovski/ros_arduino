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

#include <ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <ros_arduino_base/UpdateGains.h>
#include <ros_arduino_msgs/BaseFeedback.h>
#include <ros_arduino_msgs/BaseStatus.h>
#include <ros_arduino_msgs/Drive.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>

/********************************************************************************************
/                                                     USER CONFIG                           *
/********************************************************************************************/

// Select your baud rate here
#define BAUD 115200

// Select your motor driver here
#define PololuMC33926
//#define DFRobotL298PShield

#define TUNING

// Define your encoder pins here.
// Try to use pins that have interrupts
// Left side encoders pins
#define LEFT_ENCODER_A 14  // Interrupt on Teensy 3.0
#define LEFT_ENCODER_B 15  // Interrupt on Teensy 3.0
// Right side encoders pins
#define RIGHT_ENCODER_A 6  // Interrupt on Teensy 3.0
#define RIGHT_ENCODER_B 7  // Interrupt on Teensy 3.0

/********************************************************************************************
/                                                 END OF USER CONFIG                        *
/********************************************************************************************/

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#if defined(PololuMC33926)
  #include <PololuMC33926.h>
#endif

#include "motor_driver_config.h"

typedef struct {
  float desired_velocity;     // [radians/s]
  float estimated_velocity;   // [radians/s]
  uint32_t current_time;      // [milliseconds]
  uint32_t previous_time;     // [milliseconds]
  int32_t current_encoder;    // [counts]
  int32_t previous_encoder;   // [counts]
  float error;                //
  float previous_error;       // 
  float total_error;          // 
  int16_t command;            // [PWM]
}
ControlData;

// Encoder objects from PJRC encoder library.
Encoder left_encoder(LEFT_ENCODER_A,LEFT_ENCODER_B);
Encoder right_encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B);

// Vehicle characteristics
float counts_per_rev[1];
int pwm_range[1];

// Gains;
float pid_gains[3];
float Kp, Ki, Kd;

// Structures containing PID data
ControlData left_motor_controller;
ControlData right_motor_controller;

// Control methods prototypes
void doControl(ControlData * ctrl, int32_t encoder_reading);
void Control();

int control_rate[1];   // [Hz]
int feedback_rate[1];  // [Hz]
int status_rate[1];    // [Hz]
int imu_rate[1];        // [Hz]
int no_cmd_timeout[1]; // [seconds]

uint32_t start_time = millis();
uint32_t last_feedback_time;  // [milliseconds]
uint32_t last_cmd_time;       // [milliseconds]
uint32_t last_control_time;   // [milliseconds]
uint32_t last_status_time;    // [milliseconds]
uint32_t last_imu_time;    // [milliseconds]

// ROS node
ros::NodeHandle_<ArduinoHardware, 3, 3, 1024, 1024> nh;

#if defined(WIRE_T3)
  #include <i2c_t3.h>
#else
  #include <Wire.h>
#endif

#include "imu_configuration.h"
bool imu_is_first = true;

// ROS subribers/service callbacks prototye
void driveCallback(const ros_arduino_msgs::Drive& drive_msg); 

// ROS subsribers
ros::Subscriber<ros_arduino_msgs::Drive> sub_drive("drive", driveCallback);

// ROS services prototype
void updateGainsCb(const ros_arduino_base::UpdateGains::Request &req, ros_arduino_base::UpdateGains::Response &res);
// ROS services
ros::ServiceServer<ros_arduino_base::UpdateGains::Request, ros_arduino_base::UpdateGains::Response> update_gains_server("update_gains", &updateGainsCb);

// ROS publishers msgs
ros_arduino_msgs::BaseFeedback feedback_msg;
ros_arduino_msgs::BaseStatus status_msg;
ros_arduino_msgs::RawImu raw_imu_msg;
char base_id[] = "base_link";
char imu_id[] = "imu_link";

// ROS publishers
ros::Publisher pub_feedback("feedback", &feedback_msg);
ros::Publisher pub_status("status", &status_msg);
ros::Publisher pub_raw_imu("raw_imu", &raw_imu_msg);

void setup() 
{ 
  // Set the node handle
  //nh.getHardware()->setBaud(BAUD);
  nh.initNode();

  feedback_msg.header.frame_id = base_id;
  raw_imu_msg.header.frame_id = imu_id;
  // Pub/Sub
  nh.advertise(pub_feedback);
  nh.advertise(pub_status);
  nh.advertise(pub_raw_imu);
  nh.subscribe(sub_drive);
  nh.advertiseService(update_gains_server);
  
  // Wait for ROSserial to connect
  while (!nh.connected()) 
  {
    nh.spinOnce();
  }
  #if defined(WIRE_T3)
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_400);
  #else
    Wire.begin();
  #endif
  
  nh.loginfo("Connected to microcontroller.");

  if (!nh.getParam("control_rate", control_rate,1))
  {
    control_rate[0] = 100;
  }
  if (!nh.getParam("feedback_rate", feedback_rate,1))
  {
    feedback_rate[0] = 50;
  }
  if (!nh.getParam("status_rate", status_rate,1))
  {
    status_rate[0] = 1;
  }
  if (!nh.getParam("imu_rate", imu_rate,1))
  {
    imu_rate[0] = 50;
  }
  if (!nh.getParam("no_cmd_timeout", no_cmd_timeout,1))
  {
    no_cmd_timeout[0] = 1;
  }
  if (!nh.getParam("pid_gains", pid_gains,3))
  { 
    pid_gains[0] = 150;  // Kp
    pid_gains[1] =   1;  // Ki
    pid_gains[2] =  20;  // Kd
  }

  if (!nh.getParam("counts_per_rev", counts_per_rev,1))
  {
    counts_per_rev[0] = 48.0;
  }
  if (!nh.getParam("pwm_range", pwm_range,1))
  {
    pwm_range[0] = 255;
  }

  // Create PID gains for this specific control rate
  Kp = pid_gains[0];
  Ki = pid_gains[1] / control_rate[0];
  Kd = pid_gains[2] * control_rate[0];
  
  // Initialize the motors
  setupMotors();
} 


void loop() 
{
  // Stop motors after a period of no commands
  if((millis() - last_cmd_time) >= (no_cmd_timeout[0] * 1000))
  {
    left_motor_controller.desired_velocity = 0.0;
    right_motor_controller.desired_velocity = 0.0;
  }
  if ((millis() - last_control_time) >= (1000 / control_rate[0]))
  {
    Control();
    last_control_time = millis();
  }
  

  if (nh.connected())
  {
    if ((millis() - last_feedback_time) >= (1000 / feedback_rate[0]))
    { 
      feedback_msg.motors[ros_arduino_msgs::BaseFeedback::LEFT].estimated_distance = left_encoder.read() * (2*PI / counts_per_rev[0]);
      feedback_msg.motors[ros_arduino_msgs::BaseFeedback::LEFT].estimated_velocity = left_motor_controller.estimated_velocity;
      feedback_msg.motors[ros_arduino_msgs::BaseFeedback::LEFT].fault = left_motor.fault();
      //feedback_msg.motors[ros_arduino_msgs::BaseFeedback::LEFT].current_draw = left_motor.motor_current();
     
      feedback_msg.motors[ros_arduino_msgs::BaseFeedback::RIGHT].estimated_distance = right_encoder.read() * (2*PI / counts_per_rev[0]);
      feedback_msg.motors[ros_arduino_msgs::BaseFeedback::RIGHT].estimated_velocity = right_motor_controller.estimated_velocity;
      feedback_msg.motors[ros_arduino_msgs::BaseFeedback::RIGHT].fault = right_motor.fault();
      //feedback_msg.motors[ros_arduino_msgs::BaseFeedback::RIGHT].current_draw = right_motor.motor_current();
      
      feedback_msg.header.stamp = nh.now();
      pub_feedback.publish(&feedback_msg);
      last_feedback_time = millis();
    }
    
    if ((millis() - last_status_time) >= (1000 / status_rate[0]))
    {
      status_msg.mcu_uptime.fromSec((millis() - start_time) / 1000);
      pub_status.publish(&status_msg);
      last_status_time = millis();
    }
    
    if (imu_is_first)
    { 
      status_msg.accelerometer = check_accelerometer();
      status_msg.gyroscope = check_gyroscope();
      status_msg.magnetometer = check_magnetometer();
      
      if (!status_msg.accelerometer)
      {
        nh.logerror("Accelerometer NOT FOUND!");
      }
      
      if (!status_msg.gyroscope)
      {
        nh.logerror("Gyroscope NOT FOUND!");
      }
      
      if (!status_msg.magnetometer)
      {
        nh.logerror("Magnetometer NOT FOUND!");
      }
      
      imu_is_first = false;
    }
    else if ((millis() - last_imu_time) >= (1000 / imu_rate[0]))
    {
      raw_imu_msg.header.stamp = nh.now();
      if (status_msg.accelerometer)
      {
        measure_acceleration();
        raw_imu_msg.raw_linear_acceleration = raw_acceleration;
      }
      
      if (status_msg.gyroscope)
      {
        measure_gyroscope();
        raw_imu_msg.raw_angular_velocity = raw_rotation;
      }
      
      if (status_msg.magnetometer)
      {
        measure_magnetometer();
        raw_imu_msg.raw_magnetic_field = raw_magnetic_field;
      }

      pub_raw_imu.publish(&raw_imu_msg);

      last_imu_time = millis();
    }
  }
  
  nh.spinOnce();
}


void driveCallback( const ros_arduino_msgs::Drive& drive_msg) 
{
  left_motor_controller.desired_velocity = drive_msg.drivers[ros_arduino_msgs::Drive::LEFT];
  right_motor_controller.desired_velocity = drive_msg.drivers[ros_arduino_msgs::Drive::RIGHT];
  last_cmd_time = millis();
}


void doControl(ControlData * ctrl, int32_t encoder_reading)
{
  ctrl->current_encoder = encoder_reading;
  ctrl->current_time = millis();
  ctrl->estimated_velocity = (2*PI / counts_per_rev[0]) * (ctrl->current_encoder - ctrl->previous_encoder) * 1000.0 / (ctrl->current_time - ctrl->previous_time);
  ctrl->error = ctrl->desired_velocity - ctrl->estimated_velocity;
  ctrl->command += Kp * ctrl->error + Ki * (ctrl->error + ctrl->total_error) + Kd * (ctrl->error - ctrl->previous_error);

  if(ctrl->command >= pwm_range[0])
  {
    ctrl->command = pwm_range[0];
  }
  else if (ctrl->command <= -pwm_range[0])
  {
    ctrl->command = -pwm_range[0];
  }
  else
  {
    ctrl->total_error += ctrl->error;
  }

  ctrl->previous_time = ctrl->current_time;
  ctrl->previous_encoder = ctrl->current_encoder;
  ctrl->previous_error = ctrl->error;

}

void Control()
{
  doControl(&left_motor_controller, left_encoder.read());
  doControl(&right_motor_controller, right_encoder.read());

  commandLeftMotor(left_motor_controller.command);
  commandRightMotor(right_motor_controller.command);
}

void updateGainsCb(const ros_arduino_base::UpdateGains::Request &req, ros_arduino_base::UpdateGains::Response &res)
{
  Kp = req.gains[0];
  Ki = req.gains[1] / control_rate[0];
  Kd = req.gains[2] * control_rate[0];
}




