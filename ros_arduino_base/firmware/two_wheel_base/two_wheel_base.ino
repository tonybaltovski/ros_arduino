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

#include <ros.h>
#include <ros_arduino_base/UpdateGains.h>
#include <ros_arduino_msgs/Encoders.h>
#include <ros_arduino_msgs/CmdDiffVel.h>

#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <PololuMC33926.h>

#include "user_config.h"
#include "motor_driver_config.h"

typedef struct {
  float desired_velocity;  // [m/s]
  uint32_t current_time;  // [miliseconds]
  uint32_t previous_time;  // [miliseconds]
  int32_t current_encoder;  // [counts]
  int32_t previous_encoder;  // [counts]
  float previous_error;  // 
  float total_error;  // 
  int command; // [PWM]
}
ControlData;

// Encoder objects from PJRC encoder library.
Encoder left_encoder(LEFT_ENCODER_A,LEFT_ENCODER_B);
Encoder right_encoder(RIGHT_ENCODER_A,RIGHT_ENCODER_B);

// Vehicle characteristics
float counts_per_rev[1];
float gear_ratio[1];
int encoder_on_motor_shaft[1];
float wheel_radius[1]; // [m]
float meters_per_counts;  // [m/counts]
int pwm_range[1];

// Gains;
float pid_gains[3];
float Kp, Ki, Kd;

// Structures containing PID data
ControlData left_motor_controller;
ControlData right_motor_controller;

// Control methods prototypes
void updateControl(ControlData * ctrl);
void doControl(ControlData * ctrl);
void Control();

int control_rate[1];  // Hz
int encoder_rate[1];  // Hz
int no_cmd_timeout[1]; // seconds

static uint32_t last_encoders_time;  // miliseconds
static uint32_t last_cmd_time;  // miliseconds
static uint32_t last_control_time;  // miliseconds

// ROS node
ros::NodeHandle_<ArduinoHardware, 10, 10, 1024, 1024> nh;

// ROS subribers/service callbacks prototye
void cmdDiffVelCallback(const ros_arduino_msgs::CmdDiffVel& diff_vel_msg); 

// ROS subsribers
ros::Subscriber<ros_arduino_msgs::CmdDiffVel> sub_diff_vel("cmd_diff_vel", cmdDiffVelCallback);

// ROS services prototype
void updateGainsCb(const ros_arduino_base::UpdateGains::Request &req, ros_arduino_base::UpdateGains::Response &res);
// ROS services
ros::ServiceServer<ros_arduino_base::UpdateGains::Request, ros_arduino_base::UpdateGains::Response> update_gains_server("update_gains", &updateGainsCb);

// ROS publishers msgs
ros_arduino_msgs::Encoders encoders_msg;
// ROS publishers
ros::Publisher pub_encoders("encoders", &encoders_msg);


void setup() 
{ 
  // Set the node handle
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  // Pub/Sub
  nh.advertise(pub_encoders);
  nh.subscribe(sub_diff_vel);
  nh.advertiseService(update_gains_server);
  
  // Wait for ROSserial to connect
  while (!nh.connected()) 
  {
    nh.spinOnce();
  }
  nh.loginfo("Connected to microcontroller.");

  if (!nh.getParam("control_rate", control_rate,1))
  {
    control_rate[0] = 50;
  }
  if (!nh.getParam("encoder_rate", encoder_rate,1))
  {
    encoder_rate[0] = 50;
  }
  if (!nh.getParam("no_cmd_timeout", no_cmd_timeout,1))
  {
    no_cmd_timeout[0] = 1;
  }
  if (!nh.getParam("pid_gains", pid_gains,3))
  { 
    pid_gains[0] = 150;  // Kp
    pid_gains[1] =   0;  // Ki
    pid_gains[2] =  20;  // Kd
  }

  if (!nh.getParam("counts_per_rev", counts_per_rev,1))
  {
    counts_per_rev[0] = 48.0;
  }
  if (!nh.getParam("gear_ratio", gear_ratio,1))
  {
    gear_ratio[0] = 75.0/1.0;
  }
  if (!nh.getParam("encoder_on_motor_shaft", encoder_on_motor_shaft,1))
  {
    encoder_on_motor_shaft[0] = 1;
  }
  if (!nh.getParam("wheel_radius", wheel_radius,1))
  {
    wheel_radius[0] = 0.120/2.0;
  }
  if (!nh.getParam("pwm_range", pwm_range,1))
  {
    pwm_range[0] = 255;
  }

  // Compute the meters per count
  if (encoder_on_motor_shaft[0] == 1)
  {
    meters_per_counts = ((PI * 2 * wheel_radius[0]) / (counts_per_rev[0] * gear_ratio[0]));
  }
  else
  {
    meters_per_counts = ((PI * 2 * wheel_radius[0]) / counts_per_rev[0]);
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
  if ((millis() - last_encoders_time) >= (1000 / encoder_rate[0]))
  { 
    encoders_msg.left = left_encoder.read();
    encoders_msg.right = right_encoder.read();
    pub_encoders.publish(&encoders_msg);
    last_encoders_time = millis();
  }
  if ((millis()) - last_control_time >= (1000 / control_rate[0]))
  {
    Control();
    last_control_time = millis();
  }

  // Stop motors after a period of no commands
  if((millis() - last_cmd_time) >= (no_cmd_timeout[0] * 1000))
  {
    left_motor_controller.desired_velocity = 0.0;
    right_motor_controller.desired_velocity = 0.0;
  }
  nh.spinOnce();
}


void cmdDiffVelCallback( const ros_arduino_msgs::CmdDiffVel& diff_vel_msg) 
{
  left_motor_controller.desired_velocity = diff_vel_msg.left;
  right_motor_controller.desired_velocity = diff_vel_msg.right;
  last_cmd_time = millis();
}

void updateControl()
{
  left_motor_controller.current_encoder = left_encoder.read();
  left_motor_controller.current_time = millis();
  right_motor_controller.current_encoder = right_encoder.read();
  right_motor_controller.current_time = millis();
}
void doControl(ControlData * ctrl)
{
  float estimated_velocity = meters_per_counts * (ctrl->current_encoder - ctrl->previous_encoder) * 1000.0 / (ctrl->current_time - ctrl->previous_time);
  float error = ctrl->desired_velocity - estimated_velocity;
  float cmd = Kp * error + Ki * (error + ctrl->total_error) + Kd * (error - ctrl->previous_error);

  cmd += ctrl->command;

  if(cmd >= pwm_range[0])
  {
    cmd = pwm_range[0];
  }
  else if (cmd <= -pwm_range[0])
  {
    cmd = -pwm_range[0];
  }
  else
  {
    ctrl->total_error += error;
  }

  ctrl->command = cmd;
  ctrl->previous_time = ctrl->current_time;
  ctrl->previous_encoder = ctrl->current_encoder;
  ctrl->previous_error = error;

}

void Control()
{
  updateControl();

  doControl(&left_motor_controller);
  doControl(&right_motor_controller);


  if(left_motor_controller.desired_velocity > 0 || left_motor_controller.desired_velocity < 0)
  {
    commandLeftMotor(left_motor_controller.command);
  }
  else
  {
    commandLeftMotor(0);
  }

  if(right_motor_controller.desired_velocity > 0 || right_motor_controller.desired_velocity < 0)
  {
    commandRightMotor(right_motor_controller.command);
  }
  else
  {
    commandRightMotor(0);
  }
}

void updateGainsCb(const ros_arduino_base::UpdateGains::Request & req, ros_arduino_base::UpdateGains::Response & res)
{
  pid_gains[0] = req.p;
  pid_gains[1] = req.i; 
  pid_gains[2] = req.d;
  
  Kp = pid_gains[0];
  Ki = pid_gains[1] / control_rate[0];
  Kd = pid_gains[2] * control_rate[0];
}




