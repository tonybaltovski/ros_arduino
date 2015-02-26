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
#include <math.h>
#include "ros_arduino_base/ros_arduino_base.h"

ROSArduinoBase::ROSArduinoBase(ros::NodeHandle nh, ros::NodeHandle pnh):
  nh_(nh),
  pnh_(pnh),
  x_(0.0),
  y_(0.0),
  theta_(0.0),
  dx_(0.0),
  dy_(0.0),
  dtheta_(0.0),
  right_counts_(0),
  left_counts_(0),
  old_right_counts_(0),
  old_left_counts_(0)
{
  cmd_diff_vel_pub_ = nh_.advertise<ros_arduino_msgs::CmdDiffVel>("cmd_diff_vel", 5);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 5);
  encoders_sub_ = nh_.subscribe("encoders", 5, &ROSArduinoBase::encodersCallback, this);
  cmd_vel_sub_ = nh_.subscribe("cmd_vel", 5, &ROSArduinoBase::cmdVelCallback, this);
  update_gains_client_ = nh.serviceClient<ros_arduino_base::UpdateGains>("update_gains");
  dynamic_reconfigure::Server<ros_arduino_base::MotorGainsConfig>::CallbackType 
    f = boost::bind(&ROSArduinoBase::motorGainsCallback, this, _1, _2);
  gain_server_.setCallback(f);

  // ROS driver params
  pnh_.param<bool>("publish_tf", publish_tf_, false);
  pnh_.param<std::string>("odom/odom_frame_id", odom_frame_, "odom");
  pnh_.param<std::string>("odom/base_frame_id", base_frame_, "base_link");
  pnh_.param<double>("counts_per_rev", counts_per_rev_, 48.0);
  pnh_.param<double>("gear_ratio", gear_ratio_, (75.0 / 1.0));
  pnh_.param<int>("encoder_on_motor_shaft", encoder_on_motor_shaft_, 1);
  pnh_.param<double>("wheel_radius", wheel_radius_, (0.120 / 2.0));
  pnh_.param<double>("base_width", base_width_ , 0.225);

  if (encoder_on_motor_shaft_ == 1)
  {
    meters_per_counts_ = ((M_PI * 2 * wheel_radius_) / (counts_per_rev_ * gear_ratio_));
  }
  else
  {
    meters_per_counts_ = ((M_PI * 2 * wheel_radius_) / counts_per_rev_);
  }

  pnh_.param<bool>("custom_covar", custom_covar_, false);
  if(custom_covar_)
  {
    pnh_.param<double>("pose_stdev/x", pose_x_stdev_, 0.001);
    pnh_.param<double>("pose_stdev/y", pose_y_stdev_, 0.001);
    pnh_.param<double>("pose_stdev/yaw", pose_yaw_stdev_, 0.001);
    pnh_.param<double>("twist_stdev/x", twist_x_stdev_, 0.01);
    pnh_.param<double>("twist_stdev/y", twist_y_stdev_, 0.01);
    pnh_.param<double>("twist_stdev/yaw", twist_yaw_stdev_, 0.01);

    ROSArduinoBase::fillCovar(pose_covar_, pose_x_stdev_, pose_y_stdev_, pose_yaw_stdev_);
    ROSArduinoBase::fillCovar(twist_covar_, twist_x_stdev_, twist_y_stdev_, twist_yaw_stdev_);
  }


  ROS_INFO("Starting ROS Arduino Base");
}

void ROSArduinoBase::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg)
{
  ros_arduino_msgs::CmdDiffVel cmd_diff_vel_msg;
  // Convert to velocity to each wheel
  cmd_diff_vel_msg.right = (vel_msg->linear.x + ((base_width_ /  2) * vel_msg->angular.z));
  cmd_diff_vel_msg.left  = (vel_msg->linear.x + ((base_width_ / -2) * vel_msg->angular.z));
  cmd_diff_vel_pub_.publish(cmd_diff_vel_msg);
}

void ROSArduinoBase::motorGainsCallback(ros_arduino_base::MotorGainsConfig &config, uint32_t level) 
{
  gains_[0] = config.K_P;
  gains_[1] = config.K_I;
  gains_[2] = config.K_D;

  ros_arduino_base::UpdateGains srv;

  for (int i = 0; i < 3; i++)
  {
    srv.request.gains[i] = gains_[i];
  }

  if (update_gains_client_.call(srv))
  {
    ROS_INFO("Motor Gains changed to P:%f I:%f D: %f", gains_[0], gains_[1], gains_[2]);
  }
  else
  {
    ROS_ERROR("Failed to update gains");
  }

}

void ROSArduinoBase::encodersCallback(const ros_arduino_msgs::Encoders::ConstPtr& encoders_msg)
{
  nav_msgs::Odometry odom;
  left_counts_ = encoders_msg->left;
  right_counts_ = encoders_msg->right;

  double dt = encoders_msg->header.stamp.toSec() - encoder_previous_time_.toSec();                  // [seconds]
  double velocity_estimate_left_ = meters_per_counts_ * (left_counts_ - old_left_counts_) / dt;     // [m/s]
  double velocity_estimate_right_ = meters_per_counts_ * (right_counts_ - old_right_counts_) / dt;  // [m/s]
  double delta_s = meters_per_counts_ * (((right_counts_ - old_right_counts_)
                                          + (left_counts_ - old_left_counts_)) / 2.0);              // [m]
  double delta_theta = meters_per_counts_ * (((right_counts_ - old_right_counts_)
                                          - (left_counts_ - old_left_counts_)) / base_width_);      // [radians]
  double dx = delta_s * cos(theta_ + delta_theta / 2.0);                                            // [m]
  double dy = delta_s * sin(theta_ + delta_theta / 2.0);                                            // [m]
  x_ += dx;                                                                                         // [m]
  y_ += dy;                                                                                         // [m]
  theta_ += delta_theta;                                                                            // [radians]
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);

  if (publish_tf_)
  {
    odom_broadcaster_ = new tf::TransformBroadcaster();
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = encoders_msg->header.stamp;
    odom_trans.header.frame_id = odom_frame_;
    odom_trans.child_frame_id = base_frame_;
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_->sendTransform(odom_trans);
  }

  odom.header.stamp = encoders_msg->header.stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = dx / dt;
  odom.twist.twist.linear.y = dy / dt;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = delta_theta / dt;

  if(custom_covar_)
  {
    odom.pose.covariance = pose_covar_;
    odom.twist.covariance = twist_covar_;
  }

  odom_pub_.publish(odom);

  // Keep track of previous variables.
  encoder_previous_time_ = encoders_msg->header.stamp;
  old_right_counts_ = right_counts_;
  old_left_counts_ = left_counts_;
}

void ROSArduinoBase::fillCovar(boost::array<double, 36> & covar, double x_stdev, double y_stdev, double yaw_stdev)
{
  std::fill(covar.begin(), covar.end(), 0.0);
  covar[0]  = pow(x_stdev, 2);    // X
  covar[7]  = pow(y_stdev, 2);    // Y
  covar[14] = 1e6;                // Z
  covar[21] = 1e6;                // roll
  covar[28] = 1e6;                // pitch
  covar[35] = pow(yaw_stdev, 2);  // yaw(theta)
}
