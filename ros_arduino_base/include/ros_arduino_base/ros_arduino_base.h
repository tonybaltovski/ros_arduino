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

#ifndef _ROS_ARDUINO_BASE_H_
#define _ROS_ARDUINO_BASE_H_

#include <math.h>
#include <ros/ros.h>
#include <ros_arduino_msgs/Encoders.h>
#include <ros_arduino_msgs/CmdDiffVel.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros_arduino_base/MotorGainsConfig.h>
#include <ros_arduino_base/UpdateGains.h>
#include <dynamic_reconfigure/server.h>

class ROSArduinoBase
{
  private:
    ros::NodeHandle nh_, pnh_;
    // Subsribers
    ros::Subscriber encoders_sub_;
    ros::Subscriber cmd_vel_sub_;
    // Publishers
    ros::Publisher odom_pub_;
    ros::Publisher cmd_diff_vel_pub_;
    // Services
    ros::ServiceClient update_gains_client_;
    // Dynamic Reconfigure
    dynamic_reconfigure::Server<ros_arduino_base::MotorGainsConfig> gain_server_;
    // Members
    bool publish_tf_;
    ros::Time encoder_previous_time_;
    tf::TransformBroadcaster *odom_broadcaster_;
    std::string base_frame_, odom_frame_;
    bool custom_covar_;
    double pose_x_stdev_, pose_y_stdev_, pose_yaw_stdev_;
    double twist_x_stdev_, twist_y_stdev_, twist_yaw_stdev_;
    boost::array<double, 36> pose_covar_;
    boost::array<double, 36> twist_covar_;
    // Odom variables
    double x_, y_;                                // [m]
    double theta_;                                // [radians]
    double dx_, dy_;                              // [m/s]
    double dtheta_;                               // [radians/s]
    // Encoder variables
    int32_t right_counts_, left_counts_;          // [counts]
    int32_t old_right_counts_, old_left_counts_;  // [counts]
    // Control variables
    double gains_[3];
    // Vehicle characteristics
    double counts_per_rev_;                       // [counts/rev]
    double gear_ratio_;
    int encoder_on_motor_shaft_;
    double wheel_radius_;                         // [m]
    double base_width_;                           // [m]
    double meters_per_counts_;                    // [m/counts]
    // ROS Member functions 
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& vel_msg);
    void encodersCallback(const ros_arduino_msgs::Encoders::ConstPtr& encoders_msg);
    void motorGainsCallback(ros_arduino_base::MotorGainsConfig &config, uint32_t level);
    // Other Member functions
    void fillCovar(boost::array<double, 36> & covar, double x_stdev, double y_stdev, double yaw_stdev);
  public:
    ROSArduinoBase(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~ROSArduinoBase(void){};
};
#endif // _ROS_ARDUINO_BASE_H_
