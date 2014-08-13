#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3Stamped.h>

bool use_accelerometer = true;
bool use_gyroscope = true;
bool use_magnetometer = true;

// Magnetometer calibration values.
float M_X_MIN = -574;
float M_Y_MIN = -520;
float M_Z_MIN = -626;
float M_X_MAX =  638;
float M_Y_MAX =  846;
float M_Z_MAX =  552;

ros::Publisher imu_pub;
ros::Publisher mag_pub;

void rawCallback(const ros_arduino_msgs::RawImuConstPtr& raw_msg)
{
  if(!raw_msg->accelerometer && use_accelerometer)
    ROS_ERROR("Accelerometer not found!");
  if(!raw_msg->gyroscope  && use_gyroscope)
    ROS_ERROR("Gyroscope not found!");
  if(!raw_msg->magnetometer && use_magnetometer)
    ROS_ERROR("Magnetometer not found!");


    if(use_accelerometer || use_gyroscope)
    {
      sensor_msgs::Imu imu_msg;
      imu_msg.header = raw_msg->header;
      imu_msg.header.frame_id = "imu";
      imu_msg.angular_velocity.x = raw_msg->raw_angular_velocity.x;
      imu_msg.angular_velocity.y = raw_msg->raw_angular_velocity.y;
      imu_msg.angular_velocity.z = raw_msg->raw_angular_velocity.z;
      imu_msg.linear_acceleration.x = raw_msg->raw_linear_acceleration.x;
      imu_msg.linear_acceleration.y = raw_msg->raw_linear_acceleration.y;
      imu_msg.linear_acceleration.z = raw_msg->raw_linear_acceleration.z;
      imu_pub.publish(imu_msg);
    }

    if(use_magnetometer)
    {
      float mx, my, mz;
      mx = raw_msg->raw_magnetic_field.x;
      my = raw_msg->raw_magnetic_field.y;
      mz = raw_msg->raw_magnetic_field.z;
      geometry_msgs::Vector3Stamped mag_msg;
      mag_msg.header = raw_msg->header;
      mag_msg.header.frame_id = "imu";
      mag_msg.vector.x = (float)(mx - (M_X_MAX - M_X_MIN) / 2 - M_X_MIN);
      mag_msg.vector.y = (float)(my - (M_Y_MAX - M_Y_MIN) / 2 - M_Y_MIN);
      mag_msg.vector.z = (float)(mz - (M_Z_MAX - M_Z_MIN) / 2 - M_Z_MIN);
      mag_pub.publish(mag_msg);
    }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "raw_imu_bridge");
  ros::NodeHandle n;
  // Calibration params for mag
  ros::param::param<float>("mag_x_min", M_X_MIN, M_X_MIN);
  ros::param::param<float>("mag_y_min", M_Y_MIN, M_Y_MIN);
  ros::param::param<float>("mag_z_min", M_Z_MIN, M_Z_MIN);
  ros::param::param<float>("mag_x_max", M_X_MAX, M_X_MAX);
  ros::param::param<float>("mag_y_max", M_Y_MAX, M_Y_MAX);
  ros::param::param<float>("mag_z_max", M_Z_MAX, M_Z_MAX);

  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  ros::Publisher mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1);

  ros::Subscriber raw_sub = n.subscribe("raw_imu", 1, rawCallback);

  ros::spin();
  return 0;
}
