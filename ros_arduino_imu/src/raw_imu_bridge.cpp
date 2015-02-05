#include "ros_arduino_imu/raw_imu_bridge.h"

RawImuBridge::RawImuBridge(ros::NodeHandle nh, ros::NodeHandle pnh):
  //  Members default values
  nh_(nh),
  pnh_(pnh),
  use_accelerometer_(true),
  use_gyroscope_(true),
  use_magnetometer_(true),
  use_mag_msg_(false)
{
  pnh.param<bool>("imu/use_accelerometer", use_accelerometer_, use_accelerometer_);
  pnh.param<bool>("imu/use_gyroscope", use_gyroscope_, use_gyroscope_);
  pnh.param<bool>("imu/use_magnetometer", use_magnetometer_, use_magnetometer_);
  pnh.param<bool>("imu/use_mag_msg", use_mag_msg_, use_mag_msg_);
  pnh.param<std::string>("imu/frame_id", frame_id_, "imu");

  pnh.param<double>("imu/linear_acc_stdev", linear_acc_stdev_, 0.0);
  pnh.param<double>("imu/angular_vel_stdev", angular_vel_stdev_, 0.0);
  pnh.param<double>("imu/linear_acc_stdev", magnetic_field_stdev_, 0.0);

  raw_sub_ = nh_.subscribe("raw_imu", 1, &RawImuBridge::rawCallback, this);
  
  if(use_accelerometer_ || use_gyroscope_)
  {
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    RawImuBridge::fillRowMajor(linear_acc_covar_, linear_acc_stdev_);
    RawImuBridge::fillRowMajor(angular_vel_covar_, angular_vel_stdev_);
  }

  if(use_magnetometer_)
  {
    if(use_mag_msg_)
    {
      mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
      RawImuBridge::fillRowMajor(magnetic_field_covar_, magnetic_field_stdev_);
    }
    else
    {
      mag_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 1);
    }
  }
  

  // Fill covar matrices
  // Magnetometer calibration values.
  pnh.param<double>("mag/x/min", mag_x_min_, -520);
  pnh.param<double>("mag/x/max", mag_x_max_,  846);
  pnh.param<double>("mag/y/min", mag_y_min_, -574);
  pnh.param<double>("mag/y/max", mag_y_max_,  638);
  pnh.param<double>("mag/z/min", mag_z_min_, -626);
  pnh.param<double>("mag/z/max", mag_z_max_,  552);

  ROS_INFO("Starting Raw Imu Bridge.");
}

RawImuBridge::~RawImuBridge()
{

}

void RawImuBridge::rawCallback(const ros_arduino_msgs::RawImuConstPtr& raw_msg)
{
  if(!raw_msg->accelerometer && use_accelerometer_)
  {
    ROS_ERROR_ONCE("Accelerometer not found!");
  }
  if(!raw_msg->gyroscope && use_gyroscope_)
  {
    ROS_ERROR_ONCE("Gyroscope not found!");
  }
  if(!raw_msg->magnetometer && use_magnetometer_)
  {
    ROS_ERROR_ONCE("Magnetometer not found!");
  }

    if(use_accelerometer_ || use_gyroscope_)
    {
      sensor_msgs::Imu imu_msg;
      imu_msg.header = raw_msg->header;
      imu_msg.header.frame_id = frame_id_;
      imu_msg.angular_velocity.x = raw_msg->raw_angular_velocity.x;
      imu_msg.angular_velocity.y = raw_msg->raw_angular_velocity.y;
      imu_msg.angular_velocity.z = raw_msg->raw_angular_velocity.z;
      imu_msg.orientation_covariance = angular_vel_covar_;
      imu_msg.linear_acceleration.x = raw_msg->raw_linear_acceleration.x;
      imu_msg.linear_acceleration.y = raw_msg->raw_linear_acceleration.y;
      imu_msg.linear_acceleration.z = raw_msg->raw_linear_acceleration.z;
      imu_msg.linear_acceleration_covariance = linear_acc_covar_;
      imu_pub_.publish(imu_msg);
    }

    if(use_magnetometer_)
    {
      double mx, my, mz;
      mx = raw_msg->raw_magnetic_field.x;
      my = raw_msg->raw_magnetic_field.y;
      mz = raw_msg->raw_magnetic_field.z;
      if(use_mag_msg_)
      {
        sensor_msgs::MagneticField mag_msg;
        mag_msg.header = raw_msg->header;
        mag_msg.header.frame_id = frame_id_;
        mag_msg.magnetic_field.x = (double)(mx - (mag_x_max_ - mag_x_min_) / 2 - mag_x_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg.magnetic_field.y = (double)(my - (mag_y_max_ - mag_y_min_) / 2 - mag_y_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg.magnetic_field.z = (double)(mz - (mag_z_max_ - mag_z_min_) / 2 - mag_z_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg.magnetic_field_covariance = magnetic_field_covar_;
        mag_pub_.publish(mag_msg);
      }
      else
      {
        geometry_msgs::Vector3Stamped mag_msg;
        mag_msg.header = raw_msg->header;
        mag_msg.header.frame_id = frame_id_;
        mag_msg.vector.x = (double)(mx - (mag_x_max_ - mag_x_min_) / 2 - mag_x_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg.vector.y = (double)(my - (mag_y_max_ - mag_y_min_) / 2 - mag_y_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_msg.vector.z = (double)(mz - (mag_z_max_ - mag_z_min_) / 2 - mag_z_min_) * MILIGAUSS_TO_TESLA_SCALE;
        mag_pub_.publish(mag_msg);
      }
    }
}

void RawImuBridge::fillRowMajor(boost::array<double, 9> & covar, double stdev)
{
  std::fill(covar.begin(), covar.end(), 0.0);
  covar[0] = pow(stdev, 2);  // X(roll)
  covar[4] = pow(stdev, 2);  // Y(pitch)
  covar[8] = pow(stdev, 2);  // Z(yaw)
}
