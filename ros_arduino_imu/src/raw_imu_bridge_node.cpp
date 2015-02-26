#include "ros_arduino_imu/raw_imu_bridge.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "raw_imu_bridge");
  ros::NodeHandle nh, pnh("~");
  RawImuBridge raw_imu_bridge(nh, pnh);

  ros::spin();

  return 0;
}
