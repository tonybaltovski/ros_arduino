#include <ros/ros.h>
#include <ros_arduino_msgs/RawImu.h>

float mxh, myh, mzh;
float mxl, myl, mzl;

void raw_imu_callback(const ros_arduino_msgs::RawImuConstPtr& raw_msg){

  float mx, my, mz;
  bool update = false;
  
  mx = raw_msg->raw_magnetic_field.x;
  my = raw_msg->raw_magnetic_field.y;
  mz = raw_msg->raw_magnetic_field.z;
  
  if(mx > mxh)
  {
    mxh = mx;
    update = true;
  }
  else if(mx < mxl)
  {
    mxl = mx;
    update = true;
  }
  if(my > myh)
  {
    myh = my;
    update = true;
  }
  else if( my < myl)
  {
    myl = my;
    update = true;
  }
  if(mz > mzh)
  {
    mzh = mz;
    update = true;
  }
  else if(mz < mzl)
  {
    mzl = mz;
    update = true;
  }

  if (update)
  {
    ROS_WARN("\nMin x:%f\tMin y:%f\tMin z:%f\nMax x:%f\tMax y:%f\tMax z:%f",mxl,myl,mzl,mxh,myh,mzh);
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "arduino_magnometer_calibration");
  ros::NodeHandle n;
  
  ros::Subscriber raw_imu_sub = n.subscribe("raw_imu", 1, raw_imu_callback);
  
  ros::spin();
}
