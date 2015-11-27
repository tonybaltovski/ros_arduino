
/*
 * rosserial imu node
 * grabs data off of the I2C connection to the IMU and publishes it to an IMU topic
 */

#include <Arduino.h>

#include <ros.h>
#include <ros/time.h>
#include <ros_arduino_msgs/RawImu.h>
#include <geometry_msgs/Vector3.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;


//ROS serial node for publishing imu data
ros::NodeHandle  nh;
ros_arduino_msgs::RawImu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

uint32_t last_time = 0;
uint8_t update_rate = 50; //Hz

bool is_first = true;

void setup()
{
    nh.getHardware()->setBaud(115200);
    //initalize the ros node
    nh.initNode();
    nh.advertise(raw_imu_pub);
    // Wait for ROSserial to connect
    while (!nh.connected()) 
    {
      nh.spinOnce();
    }
    nh.loginfo("ROS Arduino IMU started.");
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    //Serial.begin(38400);

    // initialize device
    //Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    delay(5);
    // verify connection
    //Serial.println("Testing device connections...");
    //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop()
{
  if (nh.connected()){
    if (is_first)
    { 
      raw_imu_msg.accelerometer = accelgyro.testConnection();
      raw_imu_msg.gyroscope = accelgyro.testConnection();
      raw_imu_msg.magnetometer = (accelgyro.checkMagStatus()==0x00);
      
      
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
    // publish the range value every 50 milliseconds
    //   since it takes that long for the sensor to stabilize
    else if (millis() - last_time >= 1000/update_rate){
       int16_t ax, ay, az;
       int16_t gx, gy, gz;
       int16_t mx, my, mz;
       // read raw accel/gyro measurements from device
       accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    
       raw_imu_msg.header.stamp = nh.now();
       raw_imu_msg.header.frame_id = "imu_link";
       //take linear acceleration off of the accelerometer
       //assumes units are in m/s^2
       raw_imu_msg.raw_linear_acceleration.x = (float)2*9.81*ax/32767.0;
       raw_imu_msg.raw_linear_acceleration.y = (float)2*9.81*ay/32767.0;
       raw_imu_msg.raw_linear_acceleration.z = (float)2*9.81*az/32767.0;
       //take angular velocity off of the gyroscope, assuming rad/sec
       raw_imu_msg.raw_angular_velocity.x = (float)4.36332313*gx/32767.0;
       raw_imu_msg.raw_angular_velocity.y = (float)4.36332313*gy/32767.0;
       raw_imu_msg.raw_angular_velocity.z = (float)4.36332313*gz/32767.0;
       
       raw_imu_msg.raw_magnetic_field.x = (float)1.2e-3*mx/32767.0;
       raw_imu_msg.raw_magnetic_field.y = (float)1.2e-3*my/32767.0;
       raw_imu_msg.raw_magnetic_field.z = (float)1.2e-3*mz/32767.0;
        
        raw_imu_msg.header.stamp = nh.now();
        raw_imu_msg.header.frame_id = "imu_link";
       
       //orientation is left as a filter for ros
       raw_imu_pub.publish(&raw_imu_msg);
       last_time =  millis();
    }
  }
  nh.spinOnce();
}
