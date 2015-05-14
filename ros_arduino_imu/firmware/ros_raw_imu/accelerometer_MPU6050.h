#ifndef _ACCELEROMETER_MPU6050_H_
#define _ACCELEROMETER_MPU6050_H_

#include "accelerometer.h"

//MPU6050 REGISTERS
#define MPU6050_DEVID 0x00
#define MPU6050_DATAX0 0x3B
#define MPU6050_ACCELEROMETER_ADDRESS 0x68
#define MPU6050_DEVICE_ID 0x34
#define MPU6050_SCALE 1.000000

bool check_accelerometer()
{
  if (check_ID(MPU6050_ACCELEROMETER_ADDRESS,MPU6050_DEVID) == MPU6050_DEVICE_ID)
  {
    return true;
  }
  else
    return false;
}

void measure_acceleration()
{
  acc_reads = 0;
  send_value(MPU6050_ACCELEROMETER_ADDRESS,MPU6050_DATAX0);
  Wire.requestFrom(MPU6050_ACCELEROMETER_ADDRESS, 6);
  while(Wire.available())
  {
    acc_buffer[acc_reads] = Wire.read();
    acc_reads++;
  }

  raw_acceleration.x =  ((float)ACC_X_INVERT*(int16_t)((int)acc_buffer[2*ACC_X_AXIS]<<8 | acc_buffer[2*ACC_X_AXIS+1]) / MPU6050_SCALE);
  raw_acceleration.y =  ((float)ACC_Y_INVERT*(int16_t)((int)acc_buffer[2*ACC_Y_AXIS]<<8 | acc_buffer[2*ACC_Y_AXIS+1]) / MPU6050_SCALE);
  raw_acceleration.z =  ((float)ACC_Z_INVERT*(int16_t)((int)acc_buffer[2*ACC_Z_AXIS]<<8 | acc_buffer[2*ACC_Z_AXIS+1]) / MPU6050_SCALE);
  Wire.endTransmission();
}

#endif  // _ACCELEROMETER_MPU6050_H_




