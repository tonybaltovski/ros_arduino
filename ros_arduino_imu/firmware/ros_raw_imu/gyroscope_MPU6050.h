// FINISH THIS AND TEST
#ifndef _GYROSCOPE_MPU6050_H_
#define _GYROSCOPE_MPU6050_H_

//MPU6050 REGISTERS
#define MPU6050_GYRO_ADDRESS 0x34
#define MPU6050_GYRO_DATAX0 0x3B
#define MPU6050_GYRO_ADDRESS 0x68
#define MPU6050_SCALE 939.275077264 //rad/s

bool check_gyroscope()
{
  if (check_ID(MPU6050_GYRO_ADDRESS,MPU6050_WHO_AM_I) == MPU6050_WHO_AM_I_VALUE)
  {
    return true;
  }
  else
    return false;
}

void measure_gyroscope()
{
  send_value(MPU6050_GYRO_ADDRESS,MPU6050_GYRO_DATAX0);
  //The driver for this chip reads 14 bits, and only the the last 6. There is a two bit gap between the two
  Wire.requestFrom(MPU6050_GYRO_ADDRESS,14);
  gyro_reads = 0;
  
  while(Wire.available())
  {
    gyro_buffer[gyro_reads] = Wire.read();
    gyro_reads++;
  }
  raw_rotation.x = (float)(GYRO_X_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_X_AXIS+8] <<8) | gyro_buffer[2*GYRO_X_AXIS+9])) / MPU6050_SCALE;  //rad/s
  raw_rotation.y = (float)(GYRO_Y_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Y_AXIS+8] <<8) | gyro_buffer[2*GYRO_Y_AXIS+9])) / MPU6050_SCALE;
  raw_rotation.z = (float)(GYRO_Z_INVERT*(int16_t)(((int)gyro_buffer[2*GYRO_Z_AXIS+8] <<8) | gyro_buffer[2*GYRO_Z_AXIS+9])) / MPU6050_SCALE;

}

#endif  // _GYROSCOPE_MPU6050_H_
