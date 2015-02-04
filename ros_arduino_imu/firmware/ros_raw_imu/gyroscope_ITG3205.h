#ifndef _GYROSCOPE_ITFG3205_H_
#define _GYROSCOPE_ITFG3205_H_

#include <Arduino.h>
#include <Wire.h>
//ITG3205 REGISTERS
#define ITG3205_GYRO_ADDRESS 0x68
#define ITG3205_WHO_AM_I 0x00
#define ITG3205_PWR_MGM 0x3E
#define ITG3205_RESET 0x80
#define ITG3205_DLPF_FS 0x16
#define ITG3205_SCALE 0.00121414209  //rad/s

bool check_gyroscope()
{
  if ((check_ID(ITG3205_GYRO_ADDRESS,ITG3205_WHO_AM_I) & 0x7E) == ITG3205_GYRO_ADDRESS)
  {
    write_to_register(ITG3205_GYRO_ADDRESS,ITG3205_PWR_MGM,ITG3205_RESET); //reset the gyro
    delay(5); 
    write_to_register(ITG3205_GYRO_ADDRESS,ITG3205_DLPF_FS,0x1B); // Set LP filter bandwidth to 42Hz
    delay(5); 
    write_to_register(ITG3205_GYRO_ADDRESS,0x15,0x13); // sample rate to to 50Hz
    delay(5); 
    write_to_register(ITG3205_GYRO_ADDRESS,ITG3205_PWR_MGM,0x03); // PLL with Z gyro ref
    delay(10);
    return true;
  }
  else
    return false;
}

bool remove_gyroscope_bias() {

  for (int samples = 0; samples < gyroscope_total_samples; samples++)
  {
    reads = 0;
    send_value(ITG3205_GYRO_ADDRESS,0x1D);
    Wire.requestFrom(ITG3205_GYRO_ADDRESS,6);
    while(Wire.available())
    {
      buffer[reads] = Wire.read(); 
      reads++;
    }
    gyroscope_samples[GYRO_X_AXIS] +=  (float)(((int16_t)(buffer[2*GYRO_X_AXIS] <<8) | buffer[2*GYRO_X_AXIS+1])) * ITG3205_SCALE;
    gyroscope_samples[GYRO_Y_AXIS] +=  (float)(((int16_t)(buffer[2*GYRO_Y_AXIS] <<8) | buffer[2*GYRO_Y_AXIS+1])) * ITG3205_SCALE;
    gyroscope_samples[GYRO_Z_AXIS] +=  (float)(((int16_t)(buffer[2*GYRO_Z_AXIS] <<8) | buffer[2*GYRO_Z_AXIS+1])) * ITG3205_SCALE;
    nh.spinOnce();
    delay(10);
  }
  gyroscope_offset[GYRO_X_AXIS] = -(gyroscope_samples[GYRO_X_AXIS]/gyroscope_total_samples);
  gyroscope_offset[GYRO_Y_AXIS] = -(gyroscope_samples[GYRO_Y_AXIS]/gyroscope_total_samples);
  gyroscope_offset[GYRO_Z_AXIS] = -(gyroscope_samples[GYRO_Z_AXIS]/gyroscope_total_samples);
  gyroscope_samples[GYRO_X_AXIS] = 0;
  gyroscope_samples[GYRO_Y_AXIS] = 0;
  gyroscope_samples[GYRO_Z_AXIS] = 0;

  return true;
}

geometry_msgs::Vector3 measure_gyroscope()
{
  reads = 0;;
  send_value(ITG3205_GYRO_ADDRESS,0x1D);
  Wire.requestFrom(ITG3205_GYRO_ADDRESS,6);
  while(Wire.available())
  {
    buffer[reads] = Wire.read();
    reads++;
  }
  raw_rotation.x = (float)(GYRO_X_INVERT*((int16_t)(buffer[2*GYRO_X_AXIS] <<8) | buffer[2*GYRO_X_AXIS+1])) * ITG3205_SCALE + gyroscope_offset[GYRO_X_AXIS];  //rad/s
  raw_rotation.y = (float)(GYRO_Y_INVERT*((int16_t)(buffer[2*GYRO_Y_AXIS] <<8) | buffer[2*GYRO_Y_AXIS+1])) * ITG3205_SCALE + gyroscope_offset[GYRO_Y_AXIS];
  raw_rotation.z = (float)(GYRO_X_INVERT*((int16_t)(buffer[2*GYRO_Z_AXIS] <<8) | buffer[2*GYRO_Z_AXIS+1])) * ITG3205_SCALE + gyroscope_offset[GYRO_Z_AXIS];
  return raw_rotation;
}
#endif  // _GYROSCOPE_ITFG3205_H_

