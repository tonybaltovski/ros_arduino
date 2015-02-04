// FINISH THIS AND TEST
#ifndef _GYROSCOPE_L3G4200D_H_
#define _GYROSCOPE_L3G4200D_H_

#include <Arduino.h>

//L3G4200D REGISTERS
#define L3G4200D_GYRO_ADDRESS 0x69
#define L3G4200D_WHO_AM_I 0x0F
#define L3G4200D_WHO_AM_I_VALUE 0xd3
#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_CTRL_REG4 0x23
#define L3G4200D_CTRL_REG5 0x24
#define L3G4200D_SCALE 939.275077264 //rad/s

bool check_gyroscope()
{
  if (check_ID(L3G4200D_GYRO_ADDRESS,L3G4200D_WHO_AM_I) == L3G4200D_WHO_AM_I_VALUE)
  {
    write_to_register(L3G4200D_GYRO_ADDRESS,L3G4200D_CTRL_REG1,0x0F); //xyz and no power down
    delay(10); 
    write_to_register(L3G4200D_GYRO_ADDRESS,L3G4200D_CTRL_REG4,0x30); //full scale at 2000 dps
    delay(10);
    write_to_register(L3G4200D_GYRO_ADDRESS,L3G4200D_CTRL_REG5,0xf0); // hpf
    delay(20);
    return true;
  }
  else
    return false;
}

bool remove_gyroscope_bias() {

  for (int samples = 0; samples < gyroscope_total_samples; samples++)
  {
    reads = 0;
    send_value(L3G4200D_GYRO_ADDRESS,0x80 | 0x28);
    Wire.requestFrom(L3G4200D_GYRO_ADDRESS,6);
    while(Wire.available())
    {
      buffer[reads] = Wire.read(); 
      reads++;
    }
    gyroscope_samples[GYRO_X_AXIS] +=  (float)(((int16_t)(buffer[2*GYRO_X_AXIS+1] <<8) | buffer[2*GYRO_X_AXIS])) / L3G4200D_SCALE;
    gyroscope_samples[GYRO_Y_AXIS] +=  (float)(((int16_t)(buffer[2*GYRO_Y_AXIS+1] <<8) | buffer[2*GYRO_Y_AXIS])) / L3G4200D_SCALE;
    gyroscope_samples[GYRO_Z_AXIS] +=  (float)(((int16_t)(buffer[2*GYRO_Z_AXIS+1] <<8) | buffer[2*GYRO_Z_AXIS])) / L3G4200D_SCALE;
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
  reads = 0;
  send_value(L3G4200D_GYRO_ADDRESS,0x80 | 0x28);
  Wire.requestFrom(L3G4200D_GYRO_ADDRESS,6);
  while(Wire.available())
  {
    buffer[reads] = Wire.read();
    reads++;
  }
  raw_rotation.x = (float)(GYRO_X_INVERT*((int16_t)(buffer[2*GYRO_X_AXIS+1] <<8) | buffer[2*GYRO_X_AXIS])) / L3G4200D_SCALE + gyroscope_offset[GYRO_X_AXIS];  //rad/s
  raw_rotation.y = (float)(GYRO_Y_INVERT*((int16_t)(buffer[2*GYRO_Y_AXIS+1] <<8) | buffer[2*GYRO_Y_AXIS])) / L3G4200D_SCALE + gyroscope_offset[GYRO_Y_AXIS];
  raw_rotation.z = (float)(GYRO_X_INVERT*((int16_t)(buffer[2*GYRO_Z_AXIS+1] <<8) | buffer[2*GYRO_Z_AXIS])) / L3G4200D_SCALE + gyroscope_offset[GYRO_Z_AXIS];
  return raw_rotation;
}

#endif  // _GYROSCOPE_L3G4200D_H_
