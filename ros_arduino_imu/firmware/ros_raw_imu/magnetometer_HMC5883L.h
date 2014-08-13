#ifndef _MAGNETOMETER_HMC5883L_H_
#define _MAGNETOMETER_HMC5883L_H_

#define HMC5883L_MAG_ADDRESS 0x1E
#define HMC5883L_MAG_ID 0x10
#define HMC5883L_MAG_REG_A 0x00
#define HMC5883L_MAG_REG_B 0x01
#define HMC5883L_MAG_MODE 0x02
#define HMC5883L_MAG_DATAX0 0x03
#define HMC5883L_MAG_GAIN 0x20  //Default gain
#define HMC5883L_MAG_SCALE 0.92  // mG/LSb

bool check_magnetometer()
{
  //if (check_ID(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_ID) == 'H')
  // {
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_B,HMC5883L_MAG_GAIN);  //Sets the gain
  delay(5);
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_REG_A,0x18); //75Hz output
  delay(5);
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01); //Single-Measurement Mode
  delay(10);
  return true;
  //}
  //else
  //return false;
}

geometry_msgs::Vector3 measure_magnetometer()
{
  int reads = 0;
  byte buffer[6];
  geometry_msgs::Vector3 raw_magnetic_field;
  send_value(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_DATAX0);
  Wire.requestFrom(HMC5883L_MAG_ADDRESS,6);
  while(Wire.available())
  {
    buffer[reads] = Wire.read();
    reads++;
  }
  raw_magnetic_field.x =  (float)(MAG_X_INVERT * ((int16_t)(buffer[2*MAG_X_AXIS] << 8) | (buffer[2*MAG_X_AXIS+1]))) * HMC5883L_MAG_SCALE; 
  raw_magnetic_field.y =  (float)(MAG_X_INVERT * ((int16_t)(buffer[2*MAG_Y_AXIS] << 8) | (buffer[2*MAG_Y_AXIS+1]))) * HMC5883L_MAG_SCALE;
  raw_magnetic_field.z =  (float)(MAG_X_INVERT * ((int16_t)(buffer[2*MAG_Z_AXIS] << 8) | (buffer[2*MAG_Z_AXIS+1]))) * HMC5883L_MAG_SCALE;
  write_to_register(HMC5883L_MAG_ADDRESS,HMC5883L_MAG_MODE,0x01);
  return raw_magnetic_field;
}

#endif  // _MAGNETOMETER_HMC5883L_H_

