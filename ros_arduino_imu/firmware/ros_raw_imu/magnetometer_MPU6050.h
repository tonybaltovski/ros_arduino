#ifndef _MAGNETOMETER_MPU6050_H_
#define _MAGNETOMETER_MPU6050_H_

//MPU6050 REGISTERS
#define MPU6050_MAG_ADDRESS 0x0C
#define MPU6050_MAG_ID 0x34
#define MPU9150_RA_MAG_XOUT_L 0x03
#define MPU6050_MAG_GAIN 0x20  //Default gain
#define MPU6050_MAG_SCALE 0.92  // mG/LSb

bool check_magnetometer()
{
  send_value(MPU6050_MAG_ADDRESS,0x02);
  Wire.requestFrom(MPU6050_MAG_ADDRESS,1);
  while(Wire.available())
  {
    if(Wire.read())
    {
	return true;
    }
    else{
        return false;
    }
  }
}

void measure_magnetometer()
{
  mag_reads = 0;
  Wire.beginTransmission(MPU6050_MAG_ID);
  Wire.write(MPU6050_RA_INT_PIN_CFG);
  Wire.write(0x02); //set i2c bypass enable
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MPU6050_MAG_ADDRESS);
  Wire.write(0x0A);
  Wire.write(0x01);//enable the magnetometer
  delay(10);
  Wire.write(MPU9150_RA_MAG_XOUT_L)
  Wire.requestFrom(MPU6050_MAG_ADDRESS,6);
  while(Wire.available())
  {
    mag_buffer[mag_reads] = Wire.read();
    mag_reads++;
  }
  raw_magnetic_field.x =  (float)(MAG_X_INVERT * ((int16_t)((int)mag_buffer[2*MAG_X_AXIS+1] << 8) | (mag_buffer[2*MAG_X_AXIS]))) * MPU6050_MAG_SCALE; 
  raw_magnetic_field.y =  (float)(MAG_Y_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Y_AXIS+1] << 8) | (mag_buffer[2*MAG_Y_AXIS]))) * MPU6050_MAG_SCALE;
  raw_magnetic_field.z =  (float)(MAG_Z_INVERT * ((int16_t)((int)mag_buffer[2*MAG_Z_AXIS+1] << 8) | (mag_buffer[2*MAG_Z_AXIS]))) * MPU6050_MAG_SCALE;

  Wire.endTransmission();
  
}

#endif  // _MAGNETOMETER_MPU6050_H_

