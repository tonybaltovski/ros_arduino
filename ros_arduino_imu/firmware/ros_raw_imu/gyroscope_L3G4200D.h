// FINISH THIS AND TEST
#ifndef _GYROSCOPE_L3G4200D_H
#define _GYROSCOPE_L3G4200D_H

#include <Arduino.h>

//L3G4200D REGISTERS
#define L3G4200D_GYRO_ADDRESS 0x69
#define L3G4200D_WHO_AM_I 0x0F
#define L3G4200D_CTRL_REG1 0x20
#define L3G4200D_CTRL_REG4 0x23
#define L3G4200D_CTRL_REG5 0x24

bool check_gyroscope()
{
  if (check_ID(L3G4200D_GYRO_ADDRESS,L3G4200D_WHO_AM_I) == L3G4200D_WHO_AM_I)
  {
    write_to_register(L3G4200D_GYRO_ADDRESS,L3G4200D_CTRL_REG1,0x9F); //xyz and no power down
    delay(10); 
    write_to_register(L3G4200D_GYRO_ADDRESS,L3G4200D_CTRL_REG4,0xB0); //full scale at 2000 dps
    delay(10);
    write_to_register(L3G4200D_GYRO_ADDRESS,GYRO_CTRL_REG5,0x02); // hpf
    delay(20);
    return true;
  }
  else
    return false;
}

geometry_msgs::Vector3 measure_gyroscope()
{
  geometry_msgs::Vector3 raw_rotation;
  send_value(L3G4200D_GYRO_ADDRESS,0x80 | 0x28);
  Wire.requestFrom(L3G4200D_GYRO_ADDRESS,6);
  raw_rotation.x = ( Wire.read() | (Wire.read() << 8)) / 939.275077264; //rad/s
  raw_rotation.y = ( Wire.read() | (Wire.read() << 8)) / 939.275077264;
  raw_rotation.z = ( Wire.read() | (Wire.read() << 8)) / 939.275077264;
  return raw_rotation;
}

#endif
