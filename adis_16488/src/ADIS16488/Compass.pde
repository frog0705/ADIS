/* This file is part of the ADIS16488 code */

void Compass_Heading()
{
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
  cos_roll = cos(roll);
  sin_roll = sin(roll);
  cos_pitch = cos(pitch);
  sin_pitch = sin(pitch);
  
  mag_x = Magn_Vector[0]*cos_pitch + Magn_Vector[1]*sin_roll*sin_pitch + Magn_Vector[2]*cos_roll*sin_pitch;
  mag_y = Magn_Vector[1]*cos_roll - Magn_Vector[2]*sin_roll;
  MAG_Heading = atan2(-mag_y, mag_x);
}
