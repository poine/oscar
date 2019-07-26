//
//
//
#ifndef CHRISTINE_HW_H
#define CHRISTINE_HW_H

struct ChristineHardwareInput {
  float steering_srv;
  float throttle_servo;
  
};

struct ChristineHardwareOutput {
  float bat_voltage;
  float mot_enc;
  float mot_vel;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float qw;
  float qx;
  float qy;
  float qz;
};

#endif // CHRISTINE_HW_H
