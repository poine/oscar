//
//
//
#ifndef CHRISTINE_HWI_MSG_H
#define CHRISTINE_HWI_MSG_H

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


struct ChristineHardwareInputMsg {
  uint8_t h1;
  uint8_t len;
  struct ChristineHardwareInput data;
  uint8_t ck1;
  uint8_t ck2;
};

struct ChristineHardwareOutputMsg {
  uint8_t h1;
  uint8_t len;
  struct ChristineHardwareOutput data;
  uint8_t ck1;
  uint8_t ck2;
};


#define CHRISTINE_HWI_MSG_STX 0x99

#endif // CHRISTINE_HWI_MSG_H
