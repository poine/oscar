#ifndef OSCAR_CONTROL__CHRISTINE_REMOTE_BBB_H
#define OSCAR_CONTROL__CHRISTINE_REMOTE_BBB_H

#include <async_comm/serial.h>

#include "oscar_control/christine_remote_bbb_protocol.h"

class BBBLink {
 public:
  BBBLink();
  bool init();
  void set_msg_callback(std::function<void(BBBLink* me, void* userdata)> fun, void* user_data);
  bool send(struct ChristineHardwareInputMsg* him);
  bool send2(float steering, float throttle);
  void msg_callback(uint8_t* buf, uint8_t len);

  void get_bat(float* bat);
  void get_motor(float* pos, float* vel);
  
 private:
  async_comm::Serial serial_;
  void serial_callback(const uint8_t* buf, size_t len);
  struct ChristineHWIParser parser_;
  uint16_t out_seq_;
  //void msg_cbk_*(BBBLink* me, void* userdata);
  std::function<void(BBBLink* me, void* userdata)> msg_callback_;
  void* user_data_;
  
  float bat_;
  float mot_pos_;
  float mot_vel_;
  
};




#endif // OSCAR_CONTROL__CHRISTINE_REMOTE_BBB_H
