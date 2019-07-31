#include "oscar_control/christine_remote_bbb.h"


#if 0
void msg_cbk(uint8_t* buf, uint8_t len) {
  std::printf("Got msg (%u)\n", len);
  struct ChristineHardwareOutput* hi = reinterpret_cast<struct ChristineHardwareOutput*>(buf);
  std::printf("  bat: %.2f\n", hi->bat_voltage);
  std::printf("  vel: %.2f\n", hi->mot_vel);
  //std::printf("  mot_enc: %f\n", hi->mot_enc);
}
#endif

void msg_cbk(void* data, uint8_t* buf, uint8_t len) { reinterpret_cast<BBBLink*>(data)->msg_callback(buf, len); }

BBBLink::BBBLink():
  serial_("/dev/ttyTHS1", 115200)
{
  parser_.msg_cbk = msg_cbk;
  parser_.msg_cbk_data = this;
  //parser_.msg_cbk = std::bind(&BBBLink::msg_callback, this, std::placeholders::_1, std::placeholders::_2)
  parser_reset(&parser_);
  out_seq_ = 0;
  mot_pos_ = 0.; 
  mot_vel_ = 0.;
  msg_callback_ = NULL;
  serial_.register_receive_callback(std::bind(&BBBLink::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void BBBLink::msg_callback(uint8_t* buf, uint8_t len) {
  struct ChristineHardwareOutput* hi = reinterpret_cast<struct ChristineHardwareOutput*>(buf);
  //std::printf("  bat: %.2f mot %.2f\n", hi->bat_voltage, hi->mot_vel);
  bat_ = hi->bat_voltage;
  mot_pos_ = 0.11f; 
  mot_vel_ =  hi->mot_vel; 
  if (msg_callback_ != NULL) msg_callback_(this, user_data_);
}


void BBBLink::get_motor(float* pos, float* vel) {
  *pos = mot_pos_;
  *vel = mot_vel_;
}

void BBBLink::get_bat(float* bat) {
  *bat = bat_;
}



void BBBLink::serial_callback(const uint8_t* buf, size_t len) {
#if 0
  fprintf(stderr, "read %lu\n", len);
  for (auto i=0; i<len; i++)
    std::printf("  %x", buf[i]);
  std::printf("\n");
#endif
  for (auto i=0; i<len; i++)
    parser_parse(&parser_, buf[i]);
}

bool BBBLink::init() {
  if (!serial_.init()) {
    std::printf("Failed to initialize serial port\n");
    return false;
  }
  std::printf("initialized serial port\n");
  return true;
}

void BBBLink::set_msg_callback(std::function<void(BBBLink* me, void* userdata)> fun, void* user_data) {
  msg_callback_ = fun;
  user_data_ = user_data;
}

bool BBBLink::send(struct ChristineHardwareInputMsg* him) {
  him->stx = CHRISTINE_HWI_MSG_STX;
  him->len = sizeof(struct ChristineHardwareInput);
  him->seq = out_seq_;
  // TODO: compute checksum
  uint8_t* buf = reinterpret_cast<uint8_t*>(him);
  compute_checksum(buf+4, sizeof(him->data), &him->ck1, &him->ck2);
  serial_.send_bytes(buf, sizeof(*him));
#if 0
  std::printf("  sending: %f %f\n", _steering, _lvel);
  std::printf("  sending: ");
  for (auto i=0; i<sizeof(*him); i++)
    std::printf("  %x", buf[i]);
  std::printf("\n");
#endif
  out_seq_ += 1;
  return true;
}

bool BBBLink::send2(float steering, float throttle) {
  struct ChristineHardwareInputMsg him;
  him.data.steering_srv = steering;
  him.data.throttle_servo = throttle;
  send(&him);
}
