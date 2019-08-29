//
//
//
#ifndef CHRISTINE_REMOTE_BBB_PROTOCOL_H
#define CHRISTINE_REMOTE_BBB_PROTOCOL_H
#include <inttypes.h>

struct ChristineHardwareInput {
  float steering_srv;
  float throttle_servo; // FIXME this is lin vel setpoint
};

struct ChristineHardwareOutput {
  float bat_voltage;
  int32_t mot_pos;
  float mot_vel;
  float dsm_steering;
  float dsm_throttle;
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
  uint8_t stx;
  uint8_t len;
  uint16_t seq;
  struct ChristineHardwareInput data;
  uint8_t ck1;
  uint8_t ck2;
};

struct ChristineHardwareOutputMsg {
  uint8_t stx;
  uint8_t len;
  uint16_t seq;
  struct ChristineHardwareOutput data;
  uint8_t ck1;
  uint8_t ck2;
};

#define CHRISTINE_HWI_MSG_BUF_LEN 255
#define CHRISTINE_HWI_MSG_STX 0x99


void compute_checksum(uint8_t* buf, uint8_t len, uint8_t* ck1, uint8_t* ck2);

#define STA_UNINIT      0
#define STA_GOT_STX     1
#define STA_GOT_LEN     2
#define STA_GOT_SEQ1    3
#define STA_GOT_SEQ2    4
#define STA_GOT_PAYLOAD 5
#define STA_GOT_CK1     6

struct ChristineHWIParser {
  uint8_t status;
  uint8_t buf[CHRISTINE_HWI_MSG_BUF_LEN];
  uint8_t buf_idx;
  uint8_t len;
  uint16_t seq;
  uint16_t err_cnt;
  //uint64_t last_rx_msg_time;
  void (*msg_cbk)(void* data, uint8_t* buf, uint8_t len);
  void* msg_cbk_data;
};


void parser_init(struct ChristineHWIParser* self);
void parser_reset(struct ChristineHWIParser* self);
void parser_parse(struct ChristineHWIParser* self, uint8_t b);


#endif // CHRISTINE_REMOTE_BBB_PROTOCOL_H
