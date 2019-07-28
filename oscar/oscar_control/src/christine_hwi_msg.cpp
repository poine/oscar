#include "oscar_control/christine_hwi_msg.h"
#include <iostream>

//#define DEBUG__
#ifdef DEBUG__
#define DEBUG_(_x) _x
#else
#define DEBUG_(_x)
#endif

void parser_init(struct ChristineHWIParser* self) {
  self->err_cnt = 0;
  parser_reset(self);
}

void parser_reset(struct ChristineHWIParser* self) {
  self->status = STA_UNINIT;
  self->buf_idx = 0;
}

void parser_parse(struct ChristineHWIParser* self, uint8_t b) {
  switch (self->status) {
    case STA_UNINIT:
      if (b == CHRISTINE_HWI_MSG_STX) {
        self->status = STA_GOT_STX;
	DEBUG_(std::printf("  stx: %x\n", b);)
      }
      DEBUG_(else std::printf("  dropping: %x\n", b);)
      break;
    case STA_GOT_STX:
      DEBUG_(std::printf("  len: %x (%d)\n", b, b);)
      self->len = b;
      self->status = STA_GOT_LEN;
      break;
    case STA_GOT_LEN:
      DEBUG_(std::printf("  seq1: %x (%d)\n", b, b);)
      self->seq = b;
      self->status = STA_GOT_SEQ1;
      break;
    case STA_GOT_SEQ1:
      DEBUG_(std::printf("  seq2: %x (%d)\n", b, b);)
      self->seq += b>>8;
      self->status = STA_GOT_SEQ2;
      break;
    case STA_GOT_SEQ2:
      self->buf[self->buf_idx] = b; self->buf_idx += 1;
      if (self->buf_idx == self->len)
	self->status = STA_GOT_PAYLOAD;
      break;
    case STA_GOT_PAYLOAD:
      DEBUG_(std::printf("  ck1: %x\n", b);)
      self->buf[self->buf_idx] = b; self->buf_idx += 1;
      self->status = STA_GOT_CK1;
      break;
    case STA_GOT_CK1:
      DEBUG_(std::printf("  ck2: %x\n", b);)
      self->buf[self->buf_idx] = b; self->buf_idx += 1;
      uint8_t ck1, ck2;
      compute_checksum(self->buf, self->len, &ck1, &ck2);
      if (ck1 == self->buf[self->len] and ck2 == self->buf[self->len+1]) 
	self->msg_cbk(self->buf, self->buf_idx);
      else
	self->err_cnt += 1;
      parser_reset(self);
      break;
	    
  }
}


void compute_checksum(uint8_t* buf, uint8_t len, uint8_t* ck1, uint8_t* ck2) {
  // TODO
  *ck1 = 42;
  *ck2 = 43;
}






