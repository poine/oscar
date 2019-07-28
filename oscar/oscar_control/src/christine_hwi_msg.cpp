#include "oscar_control/christine_hwi_msg.h"
#include <iostream>

void parser_reset(struct ChristineHWIParser* self) {
  self->status = STA_UNINIT;
  self->buf_idx = 0;
}

void parser_parse(struct ChristineHWIParser* self, uint8_t b) {
  switch (self->status) {
    case STA_UNINIT:
      if (b == CHRISTINE_HWI_MSG_STX) {
        self->status = STA_GOT_STX;
	std::printf("  stx: %x\n", b);
      }
      else
	std::printf("  dropping: %x\n", b);
      break;
    case STA_GOT_STX:
      std::printf("  len: %x (%d)\n", b, b);
      self->len = b;
      self->status = STA_GOT_LEN;
      break;
  case STA_GOT_LEN:
    //std::printf("  len: %x (%d)\n", b, b);
      self->seq = b;
      self->status = STA_GOT_SEQ1;
      break;
  case STA_GOT_SEQ1:
    //std::printf("  len: %x (%d)\n", b, b);
      self->seq += b>>8;
      self->status = STA_GOT_SEQ2;
      break;
    case STA_GOT_SEQ2:
      self->buf[self->buf_idx] = b;
      self->buf_idx += 1;
      if (self->buf_idx == self->len)
	self->status = STA_GOT_PAYLOAD;
      break;
    case STA_GOT_PAYLOAD:
      std::printf("  ck1: %x\n", b);
      self->status = STA_GOT_CK1;
      break;
    case STA_GOT_CK1:
      std::printf("  ck2: %x\n", b);
      self->msg_cbk(self->buf, self->buf_idx);
      parser_reset(self);
      break;
	    
  }
}


void compute_checksum(uint8_t* buf, uint8_t len, uint8_t* ck1, uint8_t* ck2) {
  
  *ck1 = 0;
  *ck2 = 0;
}






