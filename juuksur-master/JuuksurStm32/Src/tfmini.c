#include "tfmini.h"

void tfmini_init(TfminiState *tm) {
  tm->state = 0;
  tm->dist = 30;
  tm->strength = 65535;
}

void tfmini_receive(TfminiState *tm, uint8_t data) {
  int i;
  uint8_t checksum;
  switch(tm->state) {
    case 0:
    case 1:
      if(data == 0x59) {
        tm->state++;
      }
      break;
    case 2 ... 7:
      tm->payload[tm->state++ - 2] = data;
      break;
    case 8:
      tm->state = 0;
      checksum = 0x59+0x59;
      for(i = 0; i < 6; i++) {
        checksum += tm->payload[i];
      }
      if(checksum == data) {
        tm->dist = (uint16_t)tm->payload[0] | ((uint16_t)tm->payload[1] << 8);
        tm->strength = (uint16_t)tm->payload[2] | ((uint16_t)tm->payload[3] << 8);
        tm->reserved = tm->payload[4];
        tm->sigQ = tm->payload[5];
      }
      break;
  }
}
