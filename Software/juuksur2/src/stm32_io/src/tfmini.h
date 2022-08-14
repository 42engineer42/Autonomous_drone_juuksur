#pragma once

#include <stdint.h>

typedef struct TfminiState {
  uint8_t state;
  uint16_t dist; // in cm 30-1200 (65536 if over limit, 30 below)
  uint16_t strength;
  uint8_t reserved;
  uint8_t sigQ;
  uint8_t payload[6];
} TfminiState;

#ifdef __cplusplus
extern "C" {
#endif
void tfmini_init(TfminiState *tm);
int tfmini_receive(TfminiState *tm, uint8_t data);
#ifdef __cplusplus
}
#endif
