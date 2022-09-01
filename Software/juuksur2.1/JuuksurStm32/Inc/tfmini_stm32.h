#pragma once

#include "tfmini.h"
#include "stm32f3xx_hal.h"

typedef struct TfminiStm32State {
    TfminiState tfmini;
    volatile uint8_t rxReadPos;
    volatile uint8_t rxWritePos;
    uint8_t rxData;
    uint8_t ringBuf[64];
    UART_HandleTypeDef *huart;
} TfminiStm32State;

#ifdef __cplusplus
extern "C" {
#endif

void tfmini_stm32_init(TfminiStm32State *tfs, UART_HandleTypeDef *huart);
void tfmini_stm32_receive_complete(TfminiStm32State *tfs);
void tfmini_stm32_process_rx(TfminiStm32State *tfs);

#ifdef __cplusplus
}
#endif
