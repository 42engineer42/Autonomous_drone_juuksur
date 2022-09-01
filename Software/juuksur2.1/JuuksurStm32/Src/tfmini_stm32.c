#include "tfmini_stm32.h"

#include <assert.h>

void tfmini_stm32_init(TfminiStm32State *tfs, UART_HandleTypeDef *huart) {
    HAL_StatusTypeDef status;
    tfs->huart = huart;
    tfs->rxReadPos = 0;
    tfs->rxWritePos = 0;
    tfmini_init(&tfs->tfmini);
    status = HAL_UART_Receive_IT(huart, &tfs->rxData, 1);
    if(status != HAL_OK) {
        printf("Receive_IT failed in tfmini_stm32_init() code %d\r\n", status);
    }
}

void tfmini_stm32_receive_complete(TfminiStm32State *tfs) {
    HAL_StatusTypeDef status;
    if(tfs->rxWritePos != tfs->rxReadPos + 64) {
        tfs->ringBuf[tfs->rxWritePos & 0x3F] = tfs->rxData;
        tfs->rxWritePos++;
    } else {
        printf("Tfmini Rx buffer overflow! %d %d\r\n", tfs->rxWritePos, tfs->rxReadPos);
        assert(0);
    }
    status = HAL_UART_Receive_IT(tfs->huart, &tfs->rxData, 1);
    if(status != HAL_OK) {
        printf("Receive_IT() failed in tfmini_stm32_receive_complete() code %d\r\n", status);
    }
}

void tfmini_stm32_process_rx(TfminiStm32State *tfs) {
    uint8_t readPos;
    while(tfs->rxReadPos != tfs->rxWritePos) {
        readPos = tfs->rxReadPos & 0x3F;
        tfmini_receive(&tfs->tfmini, tfs->ringBuf[readPos]);
        tfs->rxReadPos++;
    }
}
