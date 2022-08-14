#include "tfmini_stm32.h"

#include <assert.h>
#include <main.h>

void tfmini_stm32_init(TfminiStm32State *tfs, UART_HandleTypeDef *huart) {
    tfs->huart = huart;
    tfs->rxReadPos = 0;
    tfs->rxWritePos = 0;
    tfmini_init(&tfs->tfmini);
}

void tfmini_stm32_start_receive(TfminiStm32State *tfs) {
    HAL_StatusTypeDef status;
    status = HAL_UART_Receive_IT(tfs->huart, &tfs->rxData, 1);
    if(status != HAL_OK) {
        printf("Receive_IT failed in tfmini_stm32_init() code %d\r\n", status);
        assert(0);
    }
}

void tfmini_stm32_receive_complete(TfminiStm32State *tfs) {
    HAL_StatusTypeDef status;
    if(tfs->rxWritePos != tfs->rxReadPos + TFMINI_RBUF_SIZE) {
        tfs->ringBuf[tfs->rxWritePos & TFMINI_RBUF_MASK] = tfs->rxData;
        tfs->rxWritePos++;
    } else {
        HAL_GPIO_WritePin(TestLED2_Blue_GPIO_Port, TestLED2_Blue_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(TestLED4_Blue_GPIO_Port, TestLED4_Blue_Pin, GPIO_PIN_SET);
        printf("Tfmini Rx buffer overflow! %d %d\r\n", tfs->rxWritePos, tfs->rxReadPos);
        assert(0);
    }
    status = HAL_UART_Receive_IT(tfs->huart, &tfs->rxData, 1);
    if(status != HAL_OK) {
        //HAL_GPIO_WritePin(TestLED3_Blue_GPIO_Port, TestLED3_Blue_Pin, GPIO_PIN_SET);
        printf("Receive_IT() failed in tfmini_stm32_receive_complete() code %d\r\n", status);
    }
}

void tfmini_stm32_process_rx(TfminiStm32State *tfs) {
    uint16_t readPos;
    while(tfs->rxReadPos != tfs->rxWritePos) {
        readPos = tfs->rxReadPos&TFMINI_RBUF_MASK;
        tfmini_receive(&tfs->tfmini, tfs->ringBuf[readPos]);
        tfs->rxReadPos++;
    }
}
