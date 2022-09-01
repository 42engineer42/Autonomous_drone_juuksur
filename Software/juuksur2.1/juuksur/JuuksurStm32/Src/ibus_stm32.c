#include "ibus_stm32.h"

#include <stdio.h>
#include <assert.h>

void ibus_stm32_init(IbusStm32State *ibs, UART_HandleTypeDef *huart, uint8_t halfDuplex) {
    HAL_StatusTypeDef status;
    ibs->huart = huart;
    ibs->txBusy = 0;
    ibs->rxReadPos = 0;
    ibs->rxWritePos = 0;
    ibs->halfDuplex = halfDuplex;
    ibs->totalRx = 0;
    ibs->totalTx = 0;
    for(int i = 0; i < 14; i++) {
        ibs->txCh.channels[i] = 1500;
    }
    ibus_init(&ibs->ibRx);
    // start receiving
    //status = HAL_UART_Receive_IT(huart, ibs->itData, 32);
    status = HAL_UART_Receive_IT(huart, &ibs->rxData, 1);
    if(status != HAL_OK) {
        printf("Receive_IT failed in ibus_stm32_init() code %d\r\n", status);
    }
}

void ibus_stm32_receive_complete(IbusStm32State *ibs) {
    HAL_StatusTypeDef status;
    // NOTE: readPos can not be edited because this is called from interrupt and reading is done in main loop
    if(ibs->rxWritePos != ibs->rxReadPos + RBUF_SIZE) {
        ibs->ringBuf[ibs->rxWritePos & RBUF_MASK] = ibs->rxData;
        ibs->rxWritePos++;
    } else {
        printf("IBus Rx buffer overflow! %d %d\r\n", ibs->rxWritePos, ibs->rxReadPos);
        assert(0);
    }
    status = HAL_UART_Receive_IT(ibs->huart, &ibs->rxData, 1);
    if(status != HAL_OK) {
        printf("Receive_IT failed in ibus_stm32_receive complete() code %d\r\n", status);
    }
}

void ibus_stm32_process_rx(IbusStm32State *ibs) {
    uint8_t readPos;
    while(ibs->rxReadPos != ibs->rxWritePos) {
        readPos = ibs->rxReadPos & RBUF_MASK;
        ibs->totalRx++;
        ibus_receive(&ibs->ibRx, ibs->ringBuf[readPos]);
        //printf("%02x ", ibs->ringBuf[readPos]);
        ibs->rxReadPos++;
    }
}

void ibus_stm32_transmit_complete(IbusStm32State *ibs) {
    ibs->txBusy = 0;
    if(ibs->halfDuplex == 1) {
        HAL_HalfDuplex_EnableReceiver(ibs->huart);
    }
}

void ibus_stm32_try_transmit(IbusStm32State *ibs) {
    if(!ibs->txBusy) {
        ibs->totalTx += IBUS_PACKET_SIZE;
        ibus_make_packet(&ibs->txCh, ibs->txData, IBUS_PACKET_SIZE);
        HAL_StatusTypeDef status = HAL_UART_Transmit_IT(ibs->huart, ibs->txData, IBUS_PACKET_SIZE);
        if(status != HAL_OK) {
            printf("Transmit_IT failed in ibus_stm32_try_transmit! e: %d %p\r\n", status, ibs->huart->Instance);
        }
        ibs->txBusy = 1;
    }
}

// NOTE: this function obviously only works on half duplex uart
void send_telemetry_sensor_request(IbusStm32State *ibs, uint8_t sensorId) {
  int16_t size = ibus_make_sensor_request_packet(IBUS_CMD_SENSOR_MEASUREMENT, sensorId, ibs->ibusReqBuf, 32);
  assert(size > 0);
  HAL_HalfDuplex_EnableTransmitter(ibs->huart);
  HAL_UART_Transmit_DMA(ibs->huart, ibs->ibusReqBuf, size);
  ibs->totalTx += size;
}
