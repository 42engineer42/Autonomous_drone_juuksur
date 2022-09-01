#pragma once

#include "ibus.h"
#include "stm32f3xx_hal.h"

#define IBUS_PACKET_SIZE 32

#define BUF_FLAG_DATA_READY 0x2
#define RBUF_MASK 0x3F // (1<<5)-1
#define RBUF_SIZE (1<<6) // 64

typedef struct IbusStm32State {
    uint8_t halfDuplex;
    IbusState ibRx;
    IbusChannels txCh;
    uint8_t txBusy;
    uint8_t rxData;
    volatile uint8_t rxReadPos;
    volatile uint8_t rxWritePos;
    uint8_t ringBuf[RBUF_SIZE];
    //uint8_t itData[IBUS_PACKET_SIZE]; // data buffer for receiving
    uint8_t txData[IBUS_PACKET_SIZE]; // data buffer for transmitting
    UART_HandleTypeDef *huart; // handle to HAL UART device
    uint8_t ibusReqBuf[32];
    uint32_t totalRx;
    uint32_t totalTx;
} IbusStm32State;

#ifdef __cplusplus
extern "C" {
#endif

void ibus_stm32_init(IbusStm32State *ibs, UART_HandleTypeDef *huart, uint8_t halfDuplex);
void ibus_stm32_receive_complete(IbusStm32State *ibs);
void ibus_stm32_process_rx(IbusStm32State *ibs);
void ibus_stm32_transmit_complete(IbusStm32State *ibs);
void ibus_stm32_try_transmit(IbusStm32State *ibs);
void send_telemetry_sensor_request(IbusStm32State *ibs, uint8_t sensorId);
        
#ifdef __cplusplus
}
#endif
