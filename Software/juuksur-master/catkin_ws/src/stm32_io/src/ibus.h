#pragma once

#define IBUS_CHANNEL_DEFAULT 1500

#define IBUS_MAGIC_1 0x20 // not really magic, its the packet size (32 in dec)
#define IBUS_MAGIC_2 0x40 // high nibble is command type

#define IBUS_CH_SIZE 0x20 // 32 in dec
#define IBUS_SENSOR_REQ_SIZE 0x4

#define IBUS_CMD_CH 0x40
#define IBUS_CMD_SENSOR_DISCOVER 0x80
#define IBUS_CMD_SENSOR_TYPE 0x90
#define IBUS_CMD_SENSOR_MEASUREMENT 0xA0


#include <stdint.h>

typedef struct IbusChannels {
    uint16_t channels[14];
} IbusChannels;

typedef struct IbusSensorDiscovery {
    uint8_t sensorId;
} IbusSensorDiscovery;

typedef struct IbusSensorType {
    uint8_t sensorId;
    uint8_t sensorType;
} IbusSensorType;

typedef struct IbusSensorMeasurement {
    uint8_t sensorId;
    uint16_t measurement;
} IbusSensorMeasurement;

typedef struct IbusEvent {
    union {
        IbusChannels ch;
        IbusSensorDiscovery sdisc;
        IbusSensorType stype;
        IbusSensorMeasurement smeas;
    } e;
    uint8_t type;
} IbusEvent;

typedef void (*IbusCb)(void *usrData, IbusEvent *e);
typedef void (*IbSensorDiscoveryCb)(IbusSensorDiscovery *sd);
typedef void (*IbSensorTypeCb)(IbusSensorType *st);
typedef void (*IbSensorMeasurementCb)(IbusSensorMeasurement *sm);

typedef struct IbusState {
    uint8_t state;
    uint8_t payload[28];
    //uint16_t channels[14];
    uint16_t checksum;
    uint16_t curSize;
    uint16_t curCmd;

    uint8_t curHeader[2];

    void *usrData;
    IbusCb onEvent;
} IbusState;


#ifdef __cplusplus
extern "C" {
#endif
void ibus_init(IbusState *ib);
void ibus_receive(IbusState *ib, uint8_t byte);
void ibus_set_event_callback(IbusState *ib, IbusCb cb, void *usrData);
int ibus_make_packet(IbusChannels *ib, uint8_t *buf, uint8_t bufLen);
int ibus_make_sensor_request_packet(uint8_t cmd, uint8_t sensorId, uint8_t *buf, uint8_t bufLen);
#ifdef __cplusplus
}
#endif
