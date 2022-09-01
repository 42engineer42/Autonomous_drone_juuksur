#include "ibus.h"

#include <stdio.h>
#include <assert.h>


static int ibus_test_cs(IbusState *ib);

void ibus_empty_cb(void *usrData, void *ptr) {
}

void ibus_init(IbusState *ib) {
    ib->state = 0;
    ib->onEvent = (IbusCb) ibus_empty_cb;
}

void ibus_set_event_callback(IbusState *ib, IbusCb cb, void *usrData) {
    ib->onEvent = cb;
    ib->usrData = usrData;
}

// TODO: clean these 4 functions up

static void ibus_start_ch_cmd(IbusState *ib, uint8_t byte) {
    // valid channel data cmd header
    if(byte == 0x40 && ib->curSize == 0x20) {
        ib->curCmd = IBUS_CMD_CH;
        ib->curHeader[1] = byte;
        ib->state++; 
    } else { // invalid (size didn't match or command didn't make sense)
        ib->curSize = byte;
        ib->curHeader[0] = byte;
    }
}

static void ibus_start_sensor_discovery_cmd(IbusState *ib, uint8_t byte) {
    // valid sensor discovery header
    if((byte&0xF0) == IBUS_CMD_SENSOR_DISCOVER && ib->curSize == 4) {
        ib->curCmd = IBUS_CMD_SENSOR_DISCOVER;
        ib->curHeader[1] = byte;
        ib->state++;
    } else {
        ib->curSize = byte;
        ib->curHeader[0] = byte;
    }
}

static void ibus_start_sensor_type_cmd(IbusState *ib, uint8_t byte) {
    if((byte&0xF0) == IBUS_CMD_SENSOR_TYPE && ib->curSize == 6) {
        ib->curCmd = IBUS_CMD_SENSOR_TYPE;
        ib->curHeader[1] = byte;
        ib->state++;
    } else {
        ib->curSize = byte;
        ib->curHeader[0] = byte;
    }
}

static void ibus_start_sensor_measurement_cmd(IbusState *ib, uint8_t byte) {
    // TODO: sensor data might be larger than 2, depends on sensor type
    if((byte&0xF0) == IBUS_CMD_SENSOR_MEASUREMENT && ib->curSize == 6) {
        ib->curCmd = IBUS_CMD_SENSOR_MEASUREMENT;
        ib->curHeader[1] = byte;
        ib->state++;
    } else {
        ib->curSize = byte;
        ib->curHeader[0] = byte;
    }
}

void ibus_receive(IbusState *ib, uint8_t byte) {
    uint8_t csValid = 0;
    int i, j;
    if(ib->state == 0) {
        ib->curSize = byte;
        ib->curHeader[0] = byte;
        ib->state++;
    } else if(ib->state == 1) {
        switch(byte & 0xF0) {
            case 0x40:
                ibus_start_ch_cmd(ib, byte);
                break;
            case 0x80:
                ibus_start_sensor_discovery_cmd(ib, byte);
                break;
            case 0x90:
                ibus_start_sensor_type_cmd(ib, byte);
                break;
            case 0xA0:
                ibus_start_sensor_measurement_cmd(ib, byte);
                break;
            default:
                ib->curHeader[0] = byte;
                ib->curSize = byte;
                break;
        }
    } else if(ib->state >= 2 && ib->state < ib->curSize - 2) {
        ib->payload[ib->state++ - 2] = byte;
    } else if(ib->state == ib->curSize - 2) {
        ib->checksum = (uint16_t)byte;
        ib->state++;
    } else if(ib->state == ib->curSize - 1) {
        ib->checksum |= ((uint16_t)byte)<<8;
        csValid = ibus_test_cs(ib);
        IbusEvent e;
        if(csValid) {
            e.type = ib->curCmd;
            switch(ib->curCmd) {
                case IBUS_CMD_CH:
                    for(i = 0, j = 0; i < 14; i++, j+=2) {
                        e.e.ch.channels[i] = (uint16_t)ib->payload[j];
                        e.e.ch.channels[i] |= ((uint16_t)ib->payload[j+1])<<8;
                    }
                    ib->onEvent(ib->usrData, &e);
                    break;
                case IBUS_CMD_SENSOR_DISCOVER:
                    e.e.sdisc.sensorId = ib->curHeader[1] & 0x0F;
                    ib->onEvent(ib->usrData, &e);
                    break;
                case IBUS_CMD_SENSOR_TYPE:
                    e.e.stype.sensorId = ib->curHeader[1] & 0x0F;
                    e.e.stype.sensorType = ib->payload[0];
                    ib->onEvent(ib->usrData, &e);
                    break;
                case IBUS_CMD_SENSOR_MEASUREMENT:
                    e.e.smeas.sensorId = ib->curHeader[1] & 0x0F;
                    e.e.smeas.umeas = (uint16_t)ib->payload[0] | ((uint16_t)ib->payload[1]<<8);
                    ib->onEvent(ib->usrData, &e);
                    break;
            }
            ib->state = 0;
        } else {
            printf("ibus receive checksum mismatch\r\n");
            ib->state = 0;
        }
    }
}

static int ibus_test_cs(IbusState *ib) {
    uint16_t gchecksum = 0xFFFF - ib->curHeader[0] - ib->curHeader[1];
    int i;
    for(i = 0; i < ib->curSize - 4; i++) {
        gchecksum -= (uint16_t)ib->payload[i];
    }
    return gchecksum == ib->checksum;
}

static uint16_t ibus_calc_cs(uint8_t *buf, uint8_t count) {
    uint16_t ret = 0xFFFF;
    int i;
    for(i = 0; i < count; i++) {
        ret -= buf[i];
    }
    return ret;
}

int ibus_make_packet(IbusChannels *ib, uint8_t *buf, uint8_t bufLen) {
  int i;
  uint16_t ch;
  uint8_t b;
  if(bufLen < 32) {
      return -1;
  } else {
    uint16_t checksum = 0xFFFF - IBUS_MAGIC_1 - IBUS_MAGIC_2;
    buf[0] = IBUS_CH_SIZE;
    buf[1] = IBUS_MAGIC_2;
    // TODO: just use ibus_calc_cs for checksum?
    for(i = 0; i < 14; i++) {
      ch = ib->channels[i];
      b = ch&0xFF;
      checksum -= b;
      buf[i*2 + 2] = b; 
      b = (ch>>8)&0xFF;
      checksum -= b;
      buf[i*2 + 3] = b;
    }
    buf[30] = checksum&0xFF;
    buf[31] = (checksum>>8)&0xFF;
    return 0;
  } 
}

int ibus_make_sensor_request_packet(uint8_t cmd, uint8_t sensorId, uint8_t *buf, uint8_t bufLen) {
    uint16_t cs;
    if(bufLen < 4) {
        return -1;
    } else {
        assert(cmd == IBUS_CMD_SENSOR_DISCOVER || cmd == IBUS_CMD_SENSOR_TYPE || cmd == IBUS_CMD_SENSOR_MEASUREMENT);
        buf[0] = IBUS_SENSOR_REQ_SIZE; // packet size
        buf[1] = cmd | (sensorId & 0x0F);
        cs = ibus_calc_cs(buf, 2);
        buf[2] = cs & 0xFF;
        buf[3] = (cs>>8)&0xFF;
        return IBUS_SENSOR_REQ_SIZE;
    }
}
