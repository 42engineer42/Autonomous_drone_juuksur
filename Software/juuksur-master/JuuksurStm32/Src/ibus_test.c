#if 0
cd "$(dirname "$0")"
gcc -ggdb3 ./ibus_test.c ./ibus.c -I../Inc -o ibus_test.out
gdb ./ibus_test.out
rm ibus_test.out
exit
#endif
#include <stdio.h>

#include "ibus.h"

uint8_t data1[] = {0x04, 0x81, 0x7A, 0xFF}; // sensor discover 1
uint8_t data2[] = {0x06, 0x91, 0x00, 0x02, 0x66, 0xFF}; // sensor type 1 0
uint8_t data3[] = {0x06, 0xA1, 0x00, 0x00, 0x58, 0xFF}; // measurement 1 0

void receive_all(IbusState *ib, uint8_t *buf, uint8_t len) {
    int i;
    for(i = 0; i < len; i++) {
        ibus_receive(ib, buf[i]);
    }
}

void ib_ch_callback(IbusChannels *ch) {
    printf("channels!\r\n");
}

void ib_sd_callback(IbusSensorDiscovery *sd) {
    printf("sensor discovery! sensorId: %d\r\n", sd->sensorId);
}

void ib_st_callback(IbusSensorType *st) {
    printf("sensor type! sensorId: %d sensorType: %d\r\n", st->sensorId, st->sensorType);
}

void ib_sm_callback(IbusSensorMeasurement *sm) {
    printf("sensor measurement! sensorId: %d measurement: %d\r\n", sm->sensorId, sm->measurement);
}

int main(void) {
    IbusState ib;
    ibus_init(&ib);
    ib.chCb = ib_ch_callback;
    ib.sdiscCb = ib_sd_callback;
    ib.stypeCb = ib_st_callback;
    ib.smeasCb = ib_sm_callback;
    receive_all(&ib, data1, sizeof(data1)); 
    receive_all(&ib, data2, sizeof(data2));
    receive_all(&ib, data3, sizeof(data3));
    printf("Hello, world!\r\n");
    return 0;
}
