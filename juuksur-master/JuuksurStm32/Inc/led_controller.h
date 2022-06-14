#pragma once

#include <stdint.h>
#include "stm32f3xx_hal.h"

typedef struct LEDController {
    uint8_t mask[3]; // bit0 = led0... RGB
    uint16_t toggleInterval[4]; // in milliseconds, half cycle, 0 - always on
    GPIO_TypeDef *ports[4][3];
    uint16_t pins[4][3];
} LEDController;

void led_controller_init(LEDController *lc);
void led_controller_update(LEDController *lc, uint32_t curTick);
