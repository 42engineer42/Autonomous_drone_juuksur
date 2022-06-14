#include "led_controller.h"

#include "main.h"

void led_controller_init(LEDController *lc) {
    lc->mask[0] = 0;
    lc->mask[1] = 0;
    lc->mask[2] = 0;
    lc->toggleInterval[0] = 0;
    lc->toggleInterval[1] = 0;
    lc->toggleInterval[2] = 0;
    lc->toggleInterval[3] = 0;
    lc->ports[0][0] = TestLED1_Red_GPIO_Port;
    lc->ports[0][1] = TestLED1_Green_GPIO_Port;
    lc->ports[0][2] = TestLED1_Blue_GPIO_Port;
    lc->ports[1][0] = TestLED2_Red_GPIO_Port;
    lc->ports[1][1] = TestLED2_Green_GPIO_Port;
    lc->ports[1][2] = TestLED2_Blue_GPIO_Port;
    lc->ports[2][0] = TestLED3_Red_GPIO_Port;
    lc->ports[2][1] = TestLED3_Green_GPIO_Port;
    lc->ports[2][2] = TestLED3_Blue_GPIO_Port;
    lc->ports[3][0] = TestLED4_Red_GPIO_Port;
    lc->ports[3][1] = TestLED4_Green_GPIO_Port;
    lc->ports[3][2] = TestLED4_Blue_GPIO_Port;
    lc->pins[0][0] = TestLED1_Red_Pin;
    lc->pins[0][1] = TestLED1_Green_Pin;
    lc->pins[0][2] = TestLED1_Blue_Pin;
    lc->pins[1][0] = TestLED2_Red_Pin;
    lc->pins[1][1] = TestLED2_Green_Pin;
    lc->pins[1][2] = TestLED2_Blue_Pin;
    lc->pins[2][0] = TestLED3_Red_Pin;
    lc->pins[2][1] = TestLED3_Green_Pin;
    lc->pins[2][2] = TestLED3_Blue_Pin;
    lc->pins[3][0] = TestLED4_Red_Pin;
    lc->pins[3][1] = TestLED4_Green_Pin;
    lc->pins[3][2] = TestLED4_Blue_Pin;
}

void led_controller_update(LEDController *lc, uint32_t curTick) {
    for(int i = 0; i < 4; i++) {
        uint16_t ival = lc->toggleInterval[i];
        uint8_t ivalEn = lc->toggleInterval[i] == 0 ? 1 : (curTick%(ival*2))>ival;
        for(int j = 0; j < 3; j++) {
            uint8_t en = (lc->mask[j]>>i)&1;
            HAL_GPIO_WritePin(lc->ports[i][j], lc->pins[i][j], (ivalEn&en) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
}
