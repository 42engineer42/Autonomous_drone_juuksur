#pragma once

#include <stdint.h>

typedef struct HeightController {
    float setPoint;

    float vintegral;
    
    uint16_t throttleControl;
} HeightController ;

#define HEIGHT_CONTROLLER_KPH 1.5f
#define HEIGHT_CONTROLLER_KPV 2.0f
#define HEIGHT_CONTROLLER_KIV 0.2f

#define HEIGHT_CONTROLLER_ORIGIN 1000.0f
#define HEIGHT_CONTROLLER_FEED_FORWARD 270.0f
#define HEIGHT_CONTROLLER_MAX_THROTTLE 1800.0f

#ifdef __cplusplus
extern "C" {
#endif

void height_controller_init(HeightController *hc);
void height_controller_update(HeightController *hc, float hVel, float height, float dt);

#ifdef __cplusplus
}
#endif
