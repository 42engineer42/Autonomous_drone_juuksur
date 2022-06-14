#include "height_controller.h"

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

void height_controller_init(HeightController *hc) {
    hc->setPoint = 0.0f;
    hc->vintegral = 0.0f;

    hc->throttleControl = 1000;
}

void height_controller_update(HeightController *hc, float hVel, float height, float dt) {
    float herror = hc->setPoint - height;

    float velSetPoint = HEIGHT_CONTROLLER_KPH * herror;
    float verror = velSetPoint - hVel;
    verror = CLAMP(verror, -50.0f, 50.0f);
    hc->vintegral += verror * dt;

    float output = HEIGHT_CONTROLLER_ORIGIN + HEIGHT_CONTROLLER_FEED_FORWARD + HEIGHT_CONTROLLER_KPV*verror + HEIGHT_CONTROLLER_KIV*hc->vintegral;
    output = CLAMP(output, 1000.0f, HEIGHT_CONTROLLER_MAX_THROTTLE);

    hc->throttleControl = (uint16_t)output;
}
