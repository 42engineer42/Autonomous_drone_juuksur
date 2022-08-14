#include "angle_mode.h"

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SIGN(x) ((x) >= 0.0f ? 1.0f : -1.0f)
#define ABS(x) ((x) < 0.0f ? -(x) : (x))

void angle_controller_init(AngleController *ac) {
    ac->intRoll = 0.0f;
    ac->intPitch = 0.0f;
    ac->setpointRoll = 0.0f;
    ac->setpointPitch = 0.0f;
    ac->rollControl = 1500;
    ac->pitchControl = 1500;
}

void angle_controller_update(AngleController *ac, float roll, float pitch, float dt) {
    float errorX = ac->setpointPitch - pitch;
    float px = errorX * ANGLE_CONTROLLER_P;
    ac->intPitch += errorX * ANGLE_CONTROLLER_I * dt;
    float dx = 0.0f * ANGLE_CONTROLLER_D;
    float controlX = px + ac->intPitch + dx + ANGLE_CONTROLLER_BIAS;
    controlX = CLAMP(controlX, ANGLE_CONTROLLER_BIAS-ANGLE_CONTROLLER_MAX, ANGLE_CONTROLLER_BIAS+ANGLE_CONTROLLER_MAX);
    ac->pitchControl = (uint16_t)controlX;

    float errorY = ac->setpointRoll - roll;
    float py = errorY * ANGLE_CONTROLLER_P;
    ac->intRoll += errorY * ANGLE_CONTROLLER_I * dt;
    float dy = 0.0f * ANGLE_CONTROLLER_D;
    float controlY = py + ac->intRoll + dy + ANGLE_CONTROLLER_BIAS;
    controlY = CLAMP(controlY, ANGLE_CONTROLLER_BIAS-ANGLE_CONTROLLER_MAX, ANGLE_CONTROLLER_BIAS+ANGLE_CONTROLLER_MAX);
    ac->rollControl = (uint16_t)controlY;
}


