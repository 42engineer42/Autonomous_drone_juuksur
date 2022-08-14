#pragma once

#include <stdint.h>

typedef struct AngleController {
    float intRoll;
    float intPitch;
    float setpointRoll;
    float setpointPitch;

    uint16_t rollControl; // 1000 - 2000, 1500 is neutral, around accelerometer x axis
    uint16_t pitchControl;
} AngleController;

#define ANGLE_CONTROLLER_P 9.0f
#define ANGLE_CONTROLLER_I 1.8f
#define ANGLE_CONTROLLER_D 0.0f
#define ANGLE_CONTROLLER_BIAS 1500.0f // neutral state
#define ANGLE_CONTROLLER_MAX 100.0f // max offset from neutral state (100 = 1400-1600)


void angle_controller_init(AngleController *ac);
void angle_controller_update(AngleController *ac, float roll, float pitch, float dt);
