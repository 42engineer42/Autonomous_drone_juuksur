#pragma once

#include <stdint.h>

typedef struct VelocityController {
    int mode;

    float intX;
    float intY;
    float intEX;
    float intEY;
    float setpointX;
    float setpointY;

    uint16_t rollControl; // 1000 - 2000, 1500 is neutral, around accelerometer x axis
    uint16_t pitchControl;
} VelocityController;

// good values before 13.03.2021 200
// 0.45 0.1 0.0

#define VELOCITY_CONTROLLER_P 0.50f
#define VELOCITY_CONTROLLER_I 0.12f
#define VELOCITY_CONTROLELR_D 0.0f
#define VELOCITY_CONTROLLER_BIAS 1500.0f // neutral state
#define VELOCITY_CONTROLLER_MAX 300.0f // max offset from neutral state (100 = 1400-1600)

#define VELOCITY_CONTROLLER_M1_EP 1.5f
#define VELOCITY_CONTROLLER_M1_AP 0.3f
#define VELOCITY_CONTROLLER_M1_AI 0.008f

#ifdef __cplusplus
extern "C" {
#endif

void velocity_controller_init(VelocityController *vc);
void velocity_controller_update(VelocityController *vc, float velX, float velY, float accX, float accY, float dt);

#ifdef __cplusplus
}
#endif
