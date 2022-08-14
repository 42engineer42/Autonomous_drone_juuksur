#pragma once

#include <stdint.h>

#define AUTOHOVER_KP 0.1f
#define AUTOHOVER_KI 0.05f
#define AUTOHOVER_KD 0.0f

typedef struct PidState {
    float kp, ki, kd;
    float prevErr;
    float integral;
} PidState;

typedef struct FailsafeState {
    //PidState pid;
    uint32_t lastUpdateTick;
    float holdHeightCM;
    float curThrottle;
    float lastHeight; // height on last iteration
} FailsafeState;

#ifdef __cplusplus
extern "C" {
#endif
void pid_init(PidState *ps, float kp, float ki, float kd);
float pid_update(PidState *ps, float measurement, float target, float dt);
void failsafe_pi_ctrl_lost(FailsafeState *fs, uint32_t curTick, uint16_t curHeight);
uint16_t failsafe_autohover_update2(FailsafeState *fs, uint32_t curTick, uint16_t curHeight, int16_t curAccZCMS);
#ifdef __cplusplus
}
#endif


