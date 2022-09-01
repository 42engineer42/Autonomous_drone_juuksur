#include "failsafe.h"

#define MIN(a,b) ((a)<(b)?(a):(b)) 
#define MAX(a,b) ((a)>(b)?(a):(b)) 
#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

#define USE_VELOCITY 1

#ifdef USE_VELOCITY
#define MAX_SPEED 5.0f // cm/s
#define MAX_THROTTLE 1550.0f
#define THROTTLE_GAIN 80.0f
#define THROTTLE_INITIAL 1250 // estimated hovering value
#define THROTTLE_REDUCE_FACTOR 0.3f
#else
#define MAX_ACC 2.0f
#define MAX_THROTTLE 1550
#define THROTTLE_GAIN 550.0f
#define THROTTLE_INITIAL 1250 // estimated hovering value
#define 
#endif

void pid_init(PidState *ps, float kp, float ki, float kd) {
    ps->prevErr = 0.0f;
    ps->integral = 0.0f;
    ps->kp = kp;
    ps->ki = ki;
    ps->kd = kd;
}

float pid_update(PidState *ps, float measurement, float target, float dt) {
    float error = target - measurement;
    ps->integral = CLAMP(ps->integral + error * dt, -100.0f, 100.0f);
    float derivative = (error - ps->prevErr) / dt;
    float output = ps->kp*error + ps->ki*ps->integral + ps->kd*derivative;
    ps->prevErr = error;
    return output;
}

void failsafe_pi_ctrl_lost(FailsafeState *fs, uint32_t curTick, uint16_t curHeight) {
    //pid_init(&fs->pid, AUTOHOVER_KP, AUTOHOVER_KI, AUTOHOVER_KD);
    fs->lastUpdateTick = curTick;
    //fs->holdHeightCM = (float)curHeight/* / 100.0f*/;
    fs->holdHeightCM = 100.0f;
    fs->curThrottle = THROTTLE_INITIAL;
    fs->lastHeight = (float)curHeight;
}

// with special sauce !
uint16_t failsafe_autohover_update2(FailsafeState *fs, uint32_t curTick, uint16_t curHeightCM, int16_t curAccZCMS) {
    uint32_t tickDif = curTick - fs->lastUpdateTick;
    if(tickDif >= 10) {
        // time delta
        float dt = (float)tickDif / 1000.0f; // time delta in sec
        float verticalSpeed = ((float)curHeightCM - (float)fs->lastHeight)/dt;
        // lower than set point -> positive
        // higher than set point -> negative
        float heightError = fs->holdHeightCM - curHeightCM;
        float heightErrorScalar = CLAMP(heightError / curHeightCM, -1.0f, 1.0f);
        // higher than 9.8 acceleration is up
        // lower than 9.8 acceleration is down
#ifndef USE_VELOCITY
        float accTarget = (MAX_ACC * heightErrorScalar) + 9.85; 
        float accError = accTarget - (curAccZCMS/100.0f);
        float finalScalar = accError / MAX_ACC;;
#else 
        float speedTarget = (MAX_SPEED * heightErrorScalar);
        float speedError = speedTarget - verticalSpeed;
        float finalScalar = speedError / MAX_SPEED;
#endif
        // accError positive -> move up (increase throttle)
        /*if(accErrorScalar > 0) {
            fs->curThrottle += abs(10*accErrorScalar);
        } else {
            fs->curThrottle -= abs(10*accErrorScalar);
        }*/
        float throttleChange = ((float)THROTTLE_GAIN*finalScalar*dt);
        if(throttleChange > 0.0f) {
            fs->curThrottle += throttleChange;
        } else {
            fs->curThrottle += THROTTLE_REDUCE_FACTOR*throttleChange;
        }
        fs->curThrottle = CLAMP(fs->curThrottle, 1000.0f, MAX_THROTTLE);
        fs->lastUpdateTick = curTick;
        fs->lastHeight = (float)curHeightCM;
    }
    return (uint16_t)fs->curThrottle;
}


/*uint16_t failsafe_autohover_update(FailsafeState *fs, uint32_t curTick, uint16_t curHeight) {
    // 100Hz, assuming 1 tick is 1ms
    uint32_t tickDif = curTick - fs->lastUpdateTick;
    if(tickDif >= 10) {
        float dt = (float)tickDif / 1000.0f; // time delta in sec
        float meas = (float)curHeight / 100.0f; // convert to m
        float out = pid_update(&fs->pid, meas, fs->holdHeight, dt); 
        fs->lastOutput = out;
        fs->lastUpdateTick = curTick;
    }
    // convert pid output to range 1000-2000 (first clamp pid output to range -1 to 1)
    // TODO: this should be done once!
    uint16_t throttle;
    throttle = (uint16_t)((CLAMP(fs->lastOutput, -1.0f, 1.0f) * 500.0f) + 1500);
    throttle = CLAMP(throttle, 1000, 1300);

    return throttle;
}*/
