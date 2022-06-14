#include "velocity_controller.h"

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define SIGN(x) ((x) >= 0.0f ? 1.0f : -1.0f)
#define ABS(x) ((x) < 0.0f ? -(x) : (x))

void velocity_controller_init(VelocityController *vc) {
    vc->mode = 1;
    vc->intX = 0.0f;
    vc->intY = 0.0f;
    vc->intEX = 0.0f;
    vc->intEY = 0.0f;
    vc->setpointX = 0.0f;
    vc->setpointY = 0.0f;
    vc->rollControl = 1500;
    vc->pitchControl = 1500;
}

void velocity_controller_update(VelocityController *vc, float velX, float velY, float accX, float accY, float dt) {
    switch(vc->mode) {
    default:
    case 0: { // control velocity directly
    float errorX = vc->setpointX - velX;
    float px = errorX * VELOCITY_CONTROLLER_P;
    vc->intX += errorX * VELOCITY_CONTROLLER_I * dt;
    float dx = (-accX) * VELOCITY_CONTROLELR_D;
    float controlX = px + vc->intX + dx + VELOCITY_CONTROLLER_BIAS;
    controlX = CLAMP(controlX, VELOCITY_CONTROLLER_BIAS-VELOCITY_CONTROLLER_MAX, VELOCITY_CONTROLLER_BIAS+VELOCITY_CONTROLLER_MAX);
    //controlX = VELOCITY_CONTROLLER_BIAS;
    vc->pitchControl = (uint16_t)controlX;

    float errorY = vc->setpointY - velY;
    float py = errorY * VELOCITY_CONTROLLER_P;
    vc->intY += errorY * VELOCITY_CONTROLLER_I * dt;
    float dy = (-accY) * VELOCITY_CONTROLELR_D;
    float controlY = -py - vc->intY - dy + VELOCITY_CONTROLLER_BIAS;
    controlY = CLAMP(controlY, VELOCITY_CONTROLLER_BIAS-VELOCITY_CONTROLLER_MAX, VELOCITY_CONTROLLER_BIAS+VELOCITY_CONTROLLER_MAX);
    //controlY = VELOCITY_CONTROLLER_BIAS;
    vc->rollControl = (uint16_t)controlY;
    break; }
    case 1: {// control velocity through acceleration
    float verrorX = vc->setpointX - velX; 
    float targetAX = VELOCITY_CONTROLLER_M1_EP * verrorX;
    float accErrorX = targetAX - accX;
    float px = VELOCITY_CONTROLLER_M1_AP * accErrorX;
    vc->intEX += accErrorX * VELOCITY_CONTROLLER_M1_AI;
    float controlX = px + vc->intEX + VELOCITY_CONTROLLER_BIAS;
    controlX = CLAMP(controlX, VELOCITY_CONTROLLER_BIAS-VELOCITY_CONTROLLER_MAX, VELOCITY_CONTROLLER_BIAS+VELOCITY_CONTROLLER_MAX);
    vc->pitchControl = (uint16_t)controlX;

    float verrorY = vc->setpointY - velY; 
    float targetAY = VELOCITY_CONTROLLER_M1_EP * verrorY;
    float accErrorY = targetAY - accY;
    float py = VELOCITY_CONTROLLER_M1_AP * accErrorY;
    vc->intEY += accErrorY * VELOCITY_CONTROLLER_M1_AI;
    float controlY = -py - vc->intEY + VELOCITY_CONTROLLER_BIAS;
    controlY = CLAMP(controlY, VELOCITY_CONTROLLER_BIAS-VELOCITY_CONTROLLER_MAX, VELOCITY_CONTROLLER_BIAS+VELOCITY_CONTROLLER_MAX);
    vc->rollControl = (uint16_t)controlY;

    break;}
    }
}
