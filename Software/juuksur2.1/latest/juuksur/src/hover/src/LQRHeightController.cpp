#include "HeightController.hpp"

#include <float.h>
#include <stdio.h>
#include <math.h>

#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MAX(a,b) ((a)>(b)?(a):(b))

LQRHeightController::LQRHeightController() : HeightController() {
    this->reset();
}

void LQRHeightController::heightUpdate(float newheight, double timestamp) {
    height = newheight;
}

void LQRHeightController::accelerationUpdate(float newacc, double timestamp) {
    this->acc = newacc;
}

void LQRHeightController::velocityUpdate(float newvel, double timestamp) {
    this->velocity = newvel;
}

void LQRHeightController::setTarget(float t) {
    this->setPoint = t;
}

#define SIGN(x) ((x)>=0?1.0f:-1.0f)

void LQRHeightController::update(double timestamp) {
    float dt = prevTimeStamp >= 0.0f ? timestamp - prevTimeStamp : 0.0f;

    // TODO: calculate velocity (average prev x seconds and add acelerometer data?)
    //

    float pos = height - this->setPoint;
    this->integral += pos*dt;
    this->integral = CLAMP(this->integral, -100.0f, 100.0f);

    float u = K1*pos + K2*this->velocity + K3*this->integral;
    float beforeClamp = -u;
    u = CLAMP(-u, -980.0f, 980.0f); // negative = down, positive = up

    float targetAcc = 981.0f+u; // u=-981->0(0G) u=981->1962(2G)
    /*if(fabs(this->velocity) > MAX_SPEED) {
        // positive velocity -> up (reduce targetAcc if over limit)
        // we could set targetAcc to 981, but if accelerometer is not accurate we'd still accelerate
        float overLimit = MAX(fabs(this->velocity) - MAX_SPEED, 0.0f);
        printf("over %f %f %f\r\n", overLimit, this->velocity, MAX_SPEED);
        targetAcc = 981.0f + (this->velocity > 0 ? -overLimit : overLimit); 
    }*/
    printf("targetAcc %f (%f) h %f sp %f vel %f i %f\r\n", targetAcc, this->acc, this->height, this->setPoint, this->velocity, this->integral);

    printf("output %f %f\r\n", ((targetAcc-this->acc)/981.0f)*dt*THROTTLE_GAIN, this->output);

    // current acc smaller than target -> more throttle
    this->output += ((targetAcc-this->acc)/981.0f)*dt*THROTTLE_GAIN;
    this->output = CLAMP(this->output, 0.0f, 1.0f);

    // TODO: use a lookup table for initial output (to reduce response time)
    
    prevTimeStamp = timestamp;
}

void LQRHeightController::reset() {
    this->height = 0.0f;
    this->setPoint = 0.0f;
    this->acc = 981.0f;
    this->output = 0.0f;
    this->integral = 0.0f;
    this->prevTimeStamp = -1.0f;
    this->velocity = 0.0f;
}

float LQRHeightController::getThrottle() {
    return this->output;
}
