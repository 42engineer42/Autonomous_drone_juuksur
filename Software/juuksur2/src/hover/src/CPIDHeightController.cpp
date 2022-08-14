#include "HeightController.hpp"

#include <stdio.h>

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

CPIDHeightController::CPIDHeightController() : HeightController() {
    this->reset();
}

void CPIDHeightController::reset() {
    this->targetHeight = 0.0f;
    this->velocity = 0.0f;
    this->velocityIntegral = this->KIV > 0.0f ? this->START_THROTTLE / this->KIV : 0.0f; // start out at about 20-30% throttle
    this->prevTimeStamp = 0.0;
}

void CPIDHeightController::setTarget(float target) {
    this->targetHeight = target;
}

void CPIDHeightController::accelerationUpdate(float newacc, double timestamp) {
    //this->acceleration = newacc - 981.0f;
}

void CPIDHeightController::velocityUpdate(float newvel, double timestamp) {
    this->velocity = newvel;
}

void CPIDHeightController::heightUpdate(float newh, double timestamp) {
    this->height = newh;
}

void CPIDHeightController::update(double timestamp) {
    double dt = (prevTimeStamp != 0.0 ?  timestamp - this->prevTimeStamp : 0.0);

    float heiE = this->targetHeight - this->height;
    // TODO: outer PD loop that targets height and controls velocity
    float velSP = this->KPH*heiE;

    float velEF = velSP - this->velocity;
    float velE = CLAMP(velEF, -50.0f, 50.0f);
    this->velocityIntegral += velE * dt;
/*    float der;
    if(dt > 0.0) {
        der = velSP > this->velocity ? acceleration : -acceleration;
    } else {
        der = 0.0f;
    }*/
    this->output = this->KPV*velE + this->KIV*this->velocityIntegral;
    this->output = CLAMP(this->output, 0.0f, 1.0f);

    this->prevTimeStamp = timestamp;
}

float CPIDHeightController::getThrottle() {
    return this->output;
}
