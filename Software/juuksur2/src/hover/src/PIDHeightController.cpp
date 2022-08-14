#include "HeightController.hpp"

#include <float.h>
#include <stdio.h>
#include <math.h>

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

PIDHeightController::PIDHeightController() : HeightController() {
    this->reset();
}

void PIDHeightController::heightUpdate(float newheight, double timestamp) {
    value = newheight;
}

void PIDHeightController::accelerationUpdate(float newacc, double timestamp) {
    // TODO: implement
}

void PIDHeightController::velocityUpdate(float newvel, double timestamp) {
    this->velocity = newvel;
}

void PIDHeightController::setTarget(float t) {
    this->setPoint = t;
}

#define SIGNF(x) ((x)>=0?1.0f:-1.0f)

void PIDHeightController::update(double timestamp) {
    if(this->prevUpdateTime != 0.0) {
        double dt = timestamp - prevUpdateTime;

        float error = setPoint - value;
        float derivative = (error - this->prevError) > 0.0f ? fabs(this->velocity) : -fabs(this->velocity);

        this->prevError = error;
        this->output = this->KP*error + this->KI*integral + this->KD*derivative + 0.33f; 
        float preclamp = this->output;
        this->output = CLAMP(this->output, 0.0f, 1.0f);
        bool saturated = preclamp != this->output;
        this->integral += error*dt;
        float maxIntegral = 0.1f/this->KI;
        this->integral = CLAMP(this->integral, -maxIntegral, maxIntegral);
        /*if(saturated && SIGNF(this->output-0.33f) == SIGNF(error)) { // turn of integral / anti-windup
            this->output = CLAMP(this->KP*error + this->KD*derivative + 0.33f, 0.0f, 1.0f);
        } else {
            this->integral += error*dt;
        }*/
        //printf("dt %.7f output %.4f error %.4f p %.4f i %.4f d %.4f\r\n", dt, this->output, error, KP*error, KI*integral, KD*derivative);
    }
    this->prevUpdateTime = timestamp;
}

void PIDHeightController::reset() {
    this->setPoint = 0.0f;
    this->prevError = 0.0f;
    this->integral = 0.0f;
    this->output = 0.0f;
    this->value = 0.0f;
    this->prevUpdateTime = 0.0;
    this->velocity = 0.0f;
}

float PIDHeightController::getThrottle() {
    return this->output;
}
