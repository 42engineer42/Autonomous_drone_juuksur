#include "HeightController.hpp"

#include <float.h>
#include <stdio.h>

// max vertical speed in cm/s
#define MAX_SPEED 30.0f
// max acceleration in cm/s^2
#define MAX_ACC 200.f // cm/s^2
// throttle when we turn on motors for takeoff
#define THROTTLE_GAIN 0.3f
// maximum throttle value we are allowed to use (for safety)
#define MAX_THROTTLE 0.555f
// unused?
#define DESCENT_SPEED 30.0f
// how close to target height do we start decelarting
#define DECELERATION_RANGE 100.0f

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))


TOHeightController::TOHeightController() : HeightController() {
    this->reset();
}

void TOHeightController::heightUpdate(float newheight, double timestamp) {
    // TODO: some kind of filter or averaging?
    this->lastHeight = newheight;
}

void TOHeightController::accelerationUpdate(float newacc, double timestamp) {
    this->acceleration = newacc;
}

void TOHeightController::velocityUpdate(float newvel,  double timestamp) {
    this->speed = newvel;
}

void TOHeightController::setTarget(float t) {
    this->targetHeight = t;
}

void TOHeightController::update(double timestamp) {
    double dt = lastUpdateTimestamp == 0.0 ? 0.0 : timestamp - lastUpdateTimestamp;
    float heightError = targetHeight - lastHeight;
    float heightErrorS = CLAMP(heightError / DECELERATION_RANGE, -1.0f, 1.0f);

    float speedTarget = MAX_SPEED * heightErrorS;
    float speedError = speedTarget - speed;
    float speedErrorS = CLAMP(speedError/MAX_SPEED, -1.0f, 1.0f);

    float accTarget = (MAX_ACC*speedErrorS) + 995.0f;
    float accError = accTarget - acceleration;
    float finalScalar = accError / MAX_ACC;

    float throttleChange = ((float)THROTTLE_GAIN*finalScalar*dt);
    this->curThrottle += throttleChange;
    this->curThrottle = CLAMP(curThrottle, 0.0f, 1.0f);
    this->lastUpdateTimestamp = timestamp;
}

void TOHeightController::reset() {
    this->speed = 0.0f;
    this->acceleration = 995.0f;
    this->lastHeight = FLT_MAX;
    this->targetHeight = 0.0f;
    this->lastUpdateTimestamp = 0.0;
    this->curThrottle = 0.0f;
}

float TOHeightController::getThrottle() {
    return this->curThrottle;
}
