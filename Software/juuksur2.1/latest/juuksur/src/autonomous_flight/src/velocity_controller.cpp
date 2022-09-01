#include "velocity_controller.hpp" 

#include <stdio.h>

#define SIGNF(x) ((x) >= 0 ? 1.0f : -1.0f)
#define MIN(a,b) ((a) < (b) ? (a) : (b))

VelocityController::VelocityController() {
    reset();
}

void VelocityController::setActive(bool enabled) {
    this->active = enabled;
}

void VelocityController::reset() {
    integralX = 0.0f;
    integralY = 0.0f;
    controlRoll = 1500;
    controlPitch = 1500;
}

void VelocityController::update(V2 targetDirection, float targetSpeed, V2 currentVelocity, double dt) {
    this->setVelocity = targetSpeed;

    if(std::isnan(targetDirection.x) || std::isnan(targetDirection.y) || std::isnan(targetSpeed)) {
        printf("VelocityController targetDirection was NAN!\r\n");
        targetDirection = (V2){0.0f, 0.0f};
    }
    if(std::isnan(currentVelocity.x) || std::isnan(currentVelocity.y)) {
        printf("VelocityController currentVelocity was NAN!\r\n");
        currentVelocity = (V2){0.0f, 0.0f};
    }

    float dif = targetSpeed - curVelocity;
    this->curVelocity += SIGNF(dif) * MIN(fabs(dif), MAX_ACCELERATION*dt);

    V2 targetVelocity = (V2){targetDirection.x*this->curVelocity, targetDirection.y*this->curVelocity};

    if(this->active) {
        float errorX = targetVelocity.x - currentVelocity.x;
        float rollD = HVEL_KP*errorX + HVEL_KI*this->integralX;
        float beforeClamp = rollD;
        rollD = TT_CLAMP(rollD, -MAX_CONTROL_ROLL, MAX_CONTROL_ROLL);
        bool rollSaturated = (rollD != beforeClamp);
        //if(!rollSaturated | SIGNF(rollD) != SIGNF(errorX)) {
            this->integralX += dt*errorX;
        //}

        float errorY = targetVelocity.y - currentVelocity.y;
        float pitchD = HVEL_KP*errorY + HVEL_KI*this->integralY;
        beforeClamp = pitchD;
        pitchD = TT_CLAMP(pitchD, -MAX_CONTROL_PITCH, MAX_CONTROL_PITCH);
        bool pitchSaturated = (pitchD != beforeClamp);
        //if(!pitchSaturated || SIGNF(pitchD) != SIGNF(errorY)) {
            this->integralY += dt*errorY;
        //}

        this->controlPitch = (uint16_t)(1500.0f - pitchD);
        this->controlRoll = (uint16_t)(1500.0f + rollD);
    } else {
        this->controlPitch = 1500;
        this->controlRoll = 1500;
    }
}
