#include <stdint.h>
#include <float.h>
#include <ttmath.h>

#include "params.hpp"

class VelocityController {
public:
    VelocityController();
    void reset();
    void setActive(bool enabled);
    void update(V2 targetDirection, float targetSpeed, V2 currentVelocity, double dt);

    uint16_t controlRoll;
    uint16_t controlPitch;

    float curVelocity = 0.0f;
private:

    bool active = false;

    float integralX;
    float integralY;

    float setVelocity = 0.0f;
};

