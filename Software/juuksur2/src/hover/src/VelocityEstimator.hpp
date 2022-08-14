#pragma once

#include <deque>

struct IntegralEntry {
    float value;
    double timestamp;
};

class VelocityEstimator {
public:
    VelocityEstimator(double, float);
    void updateIntegral(float value, double timestamp);
    void updateDerivative(float value, double timestamp);
    float estimate(double timestamp);
    void update(double timestamp);
private:
    float estimation;
    float lastDerivative;
    float curDerivative;
    double estimationTimestamp;

    float integralTimeFrame;

    float estimationFromIntegrals;

    std::deque<IntegralEntry> integrals;

    const float DWeight = 0.0f;
    const float IWeight = 1.0f;
};

