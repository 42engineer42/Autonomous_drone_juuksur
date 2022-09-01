#include "VelocityEstimator.hpp"

#include <math.h>
#include <stdio.h>

VelocityEstimator::VelocityEstimator(double timestamp, float intFrame) {
    this->estimation = 0.0f;
    this->lastDerivative = 0.0f;
    this->curDerivative = 0.0f;
    this->estimationTimestamp = timestamp;
    this->integralTimeFrame = intFrame;

    this->estimationFromIntegrals = 0.0f;
}

void VelocityEstimator::updateIntegral(float value, double timestamp) {
    integrals.push_back((IntegralEntry){value, timestamp});

    while(integrals.size() > 0 
            && (timestamp-integralTimeFrame) > integrals.front().timestamp) {
        integrals.pop_front();
    }
    // linear regression
    if(integrals.size() >= 2) {
        // find xtx (xtranspose x)
        double xtx11, xtx12 = 0.0, xtx21, xtx22 = 0.0;
        double xty[2] = {0.0, 0.0}; // (xtranspose y)
        xtx11 = integrals.size();
        for(int i = 0; i < integrals.size(); i++) {
            double ts = integrals[i].timestamp;
            double val = integrals[i].value;
            xtx12 += ts;
            xtx22 += ts*ts;
            xty[0] += val;
            xty[1] += val*ts;
        }
        xtx21 = xtx12;

        double det = xtx11*xtx22-xtx12*xtx21;
        if(fabs(det) < 0.0000001) { // timestamps probably same for all samples (mistake in user code)
            this->estimationFromIntegrals = 0.0f;
            return;
        }
        double invDet = 1.0 / det;
        double xtxInv[2][2] = {{invDet*xtx22, invDet*(-xtx12)},{invDet*(-xtx21), invDet*xtx11}};

        // OPTIMIZE: not all calculated values are used!
        this->estimationFromIntegrals = (float)(xtxInv[1][0]*xty[0] + xtxInv[1][1]*xty[1]);
    } else {
        this->estimationFromIntegrals = 0.0f;
    }
}

void VelocityEstimator::updateDerivative(float value, double dimestamp) {
    this->curDerivative = value;
}

void VelocityEstimator::update(double timestamp) {
    double dt = timestamp - estimationTimestamp;
    float errorFromIntegral = this->estimationFromIntegrals - this->estimation;
    // trapezoid rule
    float dV = (this->lastDerivative+this->curDerivative)*0.5f*dt;
    // complementary filter
    this->estimation += dV*DWeight + errorFromIntegral*IWeight;

    this->estimationTimestamp = timestamp;
    this->lastDerivative = this->curDerivative;
}

float VelocityEstimator::estimate(double timestamp) {
    double dt = timestamp - estimationTimestamp;
    return this->estimation + (DWeight > 0.0f ? dt*this->lastDerivative : 0.0f);
}

