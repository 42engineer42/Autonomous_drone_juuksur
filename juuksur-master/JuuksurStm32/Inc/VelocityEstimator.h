#pragma once

typedef struct VelocityEstimator {
    float velX, velY;
    float accBiasX, accBiasY;

    float Px[2][2], Py[2][2];
    // using same covariance values for both
    float Qv;
    float Qb;
    float R;

    // debug
    float kkx[2];
} VelocityEstimator;

void velocity_estimator_init(VelocityEstimator *ve);
void velocity_estimator_predict(VelocityEstimator *ve, float accX, float accY, float dt);
void velocity_estimator_update(VelocityEstimator *ve, float velX, float velY, float dt);


