#pragma once

typedef struct M3x3 {
    float m11, m12, m13;
    float m21, m22, m23;
    float m31, m32, m33;
} M3x3;

typedef struct HeightEstimator {
    float height;
    float heightVel;
    float accBias;

    M3x3 P;
    M3x3 Q;

    float R;
} HeightEstimator;

void height_estimator_init(HeightEstimator *he);
void height_estimator_predict(HeightEstimator *he, float acc, float dt);
void height_estimator_update(HeightEstimator *he, float height, float dt);
