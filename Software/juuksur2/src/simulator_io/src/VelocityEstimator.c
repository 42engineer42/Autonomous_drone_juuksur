#include "VelocityEstimator.h"

void velocity_estimator_init(VelocityEstimator *ve) {
    ve->velX = 0.0f;
    ve->velY = 0.0f;

    ve->accBiasX = 0.0f;
    ve->accBiasY = 0.0f;
    
    // set current confidence high, because we are quite confident we're not moving at startup
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            ve->Px[i][j] = 0.0f;
            ve->Py[i][j] = 0.0f;
        }
    }

    // velocity covariance
    ve->Qv = 0.002f;
    // acceleration bias covariance
    ve->Qb = 0.006f;
    // measurement covariance
    ve->R = 0.01f;
}

void velocity_estimator_predict(VelocityEstimator *ve, float accX, float accY, float dt) {
    ve->velX += dt*(accX-ve->accBiasX);
    ve->velY += dt*(accY-ve->accBiasY);

    ve->Px[0][0] += dt*(ve->Px[1][1]*dt + ve->Qv - ve->Px[0][1] - ve->Px[1][0]);
    ve->Px[0][1] -= dt*ve->Px[1][1];
    ve->Px[1][0] -= dt*ve->Px[1][1];
    ve->Px[1][1] += dt*ve->Qb;

    ve->Py[0][0] += dt*(ve->Py[1][1]*dt + ve->Qv - ve->Py[0][1] - ve->Py[1][0]);
    ve->Py[0][1] -= dt*ve->Py[1][1];
    ve->Py[1][0] -= dt*ve->Py[1][1];
    ve->Py[1][1] += dt*ve->Qb;
}

void velocity_estimator_update(VelocityEstimator *ve, float velX, float velY, float dt) {
    float yx = velX - ve->velX;
    float yy = velY - ve->velY;

    float skx = ve->Px[0][0] + ve->R;
    float sky = ve->Py[0][0] + ve->R;

    float kkx[2], kky[2];
    kkx[0] = ve->Px[0][0]/skx;
    kkx[1] = ve->Px[1][0]/skx;
    kky[0] = ve->Py[0][0]/sky;
    kky[1] = ve->Py[1][0]/sky;

    //debug
    ve->kkx[0] = kkx[0];
    ve->kkx[1] = kkx[1];

    ve->velX += kkx[0]*yx;
    ve->accBiasX += kkx[1]*yx;
    ve->velY += kky[0]*yy;
    ve->accBiasY += kky[1]*yy;

    float Px00 = ve->Px[0][0];
    float Px01 = ve->Px[0][1];
    float Py00 = ve->Py[0][0];
    float Py01 = ve->Py[0][1];

    ve->Px[0][0] -= kkx[0]*Px00;
    ve->Px[0][1] -= kkx[0]*Px01;
    ve->Px[1][0] -= kkx[1]*Px00;
    ve->Px[1][1] -= kkx[1]*Px01;
    ve->Py[0][0] -= kky[0]*Py00;
    ve->Py[0][1] -= kky[0]*Py01;
    ve->Py[1][0] -= kky[1]*Py00;
    ve->Py[1][1] -= kky[1]*Py01;

}

