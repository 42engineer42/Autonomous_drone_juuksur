#include <HeightEstimator.h>


#include <math.h>

void height_estimator_init(HeightEstimator *he) {
    //
    he->height = 2.0f;
    he->heightVel = 0.0f;
    he->accBias = 1170.0f;

    he->P.m11 = 0.0f; he->P.m12 = 0.0f; he->P.m13 = 0.0f;
    he->P.m21 = 0.0f; he->P.m22 = 0.0f; he->P.m23 = 0.0f;
    he->P.m31 = 0.0f; he->P.m32 = 0.0f; he->P.m33 = 0.0f;

    he->Q.m11 = 0.01f; he->Q.m12 = 0.0f; he->Q.m13 = 0.0f;
    he->Q.m21 = 0.0f; he->Q.m22 = 0.03f; he->Q.m23 = 0.0f;
    he->Q.m31 = 0.0f; he->Q.m32 = 0.0f; he->Q.m33 = 0.03f;

    he->R = 0.03f;
}

void mul_3x3(M3x3 *dst, M3x3 *m1, M3x3 *m2) {
    dst->m11 = m1->m11*m2->m11 + m1->m12*m2->m21 + m1->m13*m2->m31;
    dst->m12 = m1->m11*m2->m12 + m1->m12*m2->m22 + m1->m13*m2->m32;
    dst->m13 = m1->m11*m2->m13 + m1->m12*m2->m23 + m1->m13*m2->m33;
    dst->m21 = m1->m21*m2->m11 + m1->m22*m2->m21 + m1->m23*m2->m31;
    dst->m22 = m1->m21*m2->m12 + m1->m22*m2->m22 + m1->m23*m2->m32;
    dst->m23 = m1->m21*m2->m13 + m1->m22*m2->m23 + m1->m23*m2->m33;
    dst->m31 = m1->m31*m2->m11 + m1->m32*m2->m21 + m1->m33*m2->m31;
    dst->m32 = m1->m31*m2->m12 + m1->m32*m2->m22 + m1->m33*m2->m32;
    dst->m33 = m1->m31*m2->m13 + m1->m32*m2->m23 + m1->m33*m2->m33;
}

void M3x3_add(M3x3 *dst, M3x3 *A, M3x3 *B) {
    dst->m11 = A->m11 + B->m11;
    dst->m12 = A->m12 + B->m12;
    dst->m13 = A->m13 + B->m13;
    dst->m21 = A->m21 + B->m21;
    dst->m22 = A->m22 + B->m22;
    dst->m23 = A->m23 + B->m23;
    dst->m31 = A->m31 + B->m31;
    dst->m32 = A->m32 + B->m32;
    dst->m33 = A->m33 + B->m33;
}

void M3x3_scale(M3x3 *dst, M3x3 *m, float s) {
    dst->m11 = m->m11*s;
    dst->m12 = m->m12*s;
    dst->m13 = m->m13*s;
    dst->m21 = m->m21*s;
    dst->m22 = m->m22*s;
    dst->m23 = m->m23*s;
    dst->m31 = m->m31*s;
    dst->m32 = m->m32*s;
    dst->m33 = m->m33*s;
}

void transpose_3x3(M3x3 *dst, M3x3 *m) {
    dst->m11 = m->m11;
    dst->m12 = m->m21;
    dst->m13 = m->m31;
    dst->m21 = m->m12;
    dst->m22 = m->m22;
    dst->m23 = m->m32;
    dst->m31 = m->m13;
    dst->m32 = m->m23;
    dst->m33 = m->m33;
}

void height_estimator_predict(HeightEstimator *he, float acc, float dt) {
    //he->height += (he->heightVel*dt /*- 0.5f*acc*dt*dt*/);
    //he->heightVel += uacc*dt;
    he->height += dt*he->heightVel + 0.5f*dt*dt*(acc - he->accBias);
    he->heightVel += dt*(acc - he->accBias);

    M3x3 P = he->P;
    he->P.m11 = P.m11 + P.m21*dt + dt*he->Q.m11 - (dt*dt*(P.m13 + P.m23*dt - (P.m33*dt*dt)/2.0f))/2.0f - (P.m31*dt*dt)/2.0f + dt*(P.m12 + P.m22*dt - (P.m32*dt*dt)/2.0f);
    he->P.m12 = P.m12 + P.m22*dt - (P.m32*dt*dt)/2.0f - dt*(P.m13 + P.m23*dt - (P.m33*dt*dt)/2.0f);
    he->P.m13 = P.m13 + P.m23*dt - (P.m33*dt*dt)/2.0f;
    he->P.m21 = P.m21 - P.m31*dt + dt*(P.m22 - P.m32*dt) - (dt*dt*(P.m23 - P.m33*dt))/2.0f;
    he->P.m22 = P.m22 - P.m32*dt + dt*he->Q.m22 - dt*(P.m23 - P.m33*dt);
    he->P.m23 = P.m23 - P.m33*dt;
    he->P.m31 = P.m31 + P.m32*dt - (P.m33*dt*dt)/2.0f;
    he->P.m32 = P.m32 - P.m33*dt;
    he->P.m33 = P.m33 + dt*he->Q.m33;
}

void height_estimator_update(HeightEstimator *he, float height, float dt) {
    // H = [1 0 0]
    // y = z - H*x
    float y = height - he->height;

    float s = he->P.m11 + he->R;
    float k[3];

    k[0] = he->P.m11 / s;
    k[1] = he->P.m21 / s;
    k[2] = he->P.m31 / s;

    M3x3 P;
    P.m11 = he->P.m11 - k[0]*he->P.m11;
    P.m12 = he->P.m12 - k[0]*he->P.m12;
    P.m13 = he->P.m13 - k[0]*he->P.m13;
    P.m21 = he->P.m21 - k[1]*he->P.m11;
    P.m22 = he->P.m22 - k[1]*he->P.m12;
    P.m23 = he->P.m23 - k[1]*he->P.m13;
    P.m31 = he->P.m31 - k[2]*he->P.m11;
    P.m32 = he->P.m32 - k[2]*he->P.m12;
    P.m33 = he->P.m33 - k[2]*he->P.m13;

    he->P = P;

    he->height += k[0]*y;
    he->heightVel += k[1]*y;
    he->accBias += k[2]*y;
}
