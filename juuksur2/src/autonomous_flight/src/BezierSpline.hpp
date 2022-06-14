#pragma once

#include "float.h"
#include "ttmath.h"

class BezierSpline {
public:
    BezierSpline();
    ~BezierSpline();
    void Init(V2 *point, int count, bool loop);
    V2 Evaluate(float t);
    V2 Derivative(float t);
private:
    V2 GetPoint(int idx);

    V2 *_points = nullptr;
    int pointCount = 0;
    bool loop = false;
};
