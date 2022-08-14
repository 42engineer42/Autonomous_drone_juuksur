#include "BezierSpline.hpp"

BezierSpline::BezierSpline() {
}

BezierSpline::~BezierSpline() {
    delete[](_points);
}

void BezierSpline::Init(V2 *points, int count, bool loop) {
    this->loop = loop;
    _points = new V2[count];
    pointCount = count;
    for(int i = 0; i < count; i++) {
        _points[i] = points[i];
    }
}

V2 BezierSpline::GetPoint(int idx) {
    if(idx >= 0 && idx < pointCount) {
        return _points[idx];
    } else {
        if(loop) {
            int ret = idx % pointCount;
            ret = ret < 0 ? pointCount+ret : ret;
            return _points[ret];
        } else {
            if(idx < 0) {
                float dist = fabs(idx);
                V2 dir = (_points[0]-_points[1]);
                v2_normalize(&dir);
                return _points[0] + dist*dir;
            } else {
                float dist = idx - (pointCount - 1);
                V2 dir = (_points[pointCount-1] - _points[pointCount-2]);
                v2_normalize(&dir);
                return _points[pointCount-1] + dist*dir; 
            }
        }
    }
}

V2 BezierSpline::Evaluate(float t) {
    int startIdx = floorf(t);
    t = t - (float)startIdx;

    V2 prevP = GetPoint(startIdx-1);
    V2 startP =  GetPoint(startIdx);
    V2 endP = GetPoint(startIdx+1);
    V2 nextP = GetPoint(startIdx+2);

    float len = v2_len(startP-endP);
    V2 prevToNext = endP - prevP;
    V2 curToNext = startP - nextP;
    v2_normalize(&prevToNext);
    v2_normalize(&curToNext);
    V2 tangentPoint1 = startP + 0.5f*len*prevToNext;
    V2 tangentPoint2 = endP + 0.5f*len*curToNext;

    float omt = 1.0f - t;
    return omt*omt*omt*startP+3.0f*omt*omt*t*tangentPoint1+3*omt*t*t*tangentPoint2+t*t*t*endP;
}

V2 BezierSpline::Derivative(float t) {
    int startIdx = floorf(t);
    t = t - (float)startIdx;

    V2 prevP = GetPoint(startIdx-1);
    V2 startP =  GetPoint(startIdx);
    V2 endP = GetPoint(startIdx+1);
    V2 nextP = GetPoint(startIdx+2);

    float len = v2_len(startP-endP);
    V2 prevToNext = endP - prevP;
    V2 curToNext = startP - nextP;
    v2_normalize(&prevToNext);
    v2_normalize(&curToNext);
    V2 tangentPoint1 = startP + 0.5f*len*prevToNext;
    V2 tangentPoint2 = endP + 0.5f*len*curToNext;

    float omt = 1.0f - t;
    return -3.0f*omt*omt*startP + 3.0f*omt*omt*tangentPoint1 - 6.0f*t*omt*tangentPoint1 - 3.0f*t*t*tangentPoint2 + 6.0f*t*omt*tangentPoint2 + 3.0f*t*t*endP;
}
