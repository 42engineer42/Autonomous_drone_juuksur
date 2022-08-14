#include "feature_tracker.hpp"

#include "params.hpp"

FeatureTracker::FeatureTracker(int sw, int sh) {
    _width = sw;
    _height = sh;
}

void FeatureTracker::init() {
    _tracking = false;
    _foundFeature = false;
}

void FeatureTracker::setCamHeight(float h) {
    _camHeight = h;
}

void FeatureTracker::opticalFlow(std::vector<cv::Point2f> prevFeatures, std::vector<cv::Point2f> newFeatures) {
    if(_tracking && _foundFeature == true) {
        float minD = FLT_MAX;
        int best = -1;
        for(int i = 0; i < prevFeatures.size(); i++) {
            auto ftr = prevFeatures[i]; 
            float xd = ftr.x - _trackedFeature.x;
            float yd = ftr.y - _trackedFeature.y;
            float d = xd*xd + yd*yd;
            if(d < minD) {
                best = i;
                minD = d;
            }
        }
        // NOTE: minD is distance squared (to avoid unneccesary sqrt)
        if(best == -1 || minD > 10.0f*10.0f) {
            // could not found a matching feature, pick a new one
            _foundFeature = false;
        } else {
            _trackedFeature = (V2){newFeatures[best].x, newFeatures[best].y};
        }
    }

    _curFeatures.clear();
    for(int i = 0; i < newFeatures.size(); i++) {
        auto ftr = newFeatures[i];
        _curFeatures.push_back((V2){ftr.x, ftr.y});
    }

    if(_tracking && _foundFeature == false) {
        pickAndStartTrackingFeature();
    }
}

void FeatureTracker::pickAndStartTrackingFeature() {
    _tracking = true;
    V2 center = (V2){ (float)_width / 2.0f, (float)_height / 2.0f };
    float minD = FLT_MAX;
    int best = -1;
    for(int i = 0; i < _curFeatures.size(); i++) {
        auto ftr = _curFeatures[i];
        float xd = ftr.x - center.x;
        float yd = ftr.y - center.y;
        float d = xd*xd + yd*yd;
        if(d < minD) {
            best = i;
            minD = d;
        }
    }
    if(best != -1) {
        _foundFeature = true;
        _trackedFeature = _curFeatures[best];
        _trackedFeatureStart = _trackedFeature;
        _trackedFeatureStartHeight = _camHeight;
    } else {
        _foundFeature = false;
    }
}

void FeatureTracker::stopTracking() {
    _tracking = false;
    _foundFeature = false;
}

bool FeatureTracker::isTracking() {
    return _tracking;
}

bool FeatureTracker::getTrackedFeature(V2& startPosS, V2& curPosS) {
    if(_tracking && _foundFeature) {
        // project to currentHeight (assuming flat ground and roll pitch are 0)
        float thf = tanf(HORIZONTAL_FOV_RAD*0.5f);
        float w1 = 2.0f * _trackedFeatureStartHeight * thf;
        float w2 = 2.0f * _camHeight * thf;
        float h1 = 2.0f * _trackedFeatureStartHeight * thf;
        float h2 = 2.0f * _camHeight * thf;
        // offset from center in cm
        float dx = ((_trackedFeatureStart.x - IMAGE_CENTER.x) / (NATIVE_WIDTH/DOWNSCALE_DIVIDER)) * w1;
        float dy = ((_trackedFeatureStart.y - IMAGE_CENTER.y) / (NATIVE_HEIGHT/DOWNSCALE_DIVIDER)) * h1;
        // offset from destination height center in pixels
        float dxpx = (dx/w2)*(NATIVE_WIDTH/DOWNSCALE_DIVIDER);
        float dypx = (dy/h2)*(NATIVE_HEIGHT/DOWNSCALE_DIVIDER);

        startPosS = (V2){dxpx + IMAGE_CENTER.x, dypx + IMAGE_CENTER.y};

        curPosS = _trackedFeature;
        return true;
    }
    return false;
}
