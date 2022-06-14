#pragma once

#include <vector>
#include <limits>

#include <float.h>
#include <ttmath.h>
#include <opencv/cv.h>

class FeatureTracker {
    public:
        FeatureTracker(int screenW, int screenH);
        void init();
        // NOTE: features must be already matched (by array index)
        void setCamHeight(float h);
        void opticalFlow(std::vector<cv::Point2f> prevFeatures, std::vector<cv::Point2f> newFeatures);
        void pickAndStartTrackingFeature();
        void stopTracking();
        bool getTrackedFeature(V2& startPosS, V2& curPosS);
        bool isTracking();
    private:
        V2 _trackedFeature; // current position in screen coordinates
        V2 _trackedFeatureStart; // position where we started to track the feature
        float _trackedFeatureStartHeight;
        bool _tracking, _foundFeature;
        float _camHeight;

        int _width, _height;
        std::vector<V2> _curFeatures;
};

