#pragma once

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "ttmath.h"

#include <std_msgs/Float32.h>
#include <vector>
#include <opencv/cv.h>

struct IVec2 {
    int x,y;
    IVec2(int x, int y) {
        this->x = x;
        this->y = y;
    }
};

class AutonomousController {
    public:
        AutonomousController();
        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void rollCallback(const std_msgs::Float32 msg);
        void pitchCallback(const std_msgs::Float32 msg);
        void rangeCallback(const std_msgs::Float32 msg);

        float controlRoll;
        float controlPitch;
    private:
        float roll;
        float pitch;
        float distanceToGround;

        // camera space to screen space
        Mat3 cam2Screen;
        // screen space to camera space ray
        Mat3 screen2Cam;
        std::vector<std::vector<cv::Point>> contours;
        std::vector<IVec2> contourCenters;

        const int NATIVE_WIDTH = 3280;
        const int NATIVE_HEIGHT = 2464;
        const int DOWNSCALE_DIVIDER = 8; // we drownscale to 410x308 from 3280x2464
        const float FOCAL_LENGTH = 3.04f; // in mm
        const float PIXELS_PER_MM = 892.857142857f; // at native resolution
};
