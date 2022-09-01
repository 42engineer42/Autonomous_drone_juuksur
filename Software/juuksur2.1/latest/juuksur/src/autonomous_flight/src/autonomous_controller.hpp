#pragma once

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "structures.hpp"

#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <vector>
#include <fstream>
#include <ctime>
#include <opencv/cv.h>
#include <chrono>

#include "localization.hpp"
#include "velocity_controller.hpp"
#include "params.hpp"


class AutonomousController {
    public:
        AutonomousController(std::function<void(cv::Mat&, cv::Mat&)> debugImg);
        ~AutonomousController();
        void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
        void rollCallback(const std_msgs::Float32 msg);
        void pitchCallback(const std_msgs::Float32 msg);
        void rangeCallback(const std_msgs::Float32 msg);
        void throttleCallback(const std_msgs::UInt16 msg);

        bool isActive();
        void start();
        void stop();

        uint16_t controlRoll;
        uint16_t controlPitch;

    private:


        void robotexImage(cv::Mat& grayFrame, double dt);
        void followImage(cv::Mat& grayFrame, double dt);
        void deltaImage(cv::Mat& grayFrame, double dt);

        int filterContour(cv::Mat debugImage, cv::RotatedRect r, double area, float matchedRectArea, float aspect);
        V4 calculateGroundPlane(float dist, float roll, float pitch);
        void camToWorldRotation(Mat3 *dst, float roll, float pitch);
        void controlMovementFromLocation(IVec2 curPosS, double dt);
        void controlMovementToTarget(V2 targetPosS, IVec2 curPosS, double dt, float maxSpeed);
        void controlMovementToDirection(V2 targetDirection, IVec2 curPosS, V2 biasPoint, double dt);
        void calculateMovement(cv::Mat& grayFrame, float dt);
        void updateContours(cv::Mat& grayFrame);
        IVec2 findClosestContourCenter(IVec2 imagePoint);
        int findClosestContourCenterIdx(IVec2 imagePoint);
        bool findClosestPointOnTrack(cv::Mat& rgbImage, V2 *point, int* edge);
        bool findFurthestConnectedNode(cv::Mat& rgbImage, V2 *point, int* retNode, V2 *dir);
        bool findWaypoint(cv::Mat& rgbImage, V2 *wp, V2 cpOnTrack);
        void updateTrackEdges(cv::Mat& rgbImage);
        V2 calculateOpticalFlow(cv::Mat from, cv::Mat to, cv::Mat debugImage, Mat3* screenToWorld,V4 groundPlane, float dt);
        V2 findTargetPoint();
        V2 findTargetDirection();
        void reset();
        void drawTelemetry(cv::Mat& debugImage);

        Localization loc;

        std::function<void(cv::Mat&, cv::Mat&)> debugImg;

        VelocityController velCtrl;

        bool active = false;

        int mode = 0;

        float roll = 0.0f;
        float pitch = 0.0f;
        float throttle = 1000.0f;
        float distanceToGround = 0.0f;

        float targetIntegralX = 0.0f;
        float targetIntegralY = 0.0f;
        float targetSIntegralX = 0.0f;
        float targetSIntegralY = 0.0f;

        float targetVIntegralX = 0.0f;
        float targetVIntegralY = 0.0f;

        // x,y movement calculated from camera images
        V2 movement = (V2){0.0f, 0.0f};

        V2 screenMovement = (V2){0.0f, 0.0f};

        // moving average count for horizontal velocity estimation (very noisy otherwise)
        static const int H_MOV_AVG_COUNT = 4;
        V2 hMovAvg[H_MOV_AVG_COUNT] = {{0}};
        int movAvgIdx = 0;

        // target direction on image
        V2 targetDirection; // default set in reset()
        // closest point to ground in image coordinates
        V2 cpToGroundS; // default set in reset()

        // camera space to screen space
        Mat3 cam2Screen;
        // screen space to camera space ray
        Mat3 screen2Cam;
        
        Mat3 screenToWorld;
        Mat3 screenToWorldPrev;
        V4 groundPlane;

        std::vector<std::vector<cv::Point>> contours;

        std::vector<ContourCenter>* contourCenters2;
        std::vector<ContourCenter>* prevContourCenters2;

        // pairs of track indices pointing to contourCenters vector
        // NB! contains double edges (for now)
        std::vector<int> trackIndices;

        std::ofstream logfile;

        cv::Mat grayFramePrev;
        cv::Mat binaryFrame;

		cv::Mat debugImage;
		cv::Mat debugImageSmall;

		std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point lastFrame;
		int totalImagesReceived = 0;

};
