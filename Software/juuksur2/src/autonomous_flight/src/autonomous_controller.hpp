#pragma once

#include <dev_msgs/CameraFrame.h>

#include "structures.hpp"

#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <vector>
#include <fstream>
#include <ctime>
#include <opencv/cv.h>
#include <opencv2/features2d.hpp>
#include <chrono>
#include <memory>

#include "localization.hpp"
#include "feature_tracker.hpp"
#include "params.hpp"


class AutonomousController {
    public:
        AutonomousController(std::function<void(cv::Mat&, cv::Mat&)> debugImg);
        ~AutonomousController();
        void imageCallback(const dev_msgs::CameraFrame::ConstPtr& msg);
        void rollCallback(const std_msgs::Float32 msg);
        void pitchCallback(const std_msgs::Float32 msg);
        void rangeCallback(const std_msgs::Float32 msg);
        void velXCallback(const std_msgs::Float32 msg);
        void velYCallback(const std_msgs::Float32 msg);
        void accXCallback(const std_msgs::Float32 msg);
        void accYCallback(const std_msgs::Float32 msg);

        bool isActive();
        void start();
        void stop();

        uint16_t controlRoll;
        uint16_t controlPitch;
        V2 movementRelativeToEnvironment = (V2){0.0f, 0.0f};
        V2 controlVelocity = (V2){0.0f, 0.0f};
        // for deltaX platform following
        bool hoveringPlatform = false;
        bool movementRelEDirty = true;
        bool testDirty = true;
        bool clearToLand = true;
        bool landing = false;
        V2 testV;

    private:


        void robotexImage(cv::Mat& grayFrame, double dt);
        void followImage(cv::Mat& grayFrame, double dt);
        void deltaImage(cv::Mat& grayFrame, double dt);

        int filterContour(cv::Mat debugImage, cv::RotatedRect r, double area, float matchedRectArea, float aspect);
        V4 calculateGroundPlane(float dist, float roll, float pitch);
        void camToWorldRotation(Mat3& dst, float roll, float pitch);
        void controlMovementFromLocation(IVec2 curPosS, double dt);
        void controlMovementToTarget(V2 targetPosS, IVec2 curPosS, double dt, float maxSpeed, bool useFgMovement);
        void controlMovementToDirection(V2 targetDirection, IVec2 curPosS, V2 biasPoint, double dt);
        void calculateMovement(cv::Mat& grayFrame, float dt);
        void updateContours(cv::Mat& grayFrame);
        IVec2 findClosestContourCenter(IVec2 imagePoint);
        int findClosestContourCenterIdx(IVec2 imagePoint);
        bool findClosestPointOnTrack(cv::Mat& rgbImage, V2& point, int& edge);
        bool findFurthestConnectedNode(cv::Mat& rgbImage, V2& point, int& retNode, V2 &dir);
        bool findWaypoint(cv::Mat& rgbImage, V2& wp, V2 cpOnTrack);
        void updateTrackEdges(cv::Mat& rgbImage);
        bool calculateOpticalFlow(cv::Mat from, cv::Mat to, cv::Mat debugImage, Mat3& screenToWorld,V4 groundPlane, V2& of, float dt);
        V2 findTargetPoint();
        V2 findTargetDirection();
        void reset();
        void drawTelemetry(cv::Mat& debugImage);
        void setMovementRelativeToEnvironment(V2 movement);

        Localization loc;
        FeatureTracker fTracker;

        // presentation timestamp of last frame (exact time the frame was taken)
        ros::Time lastPTS;

        std::function<void(cv::Mat&, cv::Mat&)> debugImg;

        bool active = false;

        float roll = 0.0f;
        float pitch = 0.0f;
        float distanceToGround = 0.0f;

        V2 horizontalAcc = (V2){0.0f, 0.0f};
        V2 horizontalVelocity = (V2){0.0f, 0.0f};

        // debug
        float lookAhead;


        V2 platformLastSeen;

        float speedRampup = 0.0f;


        // target direction on image
        V2 targetDirection; // default set in reset()
        // closest point to ground in image coordinates
        V2 cpToGroundS; // default set in reset()

        // camera space to screen space
        Mat3 cam2Screen;
        // screen space to camera space ray
        Mat3 screen2Cam;
        // cam2Screen for opencv
        cv::Mat cvIntrinsic;
        // camera space to world space (z axis normal to grund plane)
        Mat3 screenToWorld;
        // screenToWOrld of previous frame
        Mat3 screenToWorldPrev;

        V4 groundPlane;

        // result of cv::findContours is stored here
        std::vector<std::vector<cv::Point>> contours;

        // contours we are interested in, these are pointer-swapped every frame for efficiency
        std::unique_ptr<std::vector<ContourCenter>> contourCenters;
        std::unique_ptr<std::vector<ContourCenter>> prevContourCenters;

        // pairs of track indices pointing to contourCenters vector
        // NB! contains double edges (for now)
        std::vector<int> trackIndices;

        // previous raw camera frame 
        cv::Mat grayFramePrev;
        // thresholded frame
        cv::Mat binaryFrame;

		cv::Mat debugImage;

		std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point lastFrame;
		int totalImagesReceived = 0;

};
