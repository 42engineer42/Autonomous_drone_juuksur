
#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "autonomous_controller.hpp"

#define AUTO_MODE_MANUAL 0
#define AUTO_MODE_FLY 1
#define AUTO_MODE_LAND 2

static ros::Subscriber imageSub, pitchSub, rollSub, rangeSub, controlThrottle;
static ros::Subscriber velXSub, velYSub, accXSub, accYSub;
static ros::Publisher rollPub, pitchPub, ofXPub, ofYPub, velXPub, velYPub, testXPub, testYPub;
static ros::Publisher landPub;
uint16_t rollLast, pitchLast;
float velXLast, velYLast;
AutonomousController *actrl;


void publishCallback(const ros::TimerEvent& e) {
    uint16_t roll = actrl->controlRoll;
    uint16_t pitch = actrl->controlPitch;

    if(roll != rollLast) {
        std_msgs::UInt16 data;
        data.data = roll;
        rollPub.publish(std_msgs::UInt16(data));
    }
    if(pitch != pitchLast) {
        std_msgs::UInt16 data;
        data.data = pitch;
        pitchPub.publish(data);
    }

    if(actrl->movementRelEDirty) {
        std_msgs::Float32 data;
        data.data = actrl->movementRelativeToEnvironment.x;
        ofXPub.publish(data);
        data.data = actrl->movementRelativeToEnvironment.y;
        ofYPub.publish(data);
    }
    actrl->movementRelEDirty = false;

    if(actrl->testDirty) {
        std_msgs::Float32 data;
        data.data = actrl->testV.x;
        testXPub.publish(data);
        data.data = actrl->testV.x;
        testYPub.publish(data);
    }

    if(true || velXLast != actrl->controlVelocity.x) {
        std_msgs::Float32 data;
        data.data = actrl->controlVelocity.x;
        velXPub.publish(data);
    }
    if(true || velYLast != actrl->controlVelocity.y) {
        std_msgs::Float32 data;
        data.data = actrl->controlVelocity.y;
        velYPub.publish(data);
    }

    {
    std_msgs::Bool data;
    data.data = actrl->clearToLand;
    landPub.publish(data);
    }

    std_msgs::Bool hpdata;
    hpdata.data = actrl->hoveringPlatform;

    rollLast = roll;
    pitchLast = pitch;

    velXLast = actrl->controlVelocity.x;
    velYLast = actrl->controlVelocity.y;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "autonomous_flight");
    ros::NodeHandle n;

    ros::Rate loopRate(100);

    sensor_msgs::ImagePtr imgMsg;
    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub = it.advertise("camera/image_flight", 2);

    sensor_msgs::ImagePtr pfImgMsg;
    image_transport::Publisher pf_image_pub = it.advertise("localization/particles", 1);

    auto processDebugImg = [&n, &image_pub, &imgMsg, &pf_image_pub](cv::Mat &debugImg, cv::Mat &pfImg) -> void
    {
        imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debugImg).toImageMsg();		
		image_pub.publish(imgMsg);
        imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pfImg).toImageMsg();
        pf_image_pub.publish(imgMsg);
    }; 

    AutonomousController controller(processDebugImg);
    actrl = &controller;

    imageSub = n.subscribe("/camera/image_raw2", 1, &AutonomousController::imageCallback, &controller);
    pitchSub = n.subscribe("/telemetry/pitch", 1, &AutonomousController::pitchCallback, &controller);
    rollSub = n.subscribe("/telemetry/roll", 1, &AutonomousController::rollCallback, &controller);
    rangeSub = n.subscribe("/telemetry/height", 1, &AutonomousController::rangeCallback, &controller);

    velXSub = n.subscribe("/telemetry/velX", 1, &AutonomousController::velXCallback, &controller);
    velYSub = n.subscribe("/telemetry/velY", 1, &AutonomousController::velYCallback, &controller);

    accXSub = n.subscribe("/telemetry/accX", 1, &AutonomousController::accXCallback, &controller);
    accYSub = n.subscribe("/telemetry/accY", 1, &AutonomousController::accYCallback, &controller);

    boost::function<void (const std_msgs::Int32)> ctrlModeCb = [&controller](const std_msgs::Int32 msg) -> void
    {
        // mode 1 is autonomous flight
        if(msg.data == AUTO_MODE_FLY && !controller.isActive()) {
            controller.start();
        } else if(msg.data != AUTO_MODE_FLY && controller.isActive()) {
            controller.stop();
        }
        if(msg.data == AUTO_MODE_LAND) {
            controller.landing = true;
        } else {
            controller.landing = false;
        }
    };

    ros::Subscriber telControlMode = n.subscribe<std_msgs::Int32>("/telemetry/controlmode", 1, ctrlModeCb);

    rollPub = n.advertise<std_msgs::UInt16>("/control/roll", 1, true);
    pitchPub = n.advertise<std_msgs::UInt16>("/control/pitch", 1, true);

    ofXPub = n.advertise<std_msgs::Float32>("/telemetry/rawOpticalFlowX", 1, true);
    ofYPub = n.advertise<std_msgs::Float32>("/telemetry/rawOpticalFlowY", 1, true);

    velXPub = n.advertise<std_msgs::Float32>("/control/velX", 1, true);
    velYPub = n.advertise<std_msgs::Float32>("/control/velY", 1, true);

    testXPub = n.advertise<std_msgs::Float32>("/telemetry/testX", 1, true);
    testYPub = n.advertise<std_msgs::Float32>("/telemetry/testY", 1, true);

    landPub = n.advertise<std_msgs::Bool>("/telemetry/cleartoland", 1, true);

    rollLast = controller.controlRoll;
    pitchLast = controller.controlPitch;

    velXLast = controller.controlVelocity.x;
    velYLast = controller.controlVelocity.y;

    ros::Timer publishTimer = n.createTimer(ros::Duration(0.01), publishCallback);

    ros::spin();

    return 0;
}
