#include <ros/ros.h>
#include <math.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include "HeightController.hpp"
#include "VelocityEstimator.hpp"

// throttle when we turn on motors for takeoff
#define THROTTLE_INITIAL 1000 // estimated hovering value
// maximum throttle value we are allowed to use (for safety)
#define MAX_THROTTLE 1600.0f

#define AUTO_MODE_MANUAL 0
#define AUTO_MODE_FLY 1
#define AUTO_MODE_LAND 2

#define HOVER_HEIGHT 176.0f

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

static float pitch;
static float roll;
static int32_t ctrlMode;

static ros::Time epoch;
static HeightController* controller;
static VelocityEstimator* vEst;

void pitchCb(const std_msgs::Float32 msg) {
    pitch = msg.data;
}

void rollCb(const std_msgs::Float32 msg) {
    roll = msg.data;
}

void rangeCb(const std_msgs::Float32 msg) {
    auto heightTime = ros::Time::now();
    auto dif = heightTime - epoch;
    controller->heightUpdate((float)msg.data, dif.toSec());
    vEst->updateIntegral((float)msg.data, dif.toSec());
}

void accZCb(const std_msgs::Float32 msg) {
    auto now = ros::Time::now();
    auto dif = now - epoch;
    controller->accelerationUpdate(msg.data*100.0f, dif.toSec());
    vEst->updateDerivative(msg.data*100.0f-981.0f, dif.toSec());
}

void ctrlModeCb(const std_msgs::Int32 msg) {
    ctrlMode = msg.data;
}

int main(int argc, char **argv) {
    // TODO: this should not be 0.3, but PID was tuned for this..
    vEst = new VelocityEstimator(0.0, 0.30f); // use data from last x s
    //controller = new TOHeightController();
    //controller = new LQRHeightController();
    //controller = new PIDHeightController();
    controller = new CPIDHeightController();

    ros::init(argc, argv, "hover");
    ros::NodeHandle n("~");

    epoch = ros::Time::now();
    ros::Rate loop_rate(72);

    ROS_INFO("starting hover_node...\r\n");

    ctrlMode = AUTO_MODE_MANUAL;

    // latch - true
    ros::Publisher armPub = n.advertise<std_msgs::UInt16>("/control/arm", 1, true);
    ros::Publisher yawPub = n.advertise<std_msgs::UInt16>("/control/yaw", 1, true);
    ros::Publisher pitchPub = n.advertise<std_msgs::UInt16>("/control/pitch", 1, true);
    ros::Publisher rollPub = n.advertise<std_msgs::UInt16>("/control/roll", 1, true);
    ros::Publisher throttlePub = n.advertise<std_msgs::UInt16>("/control/throttle", 1, true);

    ros::Publisher velocityPub = n.advertise<std_msgs::Float32>("/telemetry/VelZ", 1, true);

    ros::Subscriber telPitchSub = n.subscribe<std_msgs::Float32>("/telemetry/pitch", 1, pitchCb);
    ros::Subscriber telRollSub = n.subscribe<std_msgs::Float32>("/telemetry/roll", 1, rollCb);
    ros::Subscriber telRangeSub = n.subscribe<std_msgs::Float32>("/telemetry/range", 1, rangeCb);
    ros::Subscriber telAccZSub = n.subscribe<std_msgs::Float32>("/telemetry/accZ", 1, accZCb);
    ros::Subscriber telControlMode = n.subscribe<std_msgs::Int32>("/telemetry/controlmode", 1, ctrlModeCb);

    std_msgs::UInt16 v;
    v.data = 1000;
    armPub.publish(v);
    v.data = 1500;
    yawPub.publish(v);
    v.data = 0;
    throttlePub.publish(v);

    while(ros::ok()) {
        ros::spinOnce();

        float curThrottle;
        ros::Time lastTime;
        ros::Time curTime;
        if(ctrlMode > AUTO_MODE_MANUAL) {
            // startup sequence (must arm at 0 throttle)
            v.data = 1000;
            throttlePub.publish(v);
            armPub.publish(v);
            ros::Duration(5.5).sleep(); // wait
            v.data = 2000;
            armPub.publish(v);
            ROS_INFO("Armed!\r\n");
            ros::Duration(5.5).sleep(); // wait
            ROS_INFO("Takeoff!\r\n");

            if(ctrlMode == AUTO_MODE_FLY) {
                controller->setTarget(HOVER_HEIGHT);
            } else if(ctrlMode == AUTO_MODE_LAND){
                controller->setTarget(HOVER_HEIGHT);
            }
            controller->reset();
            curThrottle = THROTTLE_INITIAL;
            curTime = lastTime = ros::Time::now();
            v.data = 1500;
            yawPub.publish(v);
            //rollPub.publish(v);
            //pitchPub.publish(v);
            v.data = 1000;
            throttlePub.publish(v);
        }

        while(ros::ok() && ctrlMode > AUTO_MODE_MANUAL) {
            ros::spinOnce();

            curTime = ros::Time::now();
            auto dt = (curTime - lastTime).toSec();

            if(ctrlMode == AUTO_MODE_FLY) { // autonomous mode
                controller->setTarget(HOVER_HEIGHT);
            } else if(ctrlMode == AUTO_MODE_LAND) { // landing mode
                controller->setTarget(0.0f);
            }
            
            double timestamp = (curTime - epoch).toSec();
            vEst->update(timestamp);
            float velocity = vEst->estimate(timestamp);
            std_msgs::Float32 velD;
            velD.data = velocity;
            velocityPub.publish(velD);
            controller->velocityUpdate(velocity, timestamp);

            controller->update(timestamp);
            float ctrlVal = controller->getThrottle();


            curThrottle = CLAMP((uint16_t)(1000.0f*ctrlVal) + (uint16_t)1000, 1000, MAX_THROTTLE);
            
            v.data = (uint16_t)curThrottle;
            throttlePub.publish(v);
            lastTime = curTime;

            loop_rate.sleep();
        }

    	curTime = ros::Time::now();
    	double timestamp = (curTime - epoch).toSec();
    	float velocity = vEst->estimate(timestamp);
    	std_msgs::Float32 velD;
    	velD.data = velocity;
    	velocityPub.publish(velD);

        loop_rate.sleep();
    }
    delete controller;
    delete vEst;
}
