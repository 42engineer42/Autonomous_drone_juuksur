#include <ros/ros.h>
#include <math.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include "HeightController.hpp"
#include "VelocityEstimator.hpp"

// throttle when we turn on motors for takeoff
#define THROTTLE_INITIAL 1000 // estimated hovering value
// maximum throttle value we are allowed to use (for safety)
#define MAX_THROTTLE 1800.0f

#define AUTO_MODE_MANUAL 0
#define AUTO_MODE_FLY 1
#define AUTO_MODE_LAND 2

#define HOVER_HEIGHT 185.0f

#define CLAMP(x, low, high)  (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))

static float pitch;
static float roll;
static float height;
static int32_t ctrlMode;
static bool onPlatform;
static bool clearToLand;

static ros::Time epoch;

#define USE_PLATFORM
#define PLATFORM_HEIGHT 10.0f

void pitchCb(const std_msgs::Float32 msg) {
    pitch = msg.data;
}

void rollCb(const std_msgs::Float32 msg) {
    roll = msg.data;
}

void heightCb(const std_msgs::Float32 msg) {
    height = msg.data;
}

void onPlatformCb(const std_msgs::Bool msg) {
    onPlatform = msg.data;
}

void ctrlModeCb(const std_msgs::Int32 msg) {
    ctrlMode = msg.data;
}

void landCb(const std_msgs::Bool msg) {
    clearToLand = msg.data;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hover");
    ros::NodeHandle n("~");

    epoch = ros::Time::now();
    ros::Rate loop_rate(72);

    ROS_INFO("starting hover_node...\r\n");

    ctrlMode = AUTO_MODE_MANUAL;
    clearToLand = true;

    // latch - true
    ros::Publisher armPub = n.advertise<std_msgs::UInt16>("/control/arm", 1, true);
    ros::Publisher yawPub = n.advertise<std_msgs::UInt16>("/control/yaw", 1, true);
    ros::Publisher pitchPub = n.advertise<std_msgs::UInt16>("/control/pitch", 1, true);
    ros::Publisher rollPub = n.advertise<std_msgs::UInt16>("/control/roll", 1, true);
    ros::Publisher throttlePub = n.advertise<std_msgs::UInt16>("/control/throttle", 1, true);
    ros::Publisher heightPub = n.advertise<std_msgs::Float32>("/control/height", 1, true);
    ros::Publisher modePub = n.advertise<std_msgs::UInt16>("/control/mode", 1, true);

    ros::Subscriber telPitchSub = n.subscribe<std_msgs::Float32>("/telemetry/pitch", 1, pitchCb);
    ros::Subscriber telRollSub = n.subscribe<std_msgs::Float32>("/telemetry/roll", 1, rollCb);
    ros::Subscriber telControlMode = n.subscribe<std_msgs::Int32>("/telemetry/controlmode", 1, ctrlModeCb);
    ros::Subscriber telClearToLand = n.subscribe<std_msgs::Bool>("/telemetry/cleartoland", 1, landCb);
    ros::Subscriber telHeight = n.subscribe<std_msgs::Float32>("/telemetry/height", 1, heightCb);

    // set yaw to 1500 and throttle to 1000
    std_msgs::UInt16 v;
    std_msgs::Float32 f;
    v.data = 1500;
    yawPub.publish(v);
    v.data = 1000;
    throttlePub.publish(v);
    v.data = 0;
    modePub.publish(v);

    while(ros::ok()) {
        ros::spinOnce();

        float curThrottle;
        ros::Time lastTime;
        ros::Time curTime;

        // startup
        if(ctrlMode > AUTO_MODE_MANUAL) {
            v.data = 1000;
            throttlePub.publish(v);
            ROS_INFO("Engaged!\r\n");
            ros::Duration(5).sleep();
            ROS_INFO("Takeoff!\r\n");

            // set to altitude hold mode and set height target
            v.data = 1;
            modePub.publish(v);

            if(ctrlMode == AUTO_MODE_FLY) {
                f.data = HOVER_HEIGHT;
                heightPub.publish(f);
            } else if(ctrlMode == AUTO_MODE_LAND){
                f.data = 0.0f;
                heightPub.publish(f);
            }

            curThrottle = THROTTLE_INITIAL;
            curTime = lastTime = ros::Time::now();

            v.data = 1500;
            yawPub.publish(v);
            v.data = 1000;
            throttlePub.publish(v);
        }

        // hover loop
        float landHeight = HOVER_HEIGHT;
        float flyHeight = 0.0f;
        while(ros::ok() && ctrlMode > AUTO_MODE_MANUAL) {
            ros::spinOnce();

            curTime = ros::Time::now();
            auto dt = (curTime - lastTime).toSec();

            if(ctrlMode == AUTO_MODE_LAND && clearToLand){
                flyHeight = landHeight;
                if(landHeight > 0.0f) {
                    landHeight -= dt * 30.0f;
                } else {
                    landHeight = 0.0f;
                }
                f.data = landHeight;
                heightPub.publish(f);
                if(height < 10.0f) {
                    v.data = 0;
                    modePub.publish(v);
                }
            } else {
                /*if(flyHeight < HOVER_HEIGHT) {
                    flyHeight += dt * 60.0f;
                } else {
                    flyHeight = HOVER_HEIGHT;
                }*/
                f.data = HOVER_HEIGHT;
                heightPub.publish(f);
                landHeight = HOVER_HEIGHT;

                v.data = 1;
                modePub.publish(v);
            }
            
            double timestamp = (curTime - epoch).toSec();

            v.data = 1000;
            throttlePub.publish(v);
            lastTime = curTime;

            loop_rate.sleep();
        }

        // must be manual if we ever get here
        v.data = 0;
        modePub.publish(v);
        f.data = 0.0f;
        heightPub.publish(f);

    	curTime = ros::Time::now();

        loop_rate.sleep();
    }
}
