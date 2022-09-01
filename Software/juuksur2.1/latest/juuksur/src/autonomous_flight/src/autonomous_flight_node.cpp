
#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "autonomous_controller.hpp"

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

    ros::Subscriber imageSub = n.subscribe("/camera/image_raw", 1, &AutonomousController::imageCallback, &controller);
    ros::Subscriber pitchSub = n.subscribe("/telemetry/pitch", 1, &AutonomousController::pitchCallback, &controller);
    ros::Subscriber rollSub = n.subscribe("/telemetry/roll", 1, &AutonomousController::rollCallback, &controller);
    ros::Subscriber rangeSub = n.subscribe("/telemetry/range", 1, &AutonomousController::rangeCallback, &controller);
    ros::Subscriber controlThrottle = n.subscribe("/control/throttle", 1, &AutonomousController::throttleCallback, &controller);

    boost::function<void (const std_msgs::Int32)> ctrlModeCb = [&controller](const std_msgs::Int32 msg) -> void
    {
        // mode 1 is autonomous flight
        if(msg.data == 1 && !controller.isActive()) {
            controller.start();
        } else if(msg.data != 1 && controller.isActive()) {
            controller.stop();
        }
    };
    ros::Subscriber telControlMode = n.subscribe<std_msgs::Int32>("/telemetry/controlmode", 1, ctrlModeCb);

    ros::Publisher rollPub = n.advertise<std_msgs::UInt16>("/control/roll", 1, true);
    ros::Publisher pitchPub = n.advertise<std_msgs::UInt16>("/control/pitch", 1, true);

    uint16_t rollLast = controller.controlRoll;
    uint16_t pitchLast = controller.controlPitch;

    //controller.start();

//	cv::Mat image;
//  cv_bridge::CvImage cvImg(std_msgs::Header(), "bgr8", image);

    while(ros::ok()) {
        ros::spinOnce();

        uint16_t roll = controller.controlRoll;
        uint16_t pitch = controller.controlPitch;
	
	/*uint16_t roll = 1500;
	uint16_t pitch = 1500;*/

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

        rollLast = roll;
        pitchLast = pitch;

        loopRate.sleep();
    }

    return 0;
}
