#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dev_msgs/CameraFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "raspicam_cv.h"
#include "raspicam.h"

// camera v2
//#define IMG_WIDTH  1640
//#define IMG_HEIGHT 1232

// camera v1
#define IMG_WIDTH 648
#define IMG_HEIGHT 486

int main(int argc, char **argv)
{
	ros::init(argc, argv, "raspicam_camera_node");	
	ros::NodeHandle node;
	image_transport::ImageTransport it(node);
	image_transport::Publisher image_pub = it.advertise("camera/image_raw", 1, true);
    ros::Publisher image_pub2 = node.advertise<dev_msgs::CameraFrame>("camera/image_raw2", 1, true);

    sensor_msgs::ImagePtr imgMsg;
	cv::Mat image;
	cv::Mat resizedImage;
	/*raspicam::RaspiCam_Cv cam;
	cam.set(CV_CAP_PROP_FORMAT, CV_8UC1);
	cam.set(CV_CAP_PROP_FRAME_WIDTH, 1640);
	cam.set(CV_CAP_PROP_FRAME_HEIGHT, 1232);
	if(!cam.open()) {
		fprintf(stderr, "Failed to open camera!\r\n");
		return -1;
	}*/

	raspicam::RaspiCam cam;
	cam.setAWB(raspicam::RASPICAM_AWB_AUTO);
	cam.setWidth(IMG_WIDTH);
	cam.setHeight(IMG_HEIGHT);
	cam.setFormat(raspicam::RASPICAM_FORMAT_GRAY);
	cam.setFrameRate(40);
	printf("Opening camera...\r\n");
	if(!cam.open()) {
		fprintf(stderr, "Failed to open camera!\r\n");
		return -1;
	}
	printf("Camera opened!\r\n");

	int expectedSize = IMG_WIDTH*IMG_HEIGHT;
	unsigned char *buffer = new unsigned char[expectedSize];

	printf("raspicam_camera started...\r\n");

    // presentation timestamp in microseconds (should be time since last frame capture?)
    int64_t pts = 0;

    dev_msgs::CameraFrame cf;
	
	//ros::Rate loop_rate(200);
	while(ros::ok())
	{
		ros::spinOnce();
		/*cam.grab();
		cam.retrieve(image);
		cv::resize(image, resizedImage, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST); */

		cam.grab();
		if(expectedSize == cam.getImageBufferSize()) {
            // pts is timestamp in microseconds (gpu time)
			cam.retrieve(buffer, &pts);
            //printf("pts %lld\r\n", pts);
			cv::Mat image(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC1, buffer);
			cv::resize(image, resizedImage, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
			imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", resizedImage).toImageMsg();
			image_pub.publish(imgMsg);

            cf.image = *imgMsg;
            // TODO: use pts value
            uint64_t stime;
            auto status = cam.getSystemTime(stime);
            if(!status) {
                cf.presentationTimestamp = ros::Time::now();
            } else {
                int64_t usec = stime - pts;
                cf.presentationTimestamp = ros::Time::now() - ros::Duration(0, 1000*usec);
                //printf("raspicam_camera_node: frame is %lld (cur %llu frame %llu) us old\r\n", usec, stime, pts);
            }
            image_pub2.publish(cf);
		}

		/*imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", resizedImage).toImageMsg();
		image_pub.publish(imgMsg);*/
		//loop_rate.sleep();
	}	
	return 0;
}
