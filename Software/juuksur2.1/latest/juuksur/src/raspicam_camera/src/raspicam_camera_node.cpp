#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "raspicam_cv.h"
#include "raspicam.h"

#define IMG_WIDTH  1640
#define IMG_HEIGHT 1232

int main(int argc, char **argv)
{
	ros::init(argc, argv, "raspicam_camera_node");	
	ros::NodeHandle node;
	image_transport::ImageTransport it(node);
	image_transport::Publisher image_pub = it.advertise("camera/image_raw", 2);

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
	cam.setFrameRate(20);
	printf("Opening camera...\r\n");
	if(!cam.open()) {
		fprintf(stderr, "Failed to open camera!\r\n");
		return -1;
	}
	printf("Camera opened!\r\n");

	int expectedSize = IMG_WIDTH*IMG_HEIGHT;
	unsigned char *buffer = new unsigned char[expectedSize];

	printf("raspicam_camera started...\r\n");
	
	//ros::Rate loop_rate(200);
	while(ros::ok())
	{
		ros::spinOnce();
		/*cam.grab();
		cam.retrieve(image);
		cv::resize(image, resizedImage, cv::Size(), 0.25, 0.25, cv::INTER_NEAREST); */

		/*for(int j = 0; j < 922; j+=4) {
			for(int i = 0; i < 1640; i+=4) {
				
			}
		}*/
		cam.grab();
		if(expectedSize == cam.getImageBufferSize()) {
			cam.retrieve(buffer);
			cv::Mat image(cv::Size(IMG_WIDTH, IMG_HEIGHT), CV_8UC1, buffer);
			cv::resize(image, resizedImage, cv::Size(), 0.125, 0.125, cv::INTER_NEAREST);
			imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", resizedImage).toImageMsg();
			image_pub.publish(imgMsg);
		}

		/*imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", resizedImage).toImageMsg();
		image_pub.publish(imgMsg);*/
		//loop_rate.sleep();
	}	
	return 0;
}
