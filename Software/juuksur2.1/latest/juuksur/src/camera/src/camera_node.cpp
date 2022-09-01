#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>

namespace
{
    const int DEFAULT_FRAME_WIDTH = 800;
    const int DEFAULT_FRAME_HEIGHT = 640;
    const int DEFAULT_FRAME_RATE = 90;
}

std::string intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

int main(int argc, char **argv)
{
    //set up node
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    image_transport::Publisher image_pub = it.advertise("camera/image_raw", 0);

    ros::Publisher debug_publisher = node.advertise<std_msgs::String>("camera/debug", 1, true);

    //set up camera
    cv::VideoCapture camera(-1);

    //reading params from /launchers/params/cameraParams.yml
    //int C_FRAME_WIDTH;
    //int C_FRAME_HEIGHT;
    int C_FRAME_RATE;
    /*
    bool c1 = node.getParam("camera/width", C_FRAME_WIDTH);
    bool c2 = node.getParam("camera/height", C_FRAME_HEIGHT);

    if (!(c1 && c2))
    {
        ROS_WARN("CAMERA PARAMS - not found, loading default value");
        ROS_WARN("CAMERA PARAMS - not found, loading default value");

        C_FRAME_WIDTH = DEFAULT_FRAME_WIDTH;
        C_FRAME_HEIGHT = DEFAULT_FRAME_HEIGHT;
    }
    */
    bool c3 = node.getParam("global/rate", C_FRAME_RATE);

    if (!c3)
    {
        ROS_WARN("GLOBAL PARAMS - not found, loading default value");
        ROS_WARN("GLOBAL PARAMS - not found, loading default value");

        C_FRAME_RATE = DEFAULT_FRAME_RATE;
    }

    //set parameters
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 410);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 308);
    camera.set(CV_CAP_PROP_FPS, 6);
    camera.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M','J','P','G'));

    
    //init values
    cv::Mat frame;
    sensor_msgs::ImagePtr imgMsg;

    while(!camera.isOpened())
    {
    	std_msgs::String debug_msg;
        debug_msg.data = "Camera not open yet!";
        debug_publisher.publish(debug_msg);
    }

    camera.read(frame);

    std::cout << "[camera_node] RESOLUTION: " << frame.size().width << "x" << frame.size().height << std::endl;
    //std::cout << "[camera_node] C_FRAME_RATE: " << C_FRAME_RATE << std::endl;
    //std::cout << "[camera_node] backend: " << camera.getBackendName() << "\n";
    std::cout << "[camera_node] [GO] " << std::endl;
    std::cout << "[camera_node] [GO] " << std::endl;
    std::cout << "[camera_node] [GO] " << std::endl;

    //ROS fps
    ros::Rate loop_rate(200);

    //main loop
    while (ros::ok())
    {
        if (camera.read(frame) == false)
        {
            throw std::runtime_error("[ERROR] Lost camera");
        }
	/*imshow("frame", frame);
	cv::waitKey(1);
	printf("frame\r\n");*/

        imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub.publish(imgMsg);
	printf("frame\r\n");

        loop_rate.sleep();
    }

    return 0;
}
