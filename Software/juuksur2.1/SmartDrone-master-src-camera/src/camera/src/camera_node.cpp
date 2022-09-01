#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

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

    //set up camera
    cv::VideoCapture camera(0);

    //reading params from /launchers/params/cameraParams.yml
    int C_FRAME_WIDTH;
    int C_FRAME_HEIGHT;
    int C_FRAME_RATE;

    bool c1 = node.getParam("camera/width", C_FRAME_WIDTH);
    bool c2 = node.getParam("camera/height", C_FRAME_HEIGHT);

    if (!(c1 && c2))
    {
        ROS_WARN("CAMERA PARAMS - not found, loading default value");
        ROS_WARN("CAMERA PARAMS - not found, loading default value");

        C_FRAME_WIDTH = DEFAULT_FRAME_WIDTH;
        C_FRAME_HEIGHT = DEFAULT_FRAME_HEIGHT;
    }

    bool c3 = node.getParam("global/rate", C_FRAME_RATE);

    if (!c3)
    {
        ROS_WARN("GLOBAL PARAMS - not found, loading default value");
        ROS_WARN("GLOBAL PARAMS - not found, loading default value");

        C_FRAME_RATE = DEFAULT_FRAME_RATE;
    }

    //set parameters
    camera.set(CV_CAP_PROP_FRAME_WIDTH, C_FRAME_WIDTH);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, C_FRAME_HEIGHT);
    camera.set(CV_CAP_PROP_FPS, C_FRAME_RATE);

    //init values
    cv::Mat frame;
    sensor_msgs::ImagePtr imgMsg;

    if(camera.isOpened() == false)
    {
        throw std::runtime_error("[ERROR] Camera not found");
    }

    if (camera.read(frame) == false)
    {
        throw std::runtime_error("[ERROR] First grap failed");
    }

    std::cout << "[camera_node] RESOLUTION: " << frame.size().width << "x" << frame.size().height << std::endl;
    std::cout << "[camera_node] C_FRAME_RATE: " << C_FRAME_RATE << std::endl;
    std::cout << "[camera_node] [GO] " << std::endl;
    std::cout << "[camera_node] [GO] " << std::endl;
    std::cout << "[camera_node] [GO] " << std::endl;

    //ROS fps
    ros::Rate loop_rate(C_FRAME_RATE);

    //main loop
    while (ros::ok())
    {
        if (camera.read(frame) == false)
        {
            throw std::runtime_error("[ERROR] Lost camera");
        }

        imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub.publish(imgMsg);

        loop_rate.sleep();
    }

    return 0;
}
