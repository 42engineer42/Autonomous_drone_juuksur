#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/String.h>
#include <dev_msgs/CameraFrame.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <fcntl.h>

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

bool connectLoop(int* sockFd) {
    struct sockaddr_in camAddr;
    inet_pton(AF_INET, "127.0.0.1", &(camAddr.sin_addr));
    if((*sockFd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        perror("socket");
        exit(1);
    }
    camAddr.sin_family = AF_INET;
    camAddr.sin_port = htons(7006);
    bzero(&(camAddr.sin_zero), 8);
    while(connect(*sockFd, (struct sockaddr *)&camAddr, sizeof(struct sockaddr)) < 0 && ros::ok()){ 
        ros::spinOnce();
        fprintf(stderr, "failed to connect simulator camera, error: %s, retrying in 1s\r\n", strerror(errno));
        usleep(1000000);
    }
    int i = 1;
    setsockopt(*sockFd, IPPROTO_TCP, O_NDELAY, (void *)&i, sizeof(i));
    return ros::ok();
}

#define IMG_WIDTH 410
#define IMG_HEIGHT 308

int main(int argc, char **argv)
{
    //set up node
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle node;
    image_transport::ImageTransport it(node);
    image_transport::Publisher image_pub = it.advertise("camera/image_raw", 1);
    ros::Publisher image_pub2 = node.advertise<dev_msgs::CameraFrame>("camera/image_raw2", 1);
    //compressed_image_transport::CompressedPublisher cimage_pub = it.advertise("camera/image_raw/compressed", 4);

    //init values
    cv::Mat frame;
    sensor_msgs::ImagePtr imgMsg;

    int sockFd = -1;

    bool res = connectLoop(&sockFd);
    if(res) {
        printf("connected to simulator camera!\r\n");
    }

    //main loop
    uint8_t recvBuf[4096];
    int imageSize = 3*IMG_WIDTH*IMG_HEIGHT+64;
    uint8_t *img = NULL;
    int bytesLeft = imageSize;

    int width = 1;
    int height = 1;

    int imagesReceived = 0;
    uint64_t epoch = 0;
    uint64_t bytesReceived = 0;

    if(ros::ok()) {
        uint8_t buf[8];
        ssize_t r = recv(sockFd, buf, sizeof(buf), 0);
        if(r == 0 || r < 0) {
            exit(-1);
        }
        assert(r == 8);
        // BIG ENDIAN NOT WELCOME
        memcpy(&width, buf, 4);
        memcpy(&height, buf+4, 4);
        printf("Simulator camera resolution %dx%d\r\n", width, height);
        imageSize = 3*width*height+8;
        img = (uint8_t*)malloc(imageSize);
        bytesLeft = imageSize;
    }
    struct timeval tv;

    while (ros::ok())
    {
        ros::spinOnce();
        ssize_t r = recv(sockFd, recvBuf, sizeof(recvBuf), 0);
        if(r == 0) {
            close(sockFd);
            sockFd = -1;
            printf("Simulator camera shut down by server, attempting reconnect in 1s\r\n");
            usleep(1000000); // wait 1s before attempting reconnect
            res = connectLoop(&sockFd);
            if(res) {
                printf("connected to simulator camera!\r\n");
            } else {
                break;
            }
        }
        uint8_t *rp = (uint8_t*)recvBuf;
        while(r > 0) {
            bytesReceived += r;
            // amount of bytes received for current frame
            int copySize = bytesLeft < r ? bytesLeft : r;
            memcpy(img+(imageSize-bytesLeft), rp, copySize);
            bytesLeft -= copySize;
            r -= copySize;
            rp += copySize;
            if(bytesLeft == 0) {
                // BIG ENDIAN NOT WELCOME
                frame = cv::Mat(height, width, CV_8UC3, (void*)(img));
                uint64_t *ts = (uint64_t*)(img+(3*width*height));
                gettimeofday(&tv, NULL);
                uint64_t millisecondsSinceEpoch =
                    (unsigned long long)(tv.tv_sec) * 1000 +
                    (unsigned long long)(tv.tv_usec) / 1000;
                if(imagesReceived == 0) {
                    epoch = millisecondsSinceEpoch;
                }
                int difms = millisecondsSinceEpoch - *ts;
                ros::Time pts = ros::Time::now() + ros::Duration(0, 1000*1000*difms);
                //printf("image ts %llu current ts %llu\r\n", *ts, millisecondsSinceEpoch);
                auto img1 = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame); 
                imgMsg = img1.toImageMsg();
                image_pub.publish(imgMsg);
                bytesLeft = imageSize;

                dev_msgs::CameraFrame cf;
                cf.image = *imgMsg;
                cf.presentationTimestamp = pts;
                image_pub2.publish(cf);

                imagesReceived ++;
                float fps = 1.0f/(((millisecondsSinceEpoch - epoch)/1000.0f)/(float)imagesReceived);
                //printf("simulator camera fps %f received %luB time %f\r\n", fps, bytesReceived, (millisecondsSinceEpoch - epoch)/999.0f);
            }
        }
    }
    if(img != NULL) {
        free((void*)img);
        img = NULL;
    }
    if(sockFd != -1) {
        close(sockFd);
    }

    return 0;
}
