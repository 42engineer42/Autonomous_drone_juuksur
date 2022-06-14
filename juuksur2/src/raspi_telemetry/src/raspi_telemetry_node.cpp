#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <sys/sysinfo.h>
#include <linux/limits.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/ioctl.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <unordered_map>
#include <chrono>

#include "encoder.h"

typedef struct CpuStats {
    long long user;
    long long nice;
    long long system;
    long long idle;

    long totalTime;
} CpuStats;

typedef struct DiskProperties {
    char deviceName[PATH_MAX];

    long sectorSize;
} DiskProperties;

typedef struct DiskStats {
    char deviceName[PATH_MAX];

    long long readsCompleted;
    long long readsMerged; // adjacent reads might be merged into one op
    long long sectorsRead;
    long long msSpentReading; // milliseconds spent reading
    long long writesCompleted;
    long long writesMerged;
    long long sectorsWritten;
    long long msSpentWriting;
    long long ioInProgress; // nr of I/O in progress
    long long msSpentIO;
    long long wmsSpentIO;

    std::chrono::steady_clock::time_point ts;
} DiskStats;

#define CPU_STAT_AVG_COUNT 10

static long long cpustatIdx;
static CpuStats cpustat[CPU_STAT_AVG_COUNT];

float utilization;
float memUtilization;

std::unordered_map<std::string, DiskStats> prevDiskStats;
std::unordered_map<std::string, DiskProperties> diskProperties;
std::unordered_map<std::string, ros::Publisher> diskPubs;
std::unordered_map<std::string, ros::Publisher> diskWPubs;

ros::Publisher cpuPub, memPub;

static RPIEncoder encoder;


static cv::Mat combinedImg;

void timerCallback(const ros::TimerEvent& e) {
    FILE *procStat = NULL;
    struct sysinfo info;
    CpuStats *cs = cpustat + (cpustatIdx % CPU_STAT_AVG_COUNT);
    procStat = fopen("/proc/stat", "r");
    int read = fscanf(procStat, "%*s %lld %lld %lld %lld", &cs->user, &cs->nice, &cs->system, &cs->idle);
    fclose(procStat);
    cpustatIdx++;

    if(cpustatIdx >= 10) {
        CpuStats *csprev = cpustat + ((cpustatIdx-9) % CPU_STAT_AVG_COUNT);
        cs->totalTime = cs->user+cs->nice+cs->system+cs->idle;
        long long idledif = cs->idle - csprev->idle;
        long long totaldif = cs->totalTime - csprev->totalTime;
        utilization = 1.0 - (double)idledif/(double)totaldif;
    }

    if(sysinfo(&info) >= 0) {
        memUtilization = 1.0 - (double)info.freeram/(double)info.totalram;
    }

    std_msgs::Float32 f;
    f.data = utilization;
    cpuPub.publish(f);
    f.data = memUtilization;
    memPub.publish(f);
}

void diskStatTimerCallback(/*const ros::TimerEvent& e, */ros::NodeHandle &n) {
    DiskStats stats;
    FILE *diskStat = NULL;
    diskStat = fopen("/proc/diskstats", "r");
    if(diskStat == NULL) {
	    return;
    }
    int fieldsRead = 0;
    do { 
        fieldsRead = fscanf(diskStat, "%*u %*u %s %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %llu %*u %*u %*u %*u", stats.deviceName, &stats.readsCompleted, &stats.readsMerged, &stats.sectorsRead, &stats.msSpentReading, &stats.writesCompleted, &stats.writesMerged, &stats.sectorsWritten, &stats.msSpentWriting, &stats.ioInProgress, &stats.msSpentIO, &stats.wmsSpentIO); 
        stats.ts = std::chrono::steady_clock::now();
        if(fieldsRead >= 12) {
            std::string diskName(stats.deviceName);
	    if(diskName.find("ram") != std::string::npos || diskName.find("loop") != std::string::npos) {
		    continue;
	    }
            std::string diskDevice = "/dev/" + diskName;
            auto props = diskProperties.find(diskName);
            if(props == diskProperties.end()) {
                int fd = open(diskDevice.c_str(), O_RDONLY | O_NONBLOCK);
                int sectorSize;
                if (ioctl(fd, BLKSSZGET, &sectorSize) < 0)
                {
                    sectorSize = 512;
                    ROS_INFO("Failed to determine sector size for %s errno %d, assuming 512B\r\n", diskDevice.c_str(), errno);
                }
                DiskProperties dprops;
                strcpy(dprops.deviceName, diskName.c_str());
                dprops.sectorSize = sectorSize;
                diskProperties[diskName] = dprops;
                std::string topic = "/system/disk/" + diskName + "/read";
                ros::Publisher pub = n.advertise<std_msgs::Int32>(topic.c_str(), 1, true);
                diskPubs[diskName] = pub;
                topic = "/system/disk/" + diskName + "/write";
                pub = n.advertise<std_msgs::Int32>(topic.c_str(), 1, true);
                diskWPubs[diskName] = pub;
            }
            DiskProperties diskprops = diskProperties[diskName];
            ros::Publisher pub = diskPubs[diskName];
            ros::Publisher pub2 = diskWPubs[diskName];
            auto foundstats = prevDiskStats.find(diskName);
            if(foundstats != prevDiskStats.end()) {
                DiskStats prev = prevDiskStats[diskName];

                auto now = std::chrono::steady_clock::now();
                std::chrono::duration<double> dif = now - prev.ts;
                double difs = dif.count();

                double sectorPerSecond = (stats.sectorsRead - prev.sectorsRead) / difs;
                double sectorWPerSecond = (stats.sectorsWritten - prev.sectorsWritten) / difs;

                std_msgs::Int32 f;
                f.data = (int)sectorPerSecond * diskprops.sectorSize;
                pub.publish(f);
                f.data = (int)sectorWPerSecond * diskprops.sectorSize;
                pub2.publish(f);
            }
            prevDiskStats[diskName] = stats;
        }
    } while(fieldsRead >= 12);
    fclose(diskStat);
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    static bool skip;
    // pi camera 1 image is 324x243
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cvPtr->image;
    frame.copyTo(combinedImg(cv::Rect(0, 0, 324, 243)));
    
    int step = combinedImg.step;
    pushFrame(&encoder, 450, 336, step, (char*)combinedImg.data);

    skip = !skip;
}

void particleCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat frame = cvPtr->image;
    frame.copyTo(combinedImg(cv::Rect(0, 243, 250, 125)));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "raspi_telemetry");
    ros::NodeHandle n("~");

    combinedImg = cv::Mat::zeros(450, 336, CV_8UC3);

    char buf[256];
    char filenameBuf[64];
    strcpy(buf, "/home/pi/.ros/");
    time_t filetime; 
    struct tm *tmp;
    time(&filetime);
    tmp = localtime(&filetime);
    strftime(filenameBuf, sizeof(filenameBuf), "%d-%m-%y_%H-%M-%S", tmp);
    strcat(buf, filenameBuf);
    strcat(buf, ".264");

    //int res = initEncode(&encoder, "/home/pi/.ros/video.264");
    int res = initEncode(&encoder, buf, "/home/pi/.ros/h264stream");

    if(res < 0) {
        printf("initEncode failed with code %d\r\n", res);
        ros::shutdown();
    }

    ros::Rate loop_rate(100);
    ros::Timer publishTimer = n.createTimer(ros::Duration(0.5), timerCallback);
    //ros::Timer diskTimer = n.createTimer(ros::Duration(1.0), boost::bind(diskStatTimerCallback, n));

    cpuPub = n.advertise<std_msgs::Float32>("/system/cpuUtilization", 1, true);
    memPub = n.advertise<std_msgs::Float32>("/system/memUtilization", 1, true);

    ros::Subscriber imageSub = n.subscribe("/camera/image_flight", 1, imageCallback);
    ros::Subscriber particleSub = n.subscribe("/localization/particles", 1, particleCallback);

    ROS_INFO("starting raspi_telemetry node...\r\n");

    ros::spin();

    closeEncode(&encoder);

    return 0;
}
