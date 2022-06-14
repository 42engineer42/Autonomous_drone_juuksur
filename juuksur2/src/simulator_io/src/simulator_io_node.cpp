#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

#include <atomic>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <cstdlib>
#include <sys/ioctl.h>
#include <pthread.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <arpa/inet.h>

#include <chrono>

#include "velocity_controller.h"
#include "height_controller.h"
#include "VelocityEstimator.h"

//#include <tf/transform_broadcaster.h>

// This code is ugly ang hacky but if it works, it works

#define MSP_PORT 5761
#define MSP_IP "127.0.0.1"

#define STMIO_FLAG_CTRL_YAW_VALID (1<<0)
#define STMIO_FLAG_CTRL_PITCH_VALID (1<<1)
#define STMIO_FLAG_CTRL_ROLL_VALID (1<<2)
#define STMIO_FLAG_CTRL_THROTTLE_VALID (1<<3)
#define STMIO_FLAG_CTRL_ARM_VALID (1<<4)
#define STMIO_FLAG_CTRL_MODE_VALID (1<<5)

#define DEG2RAD 0.01745329251f

const char* armflags[] = {
    "NOGYRO",       // 0
    "FAILSAFE",     // 1
    "RXLOSS",       // 2
    "BADRX",        // 3
    "BOXFAILSAFE",  // 4
    "RUNAWAY",      // 5
    "THROTTLE",     // 6
    "ANGLE",        // 7 
    "BOOTGRACE",    // 8 
    "NOPREARM",     // 9
    "LOAD",         // 10
    "CALIB",        // 11
    "CLI",          // 12
    "CMS",          // 13
    "BST",          // 14
    "MSP",          // 15
    "PARALYZE",     // 16
    "GPS",          // 17
    "RESCUE_SW",    // 18
    "RPMFILTER",    // 19
    "ARMSWITCH",    // 20
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
    "",
};

typedef struct SensorPacket {
    int32_t range;
    float rotation[3];
    float acceleration[3];
    float velocity[3];
    float angularVelocity[3];
    int32_t mode;
} SensorPacket;

SensorPacket lastPacket;
float runTime;

typedef struct Stm32IoState {
    uint32_t flags;
    std::atomic_bool senderRunning;
    ros::Publisher rangePub, yawPub, pitchPub, rollPub;
    ros::Publisher heightPub, heightVelPub;
    ros::Publisher accXPub, accYPub, accZPub;
    ros::Publisher gyroXPub, gyroYPub, gyroZPub;
    ros::Publisher ctrlModePub;
    ros::Publisher velXPub, velYPub;
    ros::Publisher sVelXPub, sVelYPub;
    ros::Subscriber yawSub, pitchSub, rollSub, throttleSub, armSub;
    ros::Subscriber modeSub;
    ros::Subscriber ofXSub, ofYSub;
    ros::Subscriber cheightSub;
    ros::Subscriber cvelXSub, cvelYSub;
    int sockFd;
    uint8_t txBuf[32];
    uint8_t rxBuf[256];

    //tf::TransformBroadcaster *tfBc;

    uint16_t txChannels[8];

    uint32_t lastVelIdx;
    uint32_t lastVelSent;

    uint16_t ctrlYaw, ctrlPitch, ctrlRoll, ctrlThrottle, ctrlArm;
    uint16_t ctrlMode;
    float ofX, ofY;
    float cHeight, cVelX, cVelY;

    std::chrono::steady_clock::time_point lastOfUpdate;
} Stm32IoState;

Stm32IoState stmio;

VelocityController vCtrl;
HeightController hCtrl;
VelocityEstimator vEst;

float accXW, accYW, accZW;

void publish_data(Stm32IoState *stmio, uint16_t *ch);

void publish_data(Stm32IoState *stmio, uint16_t *ch2) {
    int16_t *yawRaw, *pitchRaw, *rollRaw;
    int16_t *accXRaw, *accYRaw, *accZRaw;
    std_msgs::Float32 range;
    std_msgs::Float32 yaw, pitch, roll;
    std_msgs::Float32 accX, accY, accZ;
    std_msgs::Float32 gyroX, gyroY, gyroZ;
    std_msgs::Int32 ctrlMode, landMode;
    std_msgs::Float32 heightVel;

    // pitch roll angles should be in range -180..180
    // from unity they come 0..360
    yaw.data = lastPacket.rotation[1];
    float pitchf = lastPacket.rotation[2];
    float rollf = lastPacket.rotation[0];
    pitch.data = pitchf < 180.0f ? pitchf : -(360.0f-pitchf);
    roll.data = rollf < 180.0f ? rollf : -(360.0f-rollf);

    float ax = lastPacket.acceleration[0]*100.0f;
    float ay = lastPacket.acceleration[2]*100.0f;
    float az = lastPacket.acceleration[1]*100.0f;

    float alpha = rollf*DEG2RAD;
    float beta = pitchf*DEG2RAD;
    accX.data = (ax*cosf(beta) + az*cosf(alpha)*sinf(beta) + ay*sinf(alpha)*sinf(beta));
    accY.data = (ay*cosf(alpha) - az*sinf(alpha));
    accZ.data = (az*cosf(alpha)*cosf(beta) - ax*sinf(beta) + ay*cosf(beta)*sinf(alpha));

    accXW = accX.data;
    accYW = accY.data;
    accZW = accZ.data;

    gyroX.data = -lastPacket.angularVelocity[0];
    gyroY.data = -lastPacket.angularVelocity[2];
    gyroZ.data = lastPacket.angularVelocity[1];
    // ctrlmode - 0 is manual, 1 is autonomous, 2 is landing
    //ctrlMode.data = ch->channels[7] < 1800 ? 0 : (ch->channels[8] < 1800 ? 1 : 2);
    ctrlMode.data = lastPacket.mode;
    heightVel.data = lastPacket.velocity[1]*100.0f;

    range.data = ((float)lastPacket.range*cos(DEG2RAD*roll.data)*cos(DEG2RAD*pitch.data));

    stmio->rangePub.publish(range);
    stmio->heightPub.publish(range);
    stmio->heightVelPub.publish(heightVel);
    stmio->yawPub.publish(yaw);
    stmio->pitchPub.publish(pitch);
    stmio->rollPub.publish(roll);
    stmio->accXPub.publish(accX);
    stmio->accYPub.publish(accY);
    stmio->accZPub.publish(accZ);
    stmio->gyroXPub.publish(gyroX);
    stmio->gyroYPub.publish(gyroY);
    stmio->gyroZPub.publish(gyroZ);
    stmio->ctrlModePub.publish(ctrlMode);

    std_msgs::Float32 f;
    f.data = vEst.velX;
    stmio->velXPub.publish(f);
    f.data = vEst.velY;
    stmio->velYPub.publish(f);

    f.data = lastPacket.velocity[0]*100.0f;
    stmio->sVelXPub.publish(f);
    f.data = lastPacket.velocity[2]*100.0f;
    stmio->sVelYPub.publish(f);

    /*tf::Quaternion rotQ = tf::createQuaternionFromRPY(roll.data, pitch.data, yaw.data);
    stmio->tfBc->sendTransform(tf::StampedTransform(
                tf::Transform(rotQ, tf::Vector3(0.0, 0.0, 1.3)),
                ros::Time::now(), "base_link", "base_drone"));*/
    //ROS_INFO("raw dist %04dcm yaw %.2f(%04d) pitch %.2f(%04d) roll %.2f(%04d) acc (%.2f, %.2f, %.2f)\r\n", range.data, yaw.data, *yawRaw, pitch.data, *pitchRaw, roll.data, *rollRaw, accX.data, accY.data, accZ.data);
}

// TODO: can this be done with 1 callback?
void yawCb(const std_msgs::UInt16 msg) {
	stmio.ctrlYaw = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_YAW_VALID;
}
void pitchCb(const std_msgs::UInt16 msg) {
	stmio.ctrlPitch = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_PITCH_VALID;
}
void rollCb(const std_msgs::UInt16 msg) {
	stmio.ctrlRoll = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_ROLL_VALID;
}
void throttleCb(const std_msgs::UInt16 msg) {
	stmio.ctrlThrottle = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_THROTTLE_VALID;
}
void armCb(const std_msgs::UInt16 msg) {
	stmio.ctrlArm = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_ARM_VALID;
}
void modeCb(const std_msgs::UInt16 msg) {
    stmio.ctrlMode = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_MODE_VALID;
}
void ofXCb(const std_msgs::Float32 msg) {
    stmio.ofX = msg.data;
    stmio.lastVelIdx++;
}
void ofYCb(const std_msgs::Float32 msg) {
    stmio.ofY = msg.data;
    double dt;
    std::chrono::duration<double> dif = std::chrono::steady_clock::now() - stmio.lastOfUpdate;
    dt = dif.count();
    if(stmio.lastVelIdx > stmio.lastVelSent) {
        velocity_estimator_update(&vEst, stmio.ofY, stmio.ofX, dt);
        stmio.lastVelSent = stmio.lastVelIdx;
    }
    stmio.lastOfUpdate = std::chrono::steady_clock::now();
}
void heightCb(const std_msgs::Float32 msg) {
    stmio.cHeight = msg.data;
}
void velXCb(const std_msgs::Float32 msg) {
    stmio.cVelX = msg.data;
}
void velYCb(const std_msgs::Float32 msg) {
    stmio.cVelY = msg.data;
}

void disable_msp_arm(Stm32IoState *s) {
    uint8_t sendBuf[255];
    sendBuf[0] = '$';
    sendBuf[1] = 'M';
    sendBuf[2] = '<';
    sendBuf[3] =  1;
    sendBuf[4] = 99;
    sendBuf[5] = 0;
    sendBuf[6] = sendBuf[3]^sendBuf[4]^sendBuf[5];
    send(s->sockFd, sendBuf, 6, 0);
}

void request_status(Stm32IoState *s) {
    uint8_t sendBuf[255];
    sendBuf[0] = '$';
    sendBuf[1] = 'M';
    sendBuf[2] = '<';
    sendBuf[3] =  0;
    sendBuf[4] = 101;
    sendBuf[5] = sendBuf[3]^sendBuf[4];
    send(s->sockFd, sendBuf, 6, 0);
}

void *senderThreadProc(void *usrData) {
    // TODO: get tcp socket?
    int fd = 0;
    Stm32IoState *s = (Stm32IoState*)usrData;
    uint8_t sendBuf[255];

    stmio.lastVelIdx = 0;
    stmio.lastVelSent = 0;
    uint16_t ctrlModeLast = 0;

    velocity_estimator_init(&vEst);
    stmio.lastOfUpdate = std::chrono::steady_clock::now();

    while(s->senderRunning) {

        velocity_estimator_predict(&vEst, lastPacket.acceleration[0]*100.0f, lastPacket.acceleration[2]*100.0f, 0.01f);

        int txCount;
        if(stmio.ctrlMode == 0) {
            s->txChannels[0] = 1500;
            s->txChannels[1] = 1500;
            s->txChannels[2] = 1000;
        } else if(stmio.ctrlMode == 1) {
            if(ctrlModeLast == 0) {
                velocity_controller_init(&vCtrl);
                height_controller_init(&hCtrl);
            }
            if(lastPacket.range > 30) {
                vCtrl.setpointX = stmio.cVelX;
                vCtrl.setpointY = stmio.cVelY;
                //velocity_controller_update(&vCtrl, stmio.ofY, stmio.ofX, lastPacket.acceleration[2], lastPacket.acceleration[0], 0.01f);
                velocity_controller_update(&vCtrl, vEst.velX, vEst.velY, accXW, accYW, 0.01f);
                s->txChannels[0] = vCtrl.rollControl;
                s->txChannels[1] = vCtrl.pitchControl;
            } else {
                s->txChannels[0] = 1500;
                s->txChannels[1] = 1500;
            }
            hCtrl.setPoint = stmio.cHeight;
            height_controller_update(&hCtrl, lastPacket.velocity[1]*100.0f, lastPacket.range, 0.01f);
            s->txChannels[2] = hCtrl.throttleControl;

        }
        /*s->txChannels[0] = (stmio.flags & STMIO_FLAG_CTRL_ROLL_VALID) ? stmio.ctrlRoll : 1500;
        s->txChannels[1] = (stmio.flags & STMIO_FLAG_CTRL_PITCH_VALID) ? stmio.ctrlPitch : 1500;
        s->txChannels[2] = (stmio.flags & STMIO_FLAG_CTRL_THROTTLE_VALID) ? stmio.ctrlThrottle : 1000;*/
        s->txChannels[3] = 1500;
        //s->txChannels[4] = (stmio.flags & STMIO_FLAG_CTRL_ARM_VALID) ? stmio.ctrlArm : 1500;
        static bool armed;
        if(runTime > 3.0f) {
            s->txChannels[4] = 2000;
            if(!armed) {
                printf("Simulator FC armed!\r\n");
            }
            armed = true;
        } else {
            s->txChannels[0] = 1500;
            s->txChannels[1] = 1500;
            s->txChannels[2] = 1000;
            s->txChannels[3] = 1500;
            s->txChannels[4] = 1000;
        }
        //printf("r %d p %d t %d y %d a %d\r\n", stmio.ctrlRoll, stmio.ctrlPitch, stmio.ctrlThrottle, stmio.ctrlYaw, stmio.ctrlArm);
        //s->txChannels[7] = stmio.lastRxChannels.channels[8];
        // Multiwii serial raw receiver channel data
        sendBuf[0] = '$';
        sendBuf[1] = 'M';
        sendBuf[2] = '<';
        sendBuf[3] = 16;
        sendBuf[4] = 200;
        int idx = 5;
        for(int i = 0; i < 8; i++) {
            uint16_t *m = (uint16_t*)(sendBuf+idx);
            *m = s->txChannels[i];
            idx += 2;
        }
        uint8_t cs = 0;
        for(int i = 0; i < sendBuf[3]+2; i++) {
            cs ^= sendBuf[3+i];
        }
        sendBuf[idx++] = cs;
        ssize_t scount = send(s->sockFd, sendBuf, idx, 0);
        if(scount < idx) {
            fprintf(stderr, "Not all MSP data sent, only sent %ld of %d\r\n", scount, idx);
        }        
        ctrlModeLast = stmio.ctrlMode;
        usleep(10000); // in microseconds!
        runTime += 0.01f;
    }
    return NULL;
}

int main(int argc, char **argv)
{
    uint8_t recvBuf[255];
	for(int i = 0; i < 8; i++) {
		stmio.txChannels[i] = 1501;
	}
    ROS_INFO("Starting simulator_io node...\r\n");

    // TODO: setup tcp socket
    int sockFd;
    struct sockaddr_in mspAddr;
    inet_pton(AF_INET, MSP_IP, &(mspAddr.sin_addr));
    if((sockFd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
        fprintf(stderr, "failed to create TCP socket for betaflight MSP controller, errno: %d\r\n", errno);
        exit(1);
    }

    mspAddr.sin_family = AF_INET;
    mspAddr.sin_port = htons(MSP_PORT);
    bzero(&(mspAddr.sin_zero), 8);
    while(connect(sockFd, (struct sockaddr *)&mspAddr, sizeof(struct sockaddr)) < 0) {
        fprintf(stderr, "simulator_io betaflight connect error %d, retrying\r\n", errno);
        usleep(1000000);
    }
    puts("connected to betaflight MSP controller\r\n");

    int flags = fcntl(sockFd, F_GETFL, 0);
    if (flags == -1) {
        fprintf(stderr, "fcntl F_GETFL failed in simulator_io\r\n");
        exit(-1);
    }
    flags = (flags | O_NONBLOCK);
    flags = fcntl(sockFd, F_SETFL, flags);
    if(flags == -1) {
        fprintf(stderr, "fcntl failed when trying to set TCP socket nonblocking in simulator_io\r\n");
        exit(-1);
    }

    int sensorSock;
    int sstatus = -1;
    struct sockaddr_in sensAddr;
    inet_pton(AF_INET, "127.0.0.1", &(sensAddr.sin_addr));
    if((sensorSock = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        fprintf(stderr, "failed to create UDP socket for simulator sensor data\r\n");
        exit(1);
    }
    sensAddr.sin_family = AF_INET;
    sensAddr.sin_port = htons(7004);
    bzero(&(sensAddr.sin_zero), 8);
    sstatus = bind(sensorSock, (struct sockaddr*)&sensAddr, sizeof(struct sockaddr));
    if(sstatus < 0) {
        fprintf(stderr, "Failed to bind UDP socket in simulator_io\r\n");
        exit(-1);
    }
    flags = fcntl(sensorSock, F_GETFL, 0);
    if(flags == -1) {
        fprintf(stderr, "F_GETFL failed on UDP socket in simulator_io\r\n");
        exit(-1);
    }
    flags = (flags | O_NONBLOCK);
    flags = fcntl(sensorSock, F_SETFL, flags);
    if(flags == -1) {
        fprintf(stderr, "Failed to set UDP socket non blocking in simulator_io\r\n");
        exit(-1);
    }

    puts("sensor UDP socket ready\r\n");

    stmio.sockFd = sockFd;

    pthread_t senderThread;
    stmio.senderRunning = true;
    if(pthread_create(&senderThread, NULL, senderThreadProc, &stmio)) {
        ROS_FATAL("Failed to create sender thread!\r\n");
        ros::shutdown();
    }

    ros::init(argc, argv, "simulator_io_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(300); // data is sent by stm32 at approx 142Hz

    /*tf::TransformBroadcaster tfBc;
    stmio.tfBc = &tfBc;*/
	
    // publis range, yaw, pitch, roll
    stmio.rangePub = n.advertise<std_msgs::Float32>("/telemetry/range", 1, true); // 1 - queue size
    stmio.heightPub = n.advertise<std_msgs::Float32>("/telemetry/height", 1, true);
    stmio.heightVelPub = n.advertise<std_msgs::Float32>("/telemetry/heightVel", 1, true);
    // these are actual angles reported by FC telemetry, not stick controls
    stmio.yawPub = n.advertise<std_msgs::Float32>("/telemetry/yaw", 1, true);
    stmio.pitchPub = n.advertise<std_msgs::Float32>("/telemetry/pitch", 1, true);
    stmio.rollPub = n.advertise<std_msgs::Float32>("/telemetry/roll", 1, true);
    stmio.accXPub = n.advertise<std_msgs::Float32>("/telemetry/accX", 1, true);
    stmio.accYPub = n.advertise<std_msgs::Float32>("/telemetry/accY", 1, true);
    stmio.accZPub = n.advertise<std_msgs::Float32>("/telemetry/accZ", 1, true);
    stmio.gyroXPub = n.advertise<std_msgs::Float32>("/telemetry/gyroX", 1, true);
    stmio.gyroYPub = n.advertise<std_msgs::Float32>("/telemetry/gyroY", 1, true);
    stmio.gyroZPub = n.advertise<std_msgs::Float32>("/telemetry/gyroZ", 1, true);
    stmio.ctrlModePub = n.advertise<std_msgs::Int32>("/telemetry/controlmode", 1, true);
    stmio.velXPub = n.advertise<std_msgs::Float32>("/telemetry/velX", 1, true);
    stmio.velYPub = n.advertise<std_msgs::Float32>("/telemetry/velY", 1, true);
    stmio.sVelXPub = n.advertise<std_msgs::Float32>("/simulator/velX", 1, true);
    stmio.sVelYPub = n.advertise<std_msgs::Float32>("/simulator/velY", 1, true);

    stmio.yawSub = n.subscribe<std_msgs::UInt16>("/control/yaw", 1, yawCb);
    stmio.pitchSub = n.subscribe<std_msgs::UInt16>("/control/pitch", 1, pitchCb);
    stmio.rollSub = n.subscribe<std_msgs::UInt16>("/control/roll", 1, rollCb);
    stmio.throttleSub = n.subscribe<std_msgs::UInt16>("/control/throttle", 1, throttleCb);
	stmio.armSub = n.subscribe<std_msgs::UInt16>("/control/arm", 1, armCb);
    stmio.modeSub = n.subscribe<std_msgs::UInt16>("/control/mode", 1, modeCb);
    stmio.ofXSub = n.subscribe<std_msgs::Float32>("/telemetry/rawOpticalFlowX", 1, ofXCb);
    stmio.ofYSub = n.subscribe<std_msgs::Float32>("/telemetry/rawOpticalFlowY", 1, ofYCb);
    stmio.cheightSub = n.subscribe<std_msgs::Float32>("/control/height", 1, heightCb);
    stmio.cvelXSub = n.subscribe<std_msgs::Float32>("/control/velX", 1, velXCb);
    stmio.cvelYSub = n.subscribe<std_msgs::Float32>("/control/velY", 1, velYCb);

    lastPacket.range = 30;

    // NOTE: this thread only reads and publishes, writing back to stm32 is done by a different thread
    // this should reduce latency on both sending and receiving (since OS can do scheduling better than us)
    uint8_t payload[1024];
    uint8_t pc = 0;
    int status = 0;
    int dataLeft =0;
    int len = 0;
    uint8_t cmd;
    // staircase to heaven (and back)
    while (ros::ok())
    {
        ros::spinOnce();
        // TODO: read tcp socket (for acceleration etc
        ssize_t r = recv(sockFd, &recvBuf, sizeof(recvBuf), 0);
        uint8_t skip = 0;
        if(r > 0) {
            for(int i = 0; i < r; i++) {
                //printf("%02x\r\n", recvBuf[i]);
                switch(status) {
                    case 0:
                    case 1:
                        pc = 0;
                        status++;
                        break;
                    case 2: // direction
                        status++;
                        break;
                    case 3:
                        dataLeft = recvBuf[i];
                        len = dataLeft;
                        status++;
                        break;
                    case 4:
                        cmd = recvBuf[i];
                        status ++;
                        break;
                    case 5:
                        if(dataLeft == 0) {
                            // TODO: check checksum!
                            if(cmd != 200) {
                                //printf("command complete! %d\r\n", cmd);
                            }
                            if(cmd == 101) {
                                //printf("\r\n");
                                //printf("cycletime %d len %d\r\n", payload[0]|(payload[1]<<8), len);
                                //printf("skip %d\r\n", 16);
                                //printf("flagcount %d\r\n", payload[16]);
                                uint32_t flagg = *((uint32_t*)&payload[17]);
                                if(flagg != 0){
                                    puts("arm disabled flags ");
                                    for(int h = 0; h < 32; h++) {
                                        if((flagg>>h)&1) {
                                            printf("%s ", armflags[h]);
                                        }
                                    }
                                    printf("\r\n");
                                }
                                //printf("flags %x %x %x\r\n", payload[16+1], payload[16+2], payload[16+3]);
                                //printf("\r\n");
                            }
                            status = 0;
                        } else {
                            payload[pc++] = recvBuf[i];
                            dataLeft--;
                        }
                        break;
                    default:
                        status = 0;
                        break;
                }
            }
            //printf("\r\n");
        } else if(r == 0){ 
            break;
        } else if(errno != EAGAIN){
            fprintf(stderr, "recv() error %d\r\n", errno);
            exit(-1);
        }

        struct sockaddr addr;
        unsigned int socklen = sizeof(addr);
        int rs = recvfrom(sensorSock, &recvBuf, sizeof(recvBuf), 0, &addr, &socklen);
        if(rs > 0) {
            if(rs == sizeof(SensorPacket)) {
                SensorPacket *pk = (SensorPacket*)recvBuf;
                lastPacket = *pk;
            }
        } else if(rs < 0 && errno != EAGAIN) {
            fprintf(stderr, "recvfrom() error %d\r\n", errno);
            exit(-1);
        }

        loop_rate.sleep();
        //request_status(&stmio);
        publish_data(&stmio, stmio.txChannels);
    }

    stmio.senderRunning = false;
    pthread_join(senderThread, NULL); // wait sender to exit
    close(sensorSock);
}
