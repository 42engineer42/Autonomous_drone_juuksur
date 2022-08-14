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
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <math.h>

#include "tfmini.h"
#include "ibus.h"

// NOTE: because we get angles from FC and calibration affects the numbers we need to compensate
#define PITCH_COMPENSATION 0.0f

// This code is ugly ang hacky but if it works, it works

// NOTE: these can be same
#if 0
// for tesing on PC
#define SERIAL_DEVICE_READ_FILE "/dev/pts/3"
#define SERIAL_DEVICE_WRITE_FILE "/dev/pts/2"
#else 
// for Pi
#define SERIAL_DEVICE_READ_FILE "/dev/serial0"
#define SERIAL_DEVICE_WRITE_FILE "/dev/serial0"
#endif

#define STMIO_FLAG_CTRL_YAW_VALID (1<<0)
#define STMIO_FLAG_CTRL_PITCH_VALID (1<<1)
#define STMIO_FLAG_CTRL_ROLL_VALID (1<<2)
#define STMIO_FLAG_CTRL_THROTTLE_VALID (1<<3)
#define STMIO_FLAG_CTRL_ARM_VALID (1<<4)
#define STMIO_FLAG_CTRL_OFX_VALID (1<<5)
#define STMIO_FLAG_CTRL_OFY_VALID (1<<6)
#define STMIO_FLAG_CTRL_VELX_VALID (1<<7)
#define STMIO_FLAG_CTRL_VELY_VALID (1<<8)
#define STMIO_FLAG_CTRL_HEIGHT_VALID (1<<9)
#define STMIO_FLAG_CTRL_MODE_VALID (1<<10)

#define DEG2RAD 0.01745329251f

typedef struct Stm32IoState {
    uint32_t flags;
    std::atomic_bool senderRunning;
    ros::Publisher rangePub, pitchPub, rollPub;
    ros::Publisher heightPub, heightVelPub;
    ros::Publisher accXPub, accYPub, accZPub;
    ros::Publisher ctrlModePub;
    ros::Publisher velXPub, velYPub;
    ros::Subscriber yawSub, pitchSub, rollSub, throttleSub, armSub;
    ros::Subscriber ofXSub, ofYSub;
    ros::Subscriber velXSub, velYSub;
    ros::Subscriber heightSub;
    ros::Subscriber modeSub;

    ros::Publisher sendPub;

    uint32_t lastVelIdx;
    uint32_t lastVelSent;

    IbusState ibus;
    IbusChannels txChannels;
    IbusChannels lastRxChannels;
    uint8_t txBuf[32];
    uint8_t rxBuf[256];
    uint16_t ctrlYaw, ctrlPitch, ctrlRoll, ctrlThrottle, ctrlArm;
    float ctrlVelX, ctrlVelY;
    float ctrlHeight;
    uint16_t ctrlMode;
    float ofX, ofY;
	int uartFd;
} Stm32IoState;

Stm32IoState stmio;

void ibus_event(void *usrData, IbusEvent *e);
void publish_data(Stm32IoState *stmio, IbusChannels *ch);
int getSerial(const char *fileName);

void ibus_event(void *usrData, IbusEvent *e) {
	switch(e->type) {
		case IBUS_CMD_CH:
			//ROS_DEBUG("ch %d %d %d %d\r\n", e->e.ch.channels[0], e->e.ch.channels[1], e->e.ch.channels[2], e->e.ch.channels[3]);
            publish_data((Stm32IoState*)usrData, &e->e.ch);
			break;
	}
}

void publish_data(Stm32IoState *stmio, IbusChannels *ch) {
    int16_t *pitchRaw, *rollRaw;
    int16_t *accXRaw, *accYRaw, *accZRaw;
    int16_t *velXRaw, *velYRaw;
    std_msgs::Float32 range;
    std_msgs::Float32 pitch, roll;
    std_msgs::Float32 accX, accY, accZ;
    std_msgs::Float32 heightM, heightVelM;
    std_msgs::Int32 ctrlMode, landMode;
    std_msgs::Float32 velX, velY;
    stmio->lastRxChannels = *ch;

    pitchRaw = (int16_t*)(ch->channels + 2);
    rollRaw = (int16_t*)(ch->channels + 3);
    accXRaw = (int16_t*)(ch->channels + 4);
    accYRaw = (int16_t*)(ch->channels + 5);
    accZRaw = (int16_t*)(ch->channels + 6);
    velXRaw = (int16_t*)(ch->channels + 1);
    velYRaw = (int16_t*)(ch->channels + 13);

    float height;
    float heightVel;
    memcpy(&height, ch->channels+9, 4);
    memcpy(&heightVel, ch->channels+11, 4);
    heightM.data = height;
    heightVelM.data = heightVel;

    pitch.data = ((float)(*pitchRaw) / 100.0f) + PITCH_COMPENSATION;
    roll.data = (float)(*rollRaw) / 100.0f;
    accX.data = (float)(*accXRaw);
    accY.data = (float)(*accYRaw);
    accZ.data = (float)(*accZRaw);
    velX.data = (float)(*velXRaw) / 10.0f;
    velY.data = (float)(*velYRaw) / 10.0f;

    // ctrlmode - 0 is manual, 1 is autonomous, 2 is landing
    ctrlMode.data = ch->channels[7] < 1800 ? 0 : (ch->channels[8] < 1800 ? 1 : 2);

    range.data = (float)ch->channels[0] * cos(DEG2RAD * roll.data) * cos(DEG2RAD * pitch.data);
    if(range.data < 0.0f) { // extreme angle
	range.data = (float)ch->channels[0];
    }

    stmio->rangePub.publish(range);
    stmio->pitchPub.publish(pitch);
    stmio->rollPub.publish(roll);
    stmio->accXPub.publish(accX);
    stmio->accYPub.publish(accY);
    stmio->accZPub.publish(accZ);
    stmio->ctrlModePub.publish(ctrlMode);
    stmio->heightPub.publish(heightM);
    stmio->heightVelPub.publish(heightVelM);
    stmio->velXPub.publish(velX);
    stmio->velYPub.publish(velY);

    //ROS_INFO("dist %d cm acc %.2f \r\n", ch->channels[0], accZ.data);
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
void ofXCb(const std_msgs::Float32 msg) {
    stmio.ofX = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_OFX_VALID;
    stmio.lastVelIdx++;
}
void ofYCb(const std_msgs::Float32 msg) {
    stmio.ofY = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_OFY_VALID;
}
void velXCb(const std_msgs::Float32 msg) {
    stmio.ctrlVelX = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_VELX_VALID;
}
void velYCb(const std_msgs::Float32 msg) {
    stmio.ctrlVelY = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_VELY_VALID;
}
void heightCb(const std_msgs::Float32 msg) {
    stmio.ctrlHeight = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_HEIGHT_VALID;
}
void modeCb(const std_msgs::UInt16 msg) {
    stmio.ctrlMode = msg.data;
    stmio.flags |= STMIO_FLAG_CTRL_MODE_VALID;
}

void *senderThreadProc(void *usrData) {
    int fd = getSerial(SERIAL_DEVICE_WRITE_FILE);
    Stm32IoState *s = (Stm32IoState*)usrData;
    while(s->senderRunning) {
        int txCount;
        // NOTE: even if reader thread writes stmio.ctrlRoll etc at same time it shouldn't matter
        // NOTE: 0 - roll, 1 - pitch, 2 - throttle, 3 - yaw
        // NOTE: sends 1500 if subscriber hasn't got any data (1000 for throttle)
        int16_t ofXmm = (int16_t)(stmio.ofX*10);
        int16_t ofYmm = (int16_t)(stmio.ofY*10);
        int16_t velXmm = (int16_t)(stmio.ctrlVelX*10);
        int16_t velYmm = (int16_t)(stmio.ctrlVelY*10);
        int16_t heightMm = (int16_t)(stmio.ctrlHeight*10);
        s->txChannels.channels[0] = (stmio.flags & STMIO_FLAG_CTRL_ROLL_VALID) ? stmio.ctrlRoll : IBUS_CHANNEL_DEFAULT;
        s->txChannels.channels[1] = (stmio.flags & STMIO_FLAG_CTRL_PITCH_VALID) ? stmio.ctrlPitch : IBUS_CHANNEL_DEFAULT;
        s->txChannels.channels[2] = (stmio.flags & STMIO_FLAG_CTRL_THROTTLE_VALID) ? stmio.ctrlThrottle : 1000;
        s->txChannels.channels[3] = (stmio.flags & STMIO_FLAG_CTRL_YAW_VALID) ? stmio.ctrlYaw : IBUS_CHANNEL_DEFAULT;
        s->txChannels.channels[4] = (stmio.flags & STMIO_FLAG_CTRL_ARM_VALID) ? stmio.ctrlArm : IBUS_CHANNEL_DEFAULT;
        s->txChannels.channels[7] = stmio.lastRxChannels.channels[8]; // rx ch 8 is land, dont know what this does in retrospect
        if(stmio.lastVelIdx > stmio.lastVelSent) {
            s->txChannels.channels[8] = (stmio.flags & STMIO_FLAG_CTRL_OFX_VALID) ? *((uint16_t*)&ofXmm) : 0;
            s->txChannels.channels[9] = (stmio.flags & STMIO_FLAG_CTRL_OFY_VALID) ? *((uint16_t*)&ofYmm) : 0;
            stmio.lastVelSent = stmio.lastVelIdx;
        } else {
            s->txChannels.channels[8] = 0xFFFF;
            s->txChannels.channels[9] = 0xFFFF;
        }
        s->txChannels.channels[10] = (stmio.flags & STMIO_FLAG_CTRL_VELX_VALID) ? *((uint16_t*)&velXmm) : 0;
        s->txChannels.channels[11] = (stmio.flags & STMIO_FLAG_CTRL_VELY_VALID) ? *((uint16_t*)&velYmm) : 0;
        s->txChannels.channels[12] = (stmio.flags & STMIO_FLAG_CTRL_HEIGHT_VALID) ? *((uint16_t*)&heightMm) : 0;
        s->txChannels.channels[13] = (stmio.flags & STMIO_FLAG_CTRL_MODE_VALID) ? stmio.ctrlMode : 0;
        int mres = ibus_make_packet(&s->txChannels, s->txBuf, sizeof(s->txBuf));
        if(mres >= 0) {
            txCount = write(fd, s->txBuf, sizeof(s->txBuf));
	    std_msgs::Int32 sendmsg;
	    sendmsg.data = 1;
	    s->sendPub.publish(sendmsg);
            // TODO: on partial write this isn't correct, but should not be fatal (if we use nonblocking fd)
            if(txCount < sizeof(s->txBuf)) {
                ROS_ERROR("Send fail: %s\r\n", strerror(errno));
            }
            //ROS_DEBUG("send %d\r\n", txCount);
        } else {
            ROS_ERROR("Failed to create packet, buffer too small??\r\n");
        }
        usleep(7000); // in microseconds!, nominally 142hz (7ms)
    }
    close(fd);
    return NULL;
}

int getSerial(const char *fileName) {
    int fd = -1;
	fd = open(fileName, O_RDWR | O_NOCTTY);
	if(fd < 0) {
        ROS_FATAL_STREAM("Failed to open "<< fileName << " - " << strerror(errno) << std::endl);
        ros::shutdown();
	}
	struct serial_struct serial;
	ioctl(fd, TIOCGSERIAL, &serial);
	serial.flags |= ASYNC_LOW_LATENCY;
	ioctl(fd, TIOCSSERIAL, &serial);
	struct termios serialOpt;
	tcgetattr(fd, &serialOpt);
	serialOpt.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	serialOpt.c_iflag = IGNPAR; // ignore characters with parity errors
	serialOpt.c_oflag = 0;
	serialOpt.c_lflag = 0;
    // VTIME=0 VMIN=0 is basically nonblocking mode (for reads, write() should be unaffected)
	serialOpt.c_cc[VTIME] = 0; // burst timer (tenths of second)
	serialOpt.c_cc[VMIN] = 1; // min number of ch to wait
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &serialOpt);
    return fd;
}

int main(int argc, char **argv)
{
	ibus_init(&stmio.ibus);
	ibus_set_event_callback(&stmio.ibus, ibus_event, &stmio);

	for(int i = 0; i < 14; i++) {
		stmio.txChannels.channels[i] = 1501;
	}
    ROS_INFO("Starting stm32_io node...\r\n");

    stmio.lastVelIdx = 0;

    stmio.uartFd = getSerial(SERIAL_DEVICE_READ_FILE);
    stmio.flags = 0;
	
    ros::init(argc, argv, "example_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(200); // data is sent by stm32 at approx 142Hz
	
    // publis range, pitch, roll
    stmio.rangePub = n.advertise<std_msgs::Float32>("/telemetry/range", 1, true); // 1 - queue size
    // these are actual angles reported by FC telemetry, not stick controls
    stmio.pitchPub = n.advertise<std_msgs::Float32>("/telemetry/pitch", 1, true);
    stmio.rollPub = n.advertise<std_msgs::Float32>("/telemetry/roll", 1, true);
    stmio.accXPub = n.advertise<std_msgs::Float32>("/telemetry/accX", 1, true);
    stmio.accYPub = n.advertise<std_msgs::Float32>("/telemetry/accY", 1, true);
    stmio.accZPub = n.advertise<std_msgs::Float32>("/telemetry/accZ", 1, true);
    stmio.ctrlModePub = n.advertise<std_msgs::Int32>("/telemetry/controlmode", 1, true);
    stmio.heightPub = n.advertise<std_msgs::Float32>("/telemetry/height", 1, true);
    stmio.heightVelPub = n.advertise<std_msgs::Float32>("/telemetry/heightVel", 1, true);
    stmio.velXPub = n.advertise<std_msgs::Float32>("/telemetry/velX", 1, true);
    stmio.velYPub = n.advertise<std_msgs::Float32>("/telemetry/velY", 1, true);
    // for debugging sending
    stmio.sendPub = n.advertise<std_msgs::Int32>("/telemetry/sendevent", 1);

    stmio.yawSub = n.subscribe<std_msgs::UInt16>("/control/yaw", 1, yawCb);
    stmio.pitchSub = n.subscribe<std_msgs::UInt16>("/control/pitch", 1, pitchCb);
    stmio.rollSub = n.subscribe<std_msgs::UInt16>("/control/roll", 1, rollCb);
    stmio.throttleSub = n.subscribe<std_msgs::UInt16>("/control/throttle", 1, throttleCb);
    stmio.armSub = n.subscribe<std_msgs::UInt16>("/control/arm", 1, armCb);
    stmio.ofXSub = n.subscribe<std_msgs::Float32>("/telemetry/rawOpticalFlowX", 1, ofXCb);
    stmio.ofYSub = n.subscribe<std_msgs::Float32>("/telemetry/rawOpticalFlowY", 1, ofYCb);
    stmio.velXSub = n.subscribe<std_msgs::Float32>("/control/velX", 1, velXCb);
    stmio.velYSub = n.subscribe<std_msgs::Float32>("/control/velY", 1, velYCb);
    stmio.heightSub = n.subscribe<std_msgs::Float32>("/control/height", 1, heightCb);
    stmio.modeSub = n.subscribe<std_msgs::UInt16>("/control/mode", 1, modeCb);

    pthread_t senderThread;
    stmio.senderRunning = true;
    if(pthread_create(&senderThread, NULL, senderThreadProc, &stmio)) {
        ROS_FATAL("Failed to create sender thread!\r\n");
        ros::shutdown();
    }

    // NOTE: this thread only reads and publishes, writing back to stm32 is done by a different thread
    // this should reduce latency on both sending and receiving (since OS can do scheduling better than us)
    while (ros::ok())
    {
        ros::spinOnce();
		// NOTE: checksum mismatch loop
		int rxCount = read(stmio.uartFd, (void*)stmio.rxBuf, 255);
		if(rxCount > 0) {
            //std::cout << "read() success " << errno << std::endl;
			//ROS_DEBUG("R %d\r\n", rxCount);
			for(int i = 0; i < rxCount; i++) {
				ibus_receive(&stmio.ibus, stmio.rxBuf[i]);
			}
		}
        else if(rxCount <= 0 && errno == EAGAIN) { // nothing received (must be nonblocking mode)
			//ROS_DEBUG("EAGAIN\r\n");
			//ROS_DEBUG("rx 0\r\n");
        }
        else if(errno != EINTR){ // EINTR occurs on exit, we don't care about it
            ROS_ERROR_STREAM("read() on serial port failed! " << errno << " " << rxCount << std::endl);
		} 
        //loop_rate.sleep();
    }

	close(stmio.uartFd);
	stmio.uartFd = -1;

    stmio.senderRunning = false;
    pthread_join(senderThread, NULL); // wait sender to exit
}
