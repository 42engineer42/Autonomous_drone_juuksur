#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <cstdlib>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "tfmini.h"
#include "ibus.h"

#define SERIAL_DEVICE_FILE "/dev/ttyS0"
//#define SERIAL_DEVICE_FILE "/dev/pts/3"


typedef struct Stm32IoState {
    ros::Publisher rangePub, yawPub, pitchPub, rollPub;
    ros::Subscriber yawSub, pitchSub, rollSub, throttleSub;
    IbusState ibus;
    IbusChannels txChannels;
    uint8_t txBuf[32];
    uint8_t rxBuf[256];
    uint16_t ctrlYaw, ctrlPitch, ctrlRoll, ctrlThrottle;
} Stm32IoState;

Stm32IoState stmio;

void ibus_event(void *usrData, IbusEvent *e);
void publish_data(Stm32IoState *stmio, IbusChannels *ch);

void ibus_event(void *usrData, IbusEvent *e) {
	switch(e->type) {
		case IBUS_CMD_CH:
			//printf("ch %d %d %d %d\r\n", e->e.ch.channels[0], e->e.ch.channels[1], e->e.ch.channels[2], e->e.ch.channels[3]);
            publish_data((Stm32IoState*)usrData, &e->e.ch);
			break;
	}
}

void publish_data(Stm32IoState *stmio, IbusChannels *ch) {
    int16_t *yawRaw, *pitchRaw, *rollRaw;
    std_msgs::UInt16 range;
    std_msgs::Float32 yaw, pitch, roll;

    yawRaw = (int16_t*)(ch->channels + 1);
    pitchRaw = (int16_t*)(ch->channels + 2);
    rollRaw = (int16_t*)(ch->channels + 3);
    yaw.data = (float)(*yawRaw) / 100.0f;
    pitch.data = (float)(*pitchRaw) / 100.0f;
    roll.data = (float)(*rollRaw) / 100.0f;

    // TODO: account angle!!!!
    range.data = ch->channels[0];

    stmio->rangePub.publish(range);
    stmio->yawPub.publish(yaw);
    stmio->pitchPub.publish(pitch);
    stmio->rollPub.publish(roll);
    printf("dist %dcm yaw %f pitch %f roll %f\r\n", range.data, yaw.data, pitch.data, roll.data);
}

// TODO: can this be done with 1 callback?
void yawCb(const std_msgs::UInt16 msg) {
	stmio.ctrlYaw = msg.data;
	printf("yaw %d\r\n", msg.data);
}
void pitchCb(const std_msgs::UInt16 msg) {
	stmio.ctrlPitch = msg.data;
}
void rollCb(const std_msgs::UInt16 msg) {
	stmio.ctrlRoll = msg.data;
}
void throttleCb(const std_msgs::UInt16 msg) {
	stmio.ctrlThrottle = msg.data;
}

int main(int argc, char **argv)
{

	ibus_init(&stmio.ibus);
	ibus_set_event_callback(&stmio.ibus, ibus_event, &stmio);

	for(int i = 0; i < 14; i++) {
		stmio.txChannels.channels[i] = 1666;
	}
    std::cout << "Starting..." << std::endl;

	int uartFd = -1;
	uartFd = open(SERIAL_DEVICE_FILE, O_RDWR, O_NOCTTY, O_NDELAY);
	if(uartFd < 0) {
		std::cerr << "Failed to open serial port! errno " << errno << std::endl;
		std::exit(EXIT_FAILURE);
	}
	
	// set non-blocking mode
	int uartFdFlags = fcntl(uartFd, F_GETFL);
	fcntl(uartFd, F_SETFL, uartFdFlags | O_NONBLOCK);
	
	// set low latency mode
	struct serial_struct serial;
	ioctl(uartFd, TIOCGSERIAL, &serial);
	serial.flags |= ASYNC_LOW_LATENCY;
	ioctl(uartFd, TIOCSSERIAL, &serial);

	struct termios serialOpt;
	tcgetattr(uartFd, &serialOpt);
	serialOpt.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	serialOpt.c_iflag = IGNPAR; // ignore characters with parity errors
	serialOpt.c_oflag = 0;
	serialOpt.c_lflag = 0;
	serialOpt.c_cc[VTIME] = 0; // burst timer (tenths of second)
	serialOpt.c_cc[VMIN] = 1; // min number of ch to wait
	tcflush(uartFd, TCIFLUSH);
	tcsetattr(uartFd, TCSANOW, &serialOpt);

    ros::init(argc, argv, "example_node");
    ros::NodeHandle n("~");
    ros::Rate loop_rate(50);

	
    // publis range, yaw, pitch, roll
	stmio.rangePub = n.advertise<std_msgs::UInt16>("range", 1); // 1 - queue size
    // these are actual angles reported by FC telemetry, not stick controls
    stmio.yawPub = n.advertise<std_msgs::Float32>("telemetry/yaw", 1);
    stmio.pitchPub = n.advertise<std_msgs::Float32>("telemetry/pitch", 1);
    stmio.rollPub = n.advertise<std_msgs::Float32>("telemetry/roll", 1);

    stmio.yawSub = n.subscribe<std_msgs::UInt16>("control/yaw", 1, yawCb);
    stmio.pitchSub = n.subscribe<std_msgs::UInt16>("control/pitch", 1, pitchCb);
    stmio.rollSub = n.subscribe<std_msgs::UInt16>("control/roll", 1, rollCb);
    stmio.throttleSub = n.subscribe<std_msgs::UInt16>("control/throttle", 1, throttleCb);

    // alternative control flow:
    // http://wiki.ros.org/roscpp_tutorials/Tutorials/Timers

    while (ros::ok())
    {
        ros::spinOnce();
	    // NOTE: checksum mismatch loop
		int rxCount = read(uartFd, (void*)stmio.rxBuf, 255);
		if(rxCount > 0) {
            //std::cout << "read() success " << errno << std::endl;
			for(int i = 0; i < rxCount; i++) {
				ibus_receive(&stmio.ibus, stmio.rxBuf[i]);
			}
		}
        else if(rxCount < 0 && errno == EAGAIN) { // nothing received (either nonblocking socket or something)
			//printf("rx 0\r\n");
        }
        else {
			std::cerr << "read() on serial port failed! " << errno << std::endl;
		} 

        // note 0 - roll, 1 - pitch, 2 - throttle, 3 - yaw
		stmio.txChannels.channels[0] = stmio.ctrlRoll;
		stmio.txChannels.channels[1] = stmio.ctrlPitch;
		stmio.txChannels.channels[2] = stmio.ctrlThrottle;
		stmio.txChannels.channels[3] = stmio.ctrlYaw;
		int mres = ibus_make_packet(&stmio.txChannels, stmio.txBuf, sizeof(stmio.txBuf));
		if(mres >= 0) {
			write(uartFd, stmio.txBuf, sizeof(stmio.txBuf));
			//printf("send\r\n");
		} else {
			printf("Failed to create packet, buffer too small??\r\n");
		}

        loop_rate.sleep();
    }
}
