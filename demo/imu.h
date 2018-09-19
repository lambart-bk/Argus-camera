
#ifndef __IMU__
#define __IMU__

#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <pthread.h>

typedef unsigned char u8;

struct accel_data {
	u8 xhigh;
	u8 xlow;
	u8 yhigh;
	u8 ylow;
	u8 zhigh;
	u8 zlow;
};

struct gyro_data {
	u8 xhigh;
	u8 xlow;
	u8 yhigh;
	u8 ylow;
	u8 zhigh;
	u8 zlow;
};

struct mag_data {
	u8 xlow;
	u8 xhigh;
	u8 ylow;
	u8 yhigh;
	u8 zlow;
	u8 zhigh;
};

struct mpu9250_data {
	u8  id0;   // fixed 0x55
	u8  id1;   // fixed 0xaa
	struct accel_data ad; // accel data
	u8  blank[2];  // not used
	struct gyro_data gd; // gyro data
	struct mag_data md;  // mag data
	u8  md_status; // md status
	u8  time[4];   // time
};

struct mpu9250_frame {
	u8 buffer[27];
};

#define GET_FPGA   _IOR('S', 0, struct mpu9250_frame)

class IMU
{
public:
	IMU();
	~IMU();
	int IMU_Open();
	int IMU_Close();
	int IMU_ReadAll(struct mpu9250_frame *data);

private:
	char *device = "/dev/mpu92500";
	int fd;	
};

class IMUThread
{
public:
	IMUThread(int time);
	~IMUThread();

private:
	pthread_t imuPid;
};
#endif

