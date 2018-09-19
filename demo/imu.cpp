#include "imu.h"
#include <sys/time.h>

IMU::IMU()
{
}

IMU::~IMU()
{
}

int IMU::IMU_Open()
{
	fd = open(device, O_RDONLY);
	if(-1 == fd) {
		printf("IMU open error\n");
		return -1;
	}

	return 0;
}

int IMU::IMU_Close()
{
	if(fd != 0)
		close(fd);
}


int IMU::IMU_ReadAll(struct mpu9250_frame *data)
{
	return ioctl(fd, GET_FPGA, data);
}

void *imuThread(void *arg)
{
    class IMU imu;
    struct mpu9250_frame data;
    
    int fcnt = (*(int *)arg) * 200;

    imu.IMU_Open();

    while(fcnt--)
    {
        imu.IMU_ReadAll(&data);
#if 1
	struct mpu9250_data *d = (struct mpu9250_data *)&data;
	unsigned char *ptr = (unsigned char *)&data;
	printf("0x%x, 0x%x, %d\n", d->id0, d->id1, ((d->time[0] << 24) + (d->time[1] << 16) + (d->time[2] << 8) + d->time[3]) / 1000);
	//for(int i = 0; i < 27; i++)
	//	printf("0x%x ", ptr[i]);
	//printf("\n");
#endif

	usleep(5000);
    }

    imu.IMU_Close();

    return NULL;
}

IMUThread::IMUThread(int time)
{
    if(0 != pthread_create(&imuPid, NULL, imuThread, &time))
    {
        std::cout << "create imu thread error" << std::endl;
    }
}

IMUThread::~IMUThread()
{
    pthread_join(imuPid, NULL);
}


