
#ifndef __CAMERA_API__
#define __CAMERA_API__

#include <vector>
#include <iostream>
#include <fcntl.h>
#include "Dispatcher.h"
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <sys/ioctl.h>
#include <pthread.h>


#define CAM_NUM     (3)
#define ABS(x)   ((x) > 0 ? (x) : (-x)) 
#define TIMESTAMP_DIFF  (38)   // (20)

using namespace std;

// enum capstate
// {
//     CAP_IDLE = 0,
//     CAP_RUNNING,
//     CAP_DESTROY,
// };

class CameraAPI
{
private:
    int fd;
    
    char dev_name[10];

public:
    CameraAPI();
    ~CameraAPI();
    int camNum=CAM_NUM;
    int frame_timestamp[CAM_NUM];
    int camera_list[6] = {0,1,2, 3, 4, 5};
//     pthread_t capPid[CAM_NUM];
    Dispatcher &dispatcher = Dispatcher::getInstance();
//     enum capstate capThreadState;

    int CaptureFrame();
    void publishImage(int devIndex);
};

#endif
