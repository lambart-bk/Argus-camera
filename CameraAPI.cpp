
#include "CameraAPI.h"
#include"opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"

// void *capThread(void *arg)
// {
//     CameraAPI *pobj = (CameraAPI *)arg;
//     static int index = 0;
//     int camIndex = index;
// 
//     index++;
// 
//     while(pobj->capThreadState == CAP_IDLE)
//         usleep(1);
// 
//     while(pobj->capThreadState == CAP_RUNNING)
//     {
//         pobj->dispatcher.capture_frame(pobj->camera_list[camIndex]);
//     }
// 
//     return NULL;
// }

CameraAPI::CameraAPI()
{
    int i = 0;
//     capThreadState = CAP_IDLE;

    for(i = 0; i < CAM_NUM; i++)
    {
        dispatcher.createSession(camera_list[i]);

       /* if(0 != pthread_create(&capPid[i], NULL, capThread, this))
        {
            cout << "create cap thread error" << endl;
        }*/

        sleep(1);
    }

    //capThreadState = CAP_RUNNING;
}

CameraAPI::~CameraAPI()
{
    int i = 0;

    //capThreadState = CAP_DESTROY;

    for(i = 0; i < CAM_NUM; i++)
    {
	//pthread_join(capPid[i], NULL);
        dispatcher.destroySession(camera_list[i]);
    }

    dispatcher.shutdown();
}

int CameraAPI::CaptureFrame()
{
    int i;

    for(i = 0; i < CAM_NUM; i++)
    {
        frame_timestamp[i] = dispatcher.capture_frame(camera_list[i]); 
    }  
	
    usleep(16000);
    

    return 0;
}

void CameraAPI::publishImage(int devIndex)
{
    uint8_t *y=dispatcher.m_framedata[devIndex].data_y;
//     uint8_t *u=dispatcher.m_framedata[devIndex].data_u;
//     uint8_t *v=dispatcher.m_framedata[devIndex].data_v;
  
//     cv::Mat img_yuv420(480*3/2,752,CV_8UC1); 
    
    cv::Mat img_yuv420(480,752,CV_8UC1);   //just Y   
    //read line by line 
//     for(int i=0;i<480;i++)
//       for(int j=0;j<752;j++)
//       {
// 	int offset=j+i*768;     //Notes:stride=768
// 	//std::cout<<i<<","<<j<<std::endl;
// 	img_yuv420.at<unsigned char>(i,j)=y[offset];
// 	
//       }
    //memcopy ,maybe a litter fast than read directly
    cv::Mat img_map_raw(480,768,CV_8UC1);
    memcpy(img_map_raw.data,y,480*768*sizeof(uint8_t));
    img_map_raw.colRange(0,752).copyTo(img_yuv420);
    
      
//      cv::imshow("img",img_yuv420);
//      cv::waitKey(33);
}







