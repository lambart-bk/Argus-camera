
#include "CameraAPI.h"
#include"opencv2/core/core.hpp"
#include"opencv2/highgui/highgui.hpp"
#include"opencv2/imgproc/imgproc.hpp"

#define debug false


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
#if debug
    cout << "CaptureFrame..." << endl;
#endif
    for(i = 0; i < CAM_NUM; i++)
    {
        frame_timestamp[i] = dispatcher.capture_frame(camera_list[i]); 
    }  
	
    usleep(16000);
#if debug
    cout << "CaptureFrame end" << endl;
#endif
    return 0;
}

void CameraAPI::publishImage(int devIndex,image_transport::Publisher &Publisher)
{
#if debug
    cout << "publishImage ..." << endl;
#endif
     uint8_t *y=dispatcher.m_framedata[devIndex].data_y;   
    // uint8_t *u=dispatcher.m_framedata[devIndex].data_u;
   //  uint8_t *v=dispatcher.m_framedata[devIndex].data_v;
    
    cv::Mat img_yuv420Y(480,752,CV_8UC1);   //just Y   
   // cv::Mat img_yuv420(480*3/2,752,CV_8UC1); //YUV
    //read line by line 
 /*    for(int i=0;i<480;i++)
       for(int j=0;j<752;j++)
       {
 	int offset=j+i*768;     //Notes:stride=768
 	std::cout<<i<<","<<j<<std::endl;
 	img_yuv420Y.at<unsigned char>(i,j)=y[offset];
 	
       }*/

    //memcopy ,maybe a litter fast than read directly
    cv::Mat img_map_raw(480,768,CV_8UC1);
    memcpy(img_map_raw.data,y,480*768*sizeof(uint8_t));
    img_map_raw.colRange(0,752).copyTo(img_yuv420Y);
    
    /*cv::Mat img_u(240,376,CV_8UC1);std::cout<<"4"<<std::endl;
    memcpy(img_u.data,u,240*376*sizeof(uint8_t));std::cout<<"1"<<std::endl;
    cv::Mat img_v(240,376,CV_8UC1);
    memcpy(img_v.data,v,240*376*sizeof(uint8_t));

    img_yuv420Y.copyTo(img_yuv420.rowRange(0,480));
    img_u.copyTo(img_yuv420.rowRange(480,720).colRange(0,376));std::cout<<"2"<<std::endl;
    img_v.copyTo(img_yuv420.rowRange(480,720).colRange(377,752));std::cout<<"3"<<std::endl;
    cv::Mat bgr;
    cvtColor(img_yuv420,bgr,CV_YUV2BGR_I420);*/

 /*   size_t sz=480*768*sizeof(uint8_t);
    memcpy(img_map_raw.data,y,sz);
    memcpy(img_map_raw.data+sz,u,240*376*sizeof(uint8_t));
    memcpy(img_map_raw.data+sz*5/4,v,240*376*sizeof(uint8_t));
    img_map_raw.colRange(0,752).copyTo(img_yuv420);
    cv::Mat bgr;
    cvtColor(img_yuv420,bgr,CV_YUV2BGR_I420);*/

    sensor_msgs::ImageConstPtr camera_img;
    try
    {
      camera_img=cv_bridge::CvImage(std_msgs::Header(),"mono8",img_yuv420Y).toImageMsg();
      //camera_img=cv_bridge::CvImage(std_msgs::Header(),"bgr8",bgr).toImageMsg();
    }
    catch(cv_bridge::Exception &e)
    {
      std::cout<<"cv_bridge::CvImage failed \n\t"<<e.what()<<std::endl;
    }
   
    Publisher.publish(camera_img);
    ros::spinOnce();
      
//      cv::imshow("img",img_yuv420);
//      cv::waitKey(0);
#if debug
    cout << "publishImage end" << endl;
#endif
}







