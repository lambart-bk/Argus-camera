
#include "CameraAPI.h"

#include"ros/ros.h"
#include"image_transport/image_transport.h"

#define FRAMECOUNT      (200)
using namespace std;
int main(int argc, char * argv[])
{
    ros::init(argc,argv,"camera_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ROS_INFO("camera_node running..."); 
    
    image_transport::Publisher camera_pub0=it.advertise("/camera/image0",1);
    image_transport::Publisher camera_pub1=it.advertise("/camera/image1",1);
    image_transport::Publisher camera_pub2=it.advertise("/camera/image2",1);
  
  
//     int i = FRAMECOUNT;
//     int temp[6]={0,0,0,0,0,0};
    CameraAPI camapi;
    //IMUThread imut(20); // run 20s

    while(ros::ok())
    {
	camapi.CaptureFrame();
	camapi.publishImage(0,camera_pub0);
	camapi.publishImage(1,camera_pub1);
	camapi.publishImage(2,camera_pub2);

// 	if(i<(FRAMECOUNT-1))
// 	{
// 		for(int j=0;j<camapi.camNum;j++)
// 		{
// 			int diff=camapi.frame_timestamp[j]-temp[j];
// 			cout<<"\t\t\t\tdevice: "<<j<<" : "<<diff<<"ms\n";
// 		}	
// 		//cout<<"\n";
// 	}
// 
// 	for(int j=0;j<camapi.camNum;j++)
// 	{
// 		temp[j]=camapi.frame_timestamp[j];
// 	}	
	
	
    }
    ros::shutdown();
    ROS_INFO("camera_node end !"); 

    return 0;
}
