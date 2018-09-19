#include"ros/ros.h"
#include"visualization_msgs/Marker.h"
#include"geometry_msgs/Point.h"
#include"sensor_msgs/Image.h"
#include"cv_bridge/cv_bridge.h"
#include"sensor_msgs/image_encodings.h"
#include"image_transport/image_transport.h"
#include"opencv2/highgui/highgui.hpp"
#include <boost/graph/graph_concepts.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& img)
{
  try
  {
    cv::imshow("view",cv_bridge::toCvShare(img,"bgr8")->image);
    cv::waitKey(33);
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("could't convert %s to bgr8",img->encoding.c_str());
  }
}
void markerCallback(const visualization_msgs::Marker::ConstPtr& marker)
{
  ROS_INFO("\n\n");
  
  ROS_INFO("marker type is %d",marker->type);
  int points_size=marker->points.size();
  ROS_INFO("points size is %d",points_size);
  ROS_INFO("points[size-1] is %f",marker->points[points_size-1].x);
  
  ROS_INFO("\n\n");
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"ros_test_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ROS_INFO("ros_test_node running..."); 
  
  //image_transport::Subscriber sample
//   cv::namedWindow("view");
//   cv::startWindowThread();
//   image_transport::Subscriber camera_sub=it.subscribe("/camera/image",1,imageCallback);
//   ros::spin();
//   cv::destroyAllWindows();
  
  
  image_transport::Publisher camera_pub=it.advertise("/camera/image",1);
  cv::Mat img=cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
  sensor_msgs::ImageConstPtr camera_img;
  //camera_img->header=;
  //camera_img->width=;
  camera_img=cv_bridge::CvImage(std_msgs::Header(),"bgr8",img).toImageMsg();
  
  ros::Rate loop_rate(5);
  while(ros::ok())
  {
    camera_pub.publish(camera_img);
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  //ros::Subscriber marker_sub=nh.subscribe("/Chisel/full_mesh",1000,markerCallback);
  //ros::spin();
  
  return 0;
  
  
}