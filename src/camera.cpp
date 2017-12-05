#include <avt_vimba_driver/camera.h>
#include <avt_vimba_driver/RosVimbaApi.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  VmbErrorType err = VmbErrorSuccess;

  ROS::VmbAPI::RosVimbaApi rosVimbaApi;
  err = rosVimbaApi.CameraStart();       
    if ( VmbErrorSuccess == err )
    {
    	std::cout << "error opening camera\n";
    }

  //FramePtr pFrame = ApiController.GetFrame(); 

  ros::Rate loop_rate(5);
  while (nh.ok()) {
  	ros::Time ros_time = ros::Time::now();
 	if (pub.getNumSubscribers() > 0) {
          sensor_msgs::Image img;
          std::cout << "spinning\n";
  //        if (rosVimbaApi.frameToImage(pFrame, img))
  //        {
  //          sensor_msgs::CameraInfo ci = pFrame->getCameraInfo();
  //          ci.header.stamp = img.header.stamp = ros_time;
  //          img.header.frame_id = ci.header.frame_id;
  //          pub.publish(img, ci);
  //        } else {
  //    ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
      }
          ros::spinOnce();
          loop_rate.sleep();
  }

}
