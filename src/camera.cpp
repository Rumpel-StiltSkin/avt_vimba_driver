#include <avt_vimba_driver/camera.h>

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

  err = ROS::VmbAPI::CameraStart();       
    if ( VmbErrorSuccess == err )
    {
    	std::cout << "error opening camera\n";
    }
  
  ros::Rate loop_rate(5);
  while (nh.ok()) {
  	ros::Time ros_time = ros::Time::now();
  	if (pub_.getNumSubscribers() > 0) {
	  sensor_msgs::Image img;
	  if (ROS::VmbAPI::frameToImage(vimba_frame_ptr, img))
	  {
	    sensor_msgs::CameraInfo ci = info_man_->getCameraInfo();
	    ci.header.stamp = img.header.stamp = ros_time;
	    img.header.frame_id = ci.header.frame_id;
	    pub.publish(img, ci);
	  } else {
      ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
      }
	  ros::spinOnce();
	  loop_rate.sleep();
	}

  }
}