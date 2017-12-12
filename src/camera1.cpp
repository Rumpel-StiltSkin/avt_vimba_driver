#include <avt_vimba_driver/RosVimbaApi.h>
#include <avt_vimba_driver/ApiController.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using AVT::VmbAPI::FramePtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher1");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image1", 1);

  //VmbErrorType err = VmbErrorSuccess;

  //ROS::VmbAPI::RosVimbaApi rosVimbaApi;
  //err = rosVimbaApi.CameraStart();       
  //  if ( VmbErrorSuccess == err )
  //  {
  //  	std::cout << "error opening camera\n";
  //  }

  VmbErrorType err = VmbErrorSuccess;

  AVT::VmbAPI::Examples::ApiController apiController;

  // Startup Vimba
  err = apiController.StartUp();        
  if ( VmbErrorSuccess == err )
  {
    if(true)
    {
      AVT::VmbAPI::CameraPtrVector cameras = apiController.GetCameraList();
      if( cameras.empty() )
      {
          err = VmbErrorNotFound;
      }
    }
    if ( VmbErrorSuccess == err )
    {
      std::cout<<"Opening camera with ID: 192.168.2.2\n";
      err = apiController.StartContinuousImageAcquisition("192.168.3.2");

	if ( VmbErrorSuccess != err)
	{
	  apiController.StopContinuousImageAcquisition();
	  apiController.ShutDown();
	}
     }
  }

  ros::Duration(0.5).sleep();  

  ros::Rate loop_rate(10);
  while (nh.ok()) {
      ros::Time ros_time = ros::Time::now();
      // if (pub.getNumSubscribers() > 0) {
      if (true) {
        sensor_msgs::Image img;

        FramePtr pFrame = apiController.GetFrame();
	
	//VmbUint32_t nWidth = 0;
	//VmbErrorType res;
	//res = SP_ACCESS(pFrame)->GetWidth(nWidth);
	//if (VmbErrorSuccess == res) 
	//{
	//  std::cout << nWidth << std::endl;
        //} else {
        //  std::cout < "error\n";
        //} 

        if (frameToImage(pFrame, img))
        {
          //sensor_msgs::CameraInfo ci = pFrame->getCameraInfo();
          //ci.header.stamp = img.header.stamp = ros_time;
          //img.header.frame_id = ci.header.frame_id;
          pub.publish(img); //, ci);
        } else {
          ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
        }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
}
