#include <avt_vimba_driver/RosVimbaApi.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using AVT::VmbAPI::FramePtr;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  //VmbErrorType err = VmbErrorSuccess;

  //ROS::VmbAPI::RosVimbaApi rosVimbaApi;
  //err = rosVimbaApi.CameraStart();       
  //  if ( VmbErrorSuccess == err )
  //  {
  //  	std::cout << "error opening camera\n";
  //  }

	VmbErrorType err = VmbErrorSuccess;

	AVT::VmbAPI::Examples::ApiController apiController;
  ROS::VmbAPI::RosVimbaApi rosVimbaApi;        

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
      std::cout<<"Opening camera with ID: 192.168.2.2";
      SP_SET( m_pFrameObserver , new FrameObserver( m_pCamera ) );
      err = apiController.StartContinuousImageAcquisition();

	    if ( VmbErrorSuccess != err)
	    {
	    	apiController.StopContinuousImageAcquisition();
	    	apiController.ShutDown();
	    }
	  }
	}

  FramePtr pFrame = m_ApiController.GetFrame();
  if( SP_ISNULL( pFrame ) )
  {
      std::cout << "frame pointer is NULL, late frame ready message\n";
      return;
  }

  ros::Rate loop_rate(5);
  while (nh.ok()) {
  	ros::Time ros_time = ros::Time::now();
 		if (pub.getNumSubscribers() > 0) {
      sensor_msgs::Image img;
      std::cout << "spinning\n";

      //AVT::VmbAPI::Examples::FrameObserver frameObserver
      //const FramePtr pFrame;
      //pFrame = frameObserver.GetFrame();

      // FramePtr pFrame = apiController.GetFrame();

      // if (rosVimbaApi.frameToImage(SP_ACCESS(pFrame), img))
      // {
      //   sensor_msgs::CameraInfo ci = SP_ACCESS(pFrame)->getCameraInfo();
      //   ci.header.stamp = img.header.stamp = ros_time;
      //   img.header.frame_id = ci.header.frame_id;
      //   pub.publish(img, ci);
      // } else {
      // ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
      // }
    ros::spinOnce();
    loop_rate.sleep();
  	}

	}
}
