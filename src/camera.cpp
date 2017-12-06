#include <avt_vimba_driver/RosVimbaApi.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using AVT::VmbAPI::FramePtr;

bool frameToImage(const FramePtr pFrame, sensor_msgs::Image& image) {
      VmbPixelFormatType pixel_format;
      VmbUint32_t width, height, nSize;
    
      pFrame->GetWidth(width);
      pFrame->GetHeight(height);
      pFrame->GetPixelFormat(pixel_format);
      pFrame->GetImageSize(nSize);
    
      VmbUint32_t step = nSize / height;
    
      // NOTE: YUV and ARGB formats not supported
      std::string encoding;
      if      (pixel_format == VmbPixelFormatMono8          ) encoding = sensor_msgs::image_encodings::MONO8;
      else if (pixel_format == VmbPixelFormatMono10         ) encoding = sensor_msgs::image_encodings::MONO16;
      else if (pixel_format == VmbPixelFormatMono12         ) encoding = sensor_msgs::image_encodings::MONO16;
      else if (pixel_format == VmbPixelFormatMono12Packed   ) encoding = sensor_msgs::image_encodings::MONO16;
      else if (pixel_format == VmbPixelFormatMono14         ) encoding = sensor_msgs::image_encodings::MONO16;
      else if (pixel_format == VmbPixelFormatMono16         ) encoding = sensor_msgs::image_encodings::MONO16;
      else if (pixel_format == VmbPixelFormatBayerGR8       ) encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
      else if (pixel_format == VmbPixelFormatBayerRG8       ) encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
      else if (pixel_format == VmbPixelFormatBayerGB8       ) encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
      else if (pixel_format == VmbPixelFormatBayerBG8       ) encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
      else if (pixel_format == VmbPixelFormatBayerGR10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerRG10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerGB10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerBG10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerGR12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerRG12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerGB12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerBG12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerGR12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
      else if (pixel_format == VmbPixelFormatBayerRG12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
      else if (pixel_format == VmbPixelFormatBayerGB12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
      else if (pixel_format == VmbPixelFormatBayerBG12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
      else if (pixel_format == VmbPixelFormatBayerGR16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerRG16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerGB16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatBayerBG16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
      else if (pixel_format == VmbPixelFormatRgb8           ) encoding = sensor_msgs::image_encodings::RGB8;
      else if (pixel_format == VmbPixelFormatBgr8           ) encoding = sensor_msgs::image_encodings::BGR8;
      else if (pixel_format == VmbPixelFormatRgba8          ) encoding = sensor_msgs::image_encodings::RGBA8;
      else if (pixel_format == VmbPixelFormatBgra8          ) encoding = sensor_msgs::image_encodings::BGRA8;
      else if (pixel_format == VmbPixelFormatRgb12          ) encoding = sensor_msgs::image_encodings::TYPE_16UC3;
      else if (pixel_format == VmbPixelFormatRgb16          ) encoding = sensor_msgs::image_encodings::TYPE_16UC3;
      else
        ROS_WARN("Received frame with unsupported pixel format %d", pixel_format);
      if (encoding == "") return false;
    
      VmbUchar_t *buffer_ptr;
      VmbErrorType err = pFrame->GetImage(buffer_ptr);
      bool res = false;
      if ( VmbErrorSuccess == err ) {
        res = sensor_msgs::fillImage(image,
                                     encoding,
                                     height,
                                     width,
                                     step,
                                     buffer_ptr);
      } else {
        ROS_ERROR_STREAM("[" << ros::this_node::getName()
          << "]: Could not GetImage. ");
      }
      return res;
}



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
  //ROS::VmbAPI::RosVimbaApi rosVimbaApi;  

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
      err = apiController.StartContinuousImageAcquisition();

	if ( VmbErrorSuccess != err)
	{
	  apiController.StopContinuousImageAcquisition();
	  apiController.ShutDown();
	}
     }
  }

  ros::Duration(0.5).sleep();  

  ros::Rate loop_rate(30);
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
