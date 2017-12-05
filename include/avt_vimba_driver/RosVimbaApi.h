#ifndef ROSVIMBAAPI_H
#define ROSVIMBAAPI_H

#include <VimbaCPP/Include/VimbaCPP.h>
#include <avt_vimba_driver/ApiController.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <string>
#include <map>

using AVT::VmbAPI::VimbaSystem;
using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;

namespace ROS {
namespace VmbAPI {

class RosVimbaApi {

public:

	VmbErrorType CameraStart()
	{
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
	            std::cout<<"Opening camera with ID: 192.168.2.2";

	            err = apiController.StartContinuousImageAcquisition();

	            if ( VmbErrorSuccess == err )
	            {
	                std::cout<< "Press <enter> to stop acquisition...\n" ;
	                getchar();

	                apiController.StopContinuousImageAcquisition();
	            }
	        }

	        apiController.ShutDown();
	    }

	    if ( VmbErrorSuccess == err )
	    {
	        std::cout<<"\nAcquisition stopped.\n" ;
	    }
	    else
	    {
		      std::cout<"\nerror.\n";
	    }
	    return err;
	}

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


};

}} // ROS::VmbAPI

#endif
