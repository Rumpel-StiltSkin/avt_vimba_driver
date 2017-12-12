#ifndef ROSVIMBAAPI_H
#define ROSVIMBAAPI_H

#include <VimbaCPP/Include/VimbaCPP.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

using AVT::VmbAPI::VimbaSystem;
using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;

bool frameToImage(const FramePtr pFrame, sensor_msgs::Image& image); 


//namespace ROS {
//namespace VmbAPI {

//class RosVimbaApi
//{
//  public:
//    RosVimbaApi();
//    void frameToImage(const FramePtr pFrame, sensor_msgs::Image& image); 
//
//};
//
//}} // ROS::VmbAPI

#endif
