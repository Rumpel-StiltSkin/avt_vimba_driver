#ifndef CAMERA_H
#define CAMERA_H

#include <avt_vimba_driver/RosVimbaApi.h>
#include <avt_vimba_driver/FrameObserver.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

namespace camera {
class camera {
    private:
	VmbErrorType CameraStart();	
};
}

#endif

