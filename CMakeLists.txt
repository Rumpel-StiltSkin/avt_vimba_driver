cmake_minimum_required(VERSION 2.8.3)
project(avt_vimba_driver)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  diagnostic_updater
  dynamic_reconfigure
  image_geometry
  image_transport
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  polled_camera
)

## CMAKE_MODULE_PATH sets path to search for CMake Modules
## CMAKE lacks a way of finding architecture of the system, thus include CMake module
## Arch store in variable "ARCH"

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#include(TargetArchitecture)
#target_architecture(ARCH)


## This is required to specify catkin-specific information to the build system 
## which in turn is used to generate pkg-config and CMake files.	

catkin_package(
  INCLUDE_DIRS include ## The exported include paths (i.e. cflags) for the package
)


## Prior to specifying targets, you need to specify where resources can be found 
## for said targets, specifically header files and libraries:

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(camera src/camera.cpp src/FrameObserver.cpp src/ApiController.cpp)

target_link_libraries(camera
  ${catkin_LIBRARIES}
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaC.so
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/arm_64bit/libVimbaCPP.so
)


## Install Executable & Header files

install(TARGETS camera 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

