cmake_minimum_required(VERSION 2.8.3)
project(vision_example)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_proc
  roscpp
  rospy
  image_transport
  cv_bridge
  pcl_ros
  camera_calibration_parsers
  tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vision_example
#  CATKIN_DEPENDS image_proc roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(vision_example
#   src/${PROJECT_NAME}/vision_example.cpp
# )

## Declare a cpp executable
add_executable(calibrate_camera src/calibrate_camera.cpp)
target_link_libraries(calibrate_camera ${catkin_LIBRARIES})

#add_executable(register_psm_camera src/register_psm_camera.cpp)
#target_link_libraries(register_psm_camera ${catkin_LIBRARIES})

add_executable(test_camera src/test_camera.cpp)
target_link_libraries(test_camera ${catkin_LIBRARIES})

add_executable(test_stereo_camera src/test_stereo_camera.cpp)
target_link_libraries(test_stereo_camera ${catkin_LIBRARIES})




