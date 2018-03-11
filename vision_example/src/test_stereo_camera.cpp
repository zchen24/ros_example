// Zihan Chen
// 2015-02-19
// Goal: test if triangulate and project3d id reversable
//
// Test 1: cv::triangulatePoints and stereo_cam.projectDisparityTo3d
//  -- conclusion: yes they do exactly the same thing

// Test 2:
//  -- use cv::triangulate points to get points in 3D
//  -- use project3dToPixel to project 3D point back to left & right images
//  -- verify whether they match

// Test 3:
//  -- test with DVRK robot

#include <iostream>

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

// opencv
#include <opencv2/highgui/highgui.hpp>

// pcl
#include <pcl/io/pcd_io.h>

int main()
{
  // ----------------------------------------
  // camera model
  std::string cam_info_file[2];
  std::string cam_name[2];
  sensor_msgs::CameraInfo cam_info_msg[2];
  image_geometry::PinholeCameraModel cam_model[2];
  image_geometry::StereoCameraModel cam_stereo_model;

  cam_info_file[0] = "jhu_dvrk_left.ini";
  cam_info_file[1] = "jhu_dvrk_right.ini";

  for (size_t i = 0; i < 2; i++)
  {
    camera_calibration_parsers::readCalibration(cam_info_file[i],
                                                cam_name[i],
                                                cam_info_msg[i]);
    cam_model[i].fromCameraInfo(cam_info_msg[i]);
  }

  // stereo camera model
  cam_stereo_model.fromCameraInfo(cam_info_msg[0], cam_info_msg[1]);

  // baseline
  std::cout << "baseline = " << cam_stereo_model.baseline() << std::endl;
  std::cout << "tf frame = " << cam_stereo_model.tfFrame() << std::endl;

  // getZ
  std::cout << "disparity = " << cam_stereo_model.getDisparity(0.1) << std::endl;
  std::cout << "left.cx = " << cam_model[0].cx() << "  right.cx = " << cam_model[1].cx() << std::endl;


  std::vector<cv::Point2d> corners[2];
  corners[0].push_back(cv::Point2d(110, 150));
  corners[1].push_back(cv::Point2d(100, 150));

  // Option 1: cv triangulate
  cv::Mat result(4, corners[0].size(), CV_32F);
  cv::triangulatePoints(cam_model[0].projectionMatrix(),
                        cam_model[1].projectionMatrix(),
                        corners[0], corners[1], result);
  int point_index = 0;
  cv::Point3d point(result.at<double>(0, point_index)/result.at<double>(3, point_index),
                    result.at<double>(1, point_index)/result.at<double>(3, point_index),
                    result.at<double>(2, point_index)/result.at<double>(3, point_index));
  std::cout << "point  = " << point << std::endl;

  // Option 2: disparity
  cv::Point3d point_ros;
  cam_stereo_model.projectDisparityTo3d(corners[0].at(0),
                                        10,
                                        point_ros);
  std::cout << "point  = " << point_ros << std::endl;

  return 0;
}

