// Zihan Chen
// 2015-02-19
// Goal: test if triangulate and project3d id reversable


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
#if 0
  // load camera info
  std::cout << "camera" << std::endl;
  std::string cam_info_file = ros::package::getPath("vision_example");
  cam_info_file.append("/data/jhu_dvrk_left.ini");
  std::cout << "cam_info_file = " << cam_info_file << std::endl;

  std::string cam_name;
  sensor_msgs::CameraInfo cam_info_msg;
  camera_calibration_parsers::readCalibration(cam_info_file,
                                              cam_name,
                                              cam_info_msg);
  std::cout << "camera name = " << cam_name << std::endl;

  // image_geometry
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cam_info_msg);

  // print out some values
  std::cout << "cx: " << cam_model.cx() << "  cy: " << cam_model.cy() << "\n"
            << "fx: " << cam_model.fx() << "  fy: " << cam_model.fy() << "\n"
            << "Tx: " << cam_model.Tx() << "  Ty: " << cam_model.Ty() << "\n"
            << "bx: " << cam_model.binningX() << "  by: " << cam_model.binningY() << "\n"
            << "D: " << cam_model.distortionCoeffs() << "\n\n"
            << "K: \n" << cam_model.intrinsicMatrix() << "\n\n"
            << "P: \n" << cam_model.projectionMatrix() << "\n\n"
            << "R: \n" << cam_model.rotationMatrix() << "\n\n"
            << "TF: " << cam_model.tfFrame() << "\n\n";

  // rectify images
  cv::Point2d pixel;
  pixel = cam_model.project3dToPixel(cv::Point3d(0, 0.1, 0.3));
  std::cout << "pixel = " << pixel << std::endl;

  cv::Point2d pixel_raw;
  pixel_raw = cam_model.unrectifyPoint(pixel);
  std::cout << "pixel raw = " << pixel_raw << std::endl;

  cv::Point2d pixel_rect;
  pixel_rect = cam_model.rectifyPoint(pixel_raw);
  std::cout << "pixel rec = " << pixel_rect << std::endl;

  cv::Point3d ray;
  ray = cam_model.projectPixelTo3dRay(pixel);
  std::cout << "ray = " << ray << std::endl;
#endif

  // ----------------------------------------
  // camera model
  std::string ecm_path = ros::package::getPath("vision_example");
  std::string cam_info_file[2];
  std::string cam_name[2];
  sensor_msgs::CameraInfo cam_info_msg[2];
  image_geometry::PinholeCameraModel cam_model[2];

  cam_info_file[0] = ecm_path + "/data/jhu_dvrk_left.ini";
  cam_info_file[1] = ecm_path + "/data/jhu_dvrk_right.ini";

  for (size_t i = 0; i < 2; i++)
  {
    camera_calibration_parsers::readCalibration(cam_info_file[i],
                                                cam_name[i],
                                                cam_info_msg[i]);
    cam_model[i].fromCameraInfo(cam_info_msg[i]);
  }

  // -------- Test Project Points ----------yoyo
  // read img left + right
  // project 3d points to imgs
  // draw circle on left + right imgs
  cv::namedWindow("Left");
  cv::namedWindow("Right");

  cv::Mat img[2];
  img[0] = cv::imread("image_left.png");
  img[1] = cv::imread("image_right.png");

//  cam_model[0].rectifyImage(img[0], img_rect[0]);
//  cam_model[1].rectifyImage(img[1], img_rect[1]);

  cv::imshow("Left", img[0]);
  cv::imshow("Right", img[1]);
  cv::waitKey(0);

  // --------------------------------
  // detect chessboard
  cv::Size patternSize(11, 10);        // chessboard is 11 x 10
  std::vector<cv::Point2f> corners[2];    // result
  for (size_t i = 0; i < 2; i++) {
    bool found = cv::findChessboardCorners(img[i], patternSize, corners[i]);
    std::cout << "found = " << found << std::endl;
  }

#if 0
  // print some debug info
  std::cout << "size left  = " << corners[0].size() << std::endl;
  std::cout << "size right = " << corners[1].size() << std::endl;
  std::cout << corners[0].at(0) << std::endl;
  std::cout << corners[1].at(0) << std::endl;
  std::cout << cam_model[0].projectionMatrix() << std::endl;
  std::cout << cam_model[1].projectionMatrix() << std::endl;
#endif

  // convert data to cv::Mat
  cv::Mat projMatLeft = cam_model[0].projectionMatrix();
  cv::Mat projMatRight = cam_model[1].projectionMatrix();
  cv::Mat left_corners(2, corners[0].size(), CV_32F);
  cv::Mat right_corners(2, corners[1].size(), CV_32F);
  cv::Mat result(4, corners[0].size(), CV_32F);

  for (size_t i = 0; i < corners[0].size(); i++)
  {
    left_corners.at<float>(0, i) = corners[0].at(i).x;
    left_corners.at<float>(1, i) = corners[0].at(i).y;
    right_corners.at<float>(0, i) = corners[1].at(i).x;
    right_corners.at<float>(1, i) = corners[1].at(i).y;
  }

  // triangulation
  cv::triangulatePoints(projMatLeft, projMatRight,
                        left_corners, right_corners,
                        result);

//  std::cout << result << std::endl;

  // project 3d pts
//  int point_index = 10;
//  cv::Point3d point(result.at<float>(0, point_index)/result.at<float>(3, point_index),
//                    result.at<float>(1, point_index)/result.at<float>(3, point_index),
//                    result.at<float>(2, point_index)/result.at<float>(3, point_index));

//  std::cout << result.at<float>(0, point_index) << " "
//            << result.at<float>(1, point_index) << " "
//            << result.at<float>(2, point_index) << " "
//            << result.at<float>(3, point_index) << "\n";

  cv::Point3d point(0.0245145, -0.00835915,    0.144257);
  cv::Point2d img_pixel[2];
  img_pixel[0] = cam_model[0].project3dToPixel(point);
  img_pixel[1] = cam_model[1].project3dToPixel(point);

  std::cout << "point  = " << point << std::endl;
  std::cout << "pixel left  = " << img_pixel[0] << std::endl;
  std::cout << "pixel right = " << img_pixel[1] << std::endl;

  // draw points on image
  cv::circle(img[0], img_pixel[0], 3, cv::Scalar(0, 255, 0), 2);
  cv::circle(img[1], img_pixel[1], 3, cv::Scalar(0, 255, 0), 2);

  // show rectified imgs
  cv::imshow("Left", img[0]);
  cv::imshow("Right", img[1]);
  cv::waitKey(0);

  // clean up
  cv::destroyAllWindows();
  return 0;
}

