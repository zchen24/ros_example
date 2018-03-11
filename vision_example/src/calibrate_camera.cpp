// Zihan Chen
// 2015-02-19

#include <iostream>
#include <string>

// ros
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_geometry/stereo_camera_model.h>

// pcl
#include <pcl/io/pcd_io.h>


static image_geometry::PinholeCameraModel cam_model[2];
static image_geometry::StereoCameraModel stereo_cam_model;
static cv::Mat imgs_detected[2];
static bool imgs_corners_found[2];
static std::vector<cv::Point2f> corners_detected[2];

void triangulate_corners()
{
  // projection matrix from camera_info
  cv::Mat projMatLeft = cam_model[0].projectionMatrix();
  cv::Mat projMatRight = cam_model[1].projectionMatrix();

  // convert data to cv::Mat
  cv::Mat left_corners(2, corners_detected[0].size(), CV_32F);
  cv::Mat right_corners(2, corners_detected[1].size(), CV_32F);
  cv::Mat result(4, corners_detected[0].size(), CV_32F);

  for (size_t i = 0; i < corners_detected[0].size(); i++)
  {
    left_corners.at<float>(0, i) = corners_detected[0].at(i).x;
    left_corners.at<float>(1, i) = corners_detected[0].at(i).y;
    right_corners.at<float>(0, i) = corners_detected[1].at(i).x;
    right_corners.at<float>(1, i) = corners_detected[1].at(i).y;
  }

  // triangulation
  cv::triangulatePoints(projMatLeft, projMatRight,
                        left_corners, right_corners,
                        result);

  // pick a bunch of points
  std::vector<int> indexes;
  indexes.push_back(13);
  indexes.push_back(20);
  indexes.push_back(46);
  indexes.push_back(53);
  indexes.push_back(90);
  indexes.push_back(97);

  // output result as pcd files
  pcl::PointCloud<pcl::PointXYZ> resultCloud;
  for (size_t i = 0; i < indexes.size(); i++)
  {
    resultCloud.push_back(pcl::PointXYZ(result.at<float>(0, indexes.at(i))/result.at<float>(3,indexes.at(i)),
                                        result.at<float>(1, indexes.at(i))/result.at<float>(3,indexes.at(i)),
                                        result.at<float>(2, indexes.at(i))/result.at<float>(3,indexes.at(i))));
  }

  pcl::io::savePCDFileASCII ("camera_points.pcd", resultCloud);
  std::cerr << "Saved " << resultCloud.points.size () << " data points to camera_points.pcd." << std::endl;

#if 0
  // draw circles
  for (size_t i = 0; i < indexes.size(); i++)
  {
    cv::Point2f center0(corners_detected[0].at(indexes.at(i)).x,
                     corners_detected[0].at(indexes.at(i)).y);
    std::stringstream ss;  ss << i;
    cv::circle(imgs_detected[0], center0, 3, cv::Scalar(255, 0, 0), 2);
    cv::putText(imgs_detected[0], ss.str(), center0, cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);

    cv::Point2f center1(corners_detected[1].at(indexes.at(i)).x,
                     corners_detected[1].at(indexes.at(i)).y);
    cv::circle(imgs_detected[1], center1, 3, cv::Scalar(255, 0, 0), 2);
  }
#endif

  // display in cv window
  cv::imshow("view", imgs_detected[0]);
  cv::waitKey();

  // cv triangulate
  ROS_INFO("Both Cameras Found Chessboard");
  ros::shutdown();

  // save images
  cv::imwrite("image_left.png", imgs_detected[0]);
  cv::imwrite("image_right.png", imgs_detected[1]);
}


void image_callback(const sensor_msgs::ImageConstPtr& msg,
                    const sensor_msgs::CameraInfoConstPtr& msg_info,
                    const std::string img_topic)
{
  // camera index
  //  - 0: left camera
  //  - 1: right camera

  int cam_index;
  std::string cam_name_left("left");
  std::string cam_name_right("right");
  if (img_topic.find(cam_name_left) != std::string::npos) {
    cam_index = 0;
  } else if (img_topic.find(cam_name_right) != std::string::npos) {
    cam_index = 1;
  } else {
    ROS_ERROR("Invalid camera topic");
    return;
  }

  // camera model
  cam_model[cam_index].fromCameraInfo(msg_info);
  try {
    cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

    // chessboard
    cv::Size patternSize(11, 10);        // chessboard is 11 x 10
    std::vector<cv::Point2f> corners;    // result
    bool found = cv::findChessboardCorners(img, patternSize, corners);

    if (found) {
      imgs_corners_found[cam_index] = found;
      imgs_detected[cam_index] = img.clone();
      corners_detected[cam_index] = corners;
    }

    // show image
    cv::drawChessboardCorners(img, patternSize, corners, found);
    cv::imshow("view", img);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR_STREAM("Could not convert from " << msg->encoding.c_str()
                     << " to 'bgr8'.");
  }

  if (imgs_corners_found[0] && imgs_corners_found[1]) {
    triangulate_corners();
  }
}


int main(int argc, char *argv[])
{
    std::cout << "hello world calibratec camera" << std::endl;

    ros::init(argc, argv, "calibrate_camera");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();

    // ros img
    std::string left_img_topic = "/stereo/left/image_rect_color";
    std::string right_img_topic = "/stereo/right/image_rect_color";

//    std::string left_img_topic = "/stereo/left/image_raw";
//    std::string right_img_topic = "/stereo/right/image_raw";

    image_transport::ImageTransport it(nh);
    image_transport::CameraSubscriber subLeft =
        it.subscribeCamera(left_img_topic, 1, boost::bind(image_callback, _1, _2, left_img_topic));
    image_transport::CameraSubscriber subRight =
        it.subscribeCamera(right_img_topic, 1, boost::bind(image_callback, _1, _2, right_img_topic));

    // chessboard detect
    imgs_corners_found[0] = false;
    imgs_corners_found[1] = false;

    ros::spin();
    cv::destroyAllWindows();
    return 0;
}


