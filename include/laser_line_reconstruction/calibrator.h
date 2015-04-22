/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class Calibrator {
 public:
  Calibrator(ros::NodeHandle nh, ros::NodeHandle nhp);
  void setCameraInfo(const sensor_msgs::CameraInfoConstPtr& cam_info);
  bool detectChessboard(const cv::Mat& img,
                        const std::vector<cv::Point2f>& points2d);
  void savePoints(const std::vector<double>& plane,
                  const std::vector<cv::Point2f>& points2d);
  void fitPlane();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  std::string calibration_filename_;
  double max_reproj_error_;
  int chessboard_squares_x_;
  int chessboard_squares_y_;
  double chessboard_size_;
  image_geometry::PinholeCameraModel cm_;
  std::vector<std::vector<double> > plane_pf_;
  std::vector<cv::Mat> points3d_pf_;
  std::vector<cv::Point2f> borders_;
  int images_taken_;
  cv::Mat show_img_;

  bool detectChessboardImpl(const cv::Mat& img,
                            int chessboard_squares_x,
                            int chessboard_squares_y,
                            double chessboard_size,
                            std::vector<double>& plane);
  std::vector<cv::Point2f> extractPoints(
    const std::vector<cv::Point2f> points, const cv::Mat& mask) ;
  cv::Mat removeChessboard(const cv::Mat& img);
  std::vector<cv::Point2f> projectPoints(
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const cv::Mat& camera_matrix , const cv::Mat& dist_coeffs);
  double computeReprojectionError(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const cv::Mat& camera_matrix , const cv::Mat& dist_coeffs);
  cv::Point3f intersectRay(const cv::Point2f& p,
                           const std::vector<double>& plane);
};

#endif  // CALIBRATOR_H
