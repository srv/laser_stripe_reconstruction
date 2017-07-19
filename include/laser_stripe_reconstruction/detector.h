/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef DETECTOR_H
#define DETECTOR_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

class Detector {
 public:
  Detector(ros::NodeHandle nh, ros::NodeHandle nhp);
  std::vector<cv::Point2d> detect(const cv::Mat& img);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  int roi_x_;
  int roi_y_;
  int roi_width_;
  int roi_height_;
  bool show_debug_images_;
  int integral_window_size_;
  int min_integrated_value_;
  bool blue_;

  double integral_distance_value_;
  int max_allowed_jump_;
  double max_jump_idx_distance_;
};

#endif  // DETECTOR_H
