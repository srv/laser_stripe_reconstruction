/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef UWSIM_DETECTOR_H
#define UWSIM_DETECTOR_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>

class UWSimDetector {
 public:
  UWSimDetector(ros::NodeHandle nh, ros::NodeHandle nhp);
  std::vector<cv::Point2d> detect(const cv::Mat& img);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  int threshold_value_;
  bool show_debug_images_;
  int peak_window_size_;
  int max_laser_width_;
};

#endif  // DETECTOR_H
