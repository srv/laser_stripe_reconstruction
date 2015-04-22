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
  std::vector<cv::Point2f> detect(const cv::Mat& img);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  int roi_x_;
  int roi_y_;
  int roi_width_;
  int roi_height_;
  int threshold_value_;
  int closing_element_size_;
  int opening_element_size_;
  int min_value_threshold_;
  int contours_threshold_;
  bool show_debug_images_;
  int peak_window_size_;
  int max_laser_width_;
};

#endif  // DETECTOR_H
