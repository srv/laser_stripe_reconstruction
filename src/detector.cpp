/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <laser_stripe_reconstruction/detector.h>

using namespace cv;
using namespace std;

Detector::Detector(ros::NodeHandle nh,
                   ros::NodeHandle nhp)
                 : nh_(nh), nhp_(nhp) {

  // detection parameters
  nhp_.param("integral_window_size", integral_window_size_, 5);
  nhp_.param("blue", blue_, false);
  nhp_.param("min_integrated_value", min_integrated_value_, 40);
  nhp_.param("roi_x", roi_x_, 0);
  nhp_.param("roi_y", roi_y_, 0);
  nhp_.param("roi_width", roi_width_, 0);
  nhp_.param("roi_height", roi_height_, 0);
  nhp_.param("integral_distance_value", integral_distance_value_, 1.0);
  nhp_.param("max_allowed_jump", max_allowed_jump_, 3);
  nhp_.param("max_jump_idx_distance", max_jump_idx_distance_, 4.0);

  // Show debug images in another window
  nhp_.param("show_debug_images", show_debug_images_, false);

  ROS_INFO_STREAM("[Detector]: Parameters \n" <<
  "\t\t* integral_window_size:  " << integral_window_size_ << "\n" <<
  "\t\t* min_integrated_value:  " << min_integrated_value_ << "\n" <<
  "\t\t* show_debug_images:     " << show_debug_images_ << "\n" <<
  "\t\t* roi_x:                 " << roi_x_ << "\n" <<
  "\t\t* roi_y:                 " << roi_y_ << "\n" <<
  "\t\t* roi_width:             " << roi_width_ << "\n" <<
  "\t\t* roi_height:            " << roi_height_
  );
}

vector<Point2d> Detector::detect(const Mat& img) {
  // BGR images expected
  assert(img.channels() == 3);
  cv::Mat show_img;
  if (show_debug_images_) {
    show_img = img.clone();
  }

  // Check ROI is OK
  if (roi_height_ == 0)
    roi_height_ = img.rows;

  if (roi_width_ == 0)
    roi_width_ = img.cols;

  // Prepare output vector
  vector<Point2d> points2;
  vector<pair<Point2d, bool> > points2_detections;

  // Split the image in channels
  vector<Mat> channels(3);
  split(img, channels);
  Mat laser_channel(img.size(), CV_8UC1);
  if (blue_)  laser_channel = channels[0];  // Blue
  if (!blue_) laser_channel = channels[1];  // Green

  // Substract red channel to green channel
  // green = channels[1];  // - channels[2];

  Mat laser_channel_roi = laser_channel(cv::Rect(roi_x_,
                                                 roi_y_,
                                                 roi_width_,
                                                 roi_height_));

  // Show ROI
  if (show_debug_images_) {
    if (roi_y_ > 0 && roi_width_ > 0 && roi_height_ > 0) {
      cv::rectangle(show_img,
                    cv::Point(roi_x_, roi_y_),
                    cv::Point(roi_x_ + roi_width_, roi_y_ + roi_height_),
                    cv::Scalar(255, 255, 255));
      cv::putText(show_img,
                  std::string("ROI"),
                  cv::Point(roi_x_, roi_y_ - 5),
                  cv::FONT_HERSHEY_DUPLEX,
                  2,  // font scale
                  cv::Scalar(255, 255, 255), 2);
    }
  }

  // Declare required vars
  double max_integrated_value, near_integrated_value, actual_integrated_value;
  double greenness_acc, greenness_distance_acc, weight;
  double max_integrated_value_idx;
  double iws = static_cast<double>(integral_window_size_);


  for (int u = 0; u < laser_channel_roi.cols;  u++) {
    max_integrated_value = 0;
    for (int vw = 0; vw < laser_channel_roi.rows - integral_window_size_; vw++) {
      actual_integrated_value = 0;
      greenness_acc = 0;
      greenness_distance_acc = 0;
      // Weighted integral for calculating the likelihood that the following
      // points are part of the laser line
      for (int v = vw; v < vw + integral_window_size_; v++) {
        weight = 1.0 - 2.0 * abs(vw + (iws - 1.0) / 2.0 - v) / iws;
        unsigned char val = laser_channel_roi.at<unsigned char>(v, u);
        double greenness = static_cast<double>(val);
        actual_integrated_value = weight*greenness + actual_integrated_value;
        greenness_acc = greenness + greenness_acc;
        greenness_distance_acc = (v-vw)*greenness + greenness_distance_acc;
      }
      if (actual_integrated_value > max_integrated_value) {
        // max_integrated_value:  highest integrated green value
        near_integrated_value = max_integrated_value;
        max_integrated_value = actual_integrated_value;
        // max_integrated_value_idx: row where max_integrated_value occurs
        max_integrated_value_idx = vw + (greenness_distance_acc/greenness_acc);
      }
    }
    // If 'true', there is a point in the current column,
    // which presumably belongs to the laser line
    cv::Point2d p2(u + roi_x_, max_integrated_value_idx + roi_y_);
    bool inlier = false;
    if (max_integrated_value > min_integrated_value_*integral_window_size_/2) {
      // Create the point and add to Map Points
      points2.push_back(p2);
      inlier = true;
      if (show_debug_images_) {
        double dist = max_integrated_value - near_integrated_value;
        if (dist < integral_distance_value_) {
          cv::circle(show_img, p2, 0, cv::Scalar(144, 238, 144), 1);
        } else {
          cv::circle(show_img, p2, 0, cv::Scalar(0, 255, 0), 1);
        }
      }
    } else {
      if (show_debug_images_) {
        cv::circle(show_img, p2, 0, cv::Scalar(0, 0, 255), 1);
      }
    }
    points2_detections.push_back(make_pair(p2, inlier));
  }

  for (int z = 0; z < 3; z++) {
    for (size_t i = 0; i < points2_detections.size() - 1; i++) {
      cv::Point2d p_act = points2_detections[i].first;
      bool inlier_act = points2_detections[i].second;

      for (size_t j = 1; j <= max_allowed_jump_; j++) {
        cv::Point2d p_next = points2_detections[i+j].first;
        bool inlier_next = points2_detections[i+j].second;

        if (inlier_act && !inlier_next) {
          double dist = abs(p_act.y - p_next.y);
          if (dist < max_jump_idx_distance_) {
            points2_detections[i+j].second = true;
            points2.push_back(p_next);
            cv::circle(show_img, p_next, 0, cv::Scalar(0, 255, 255), 1);
          }
        }
      }
    }

    for (size_t i = points2_detections.size() - 1; i > 0; i--) {
      cv::Point2d p_act = points2_detections[i].first;
      bool inlier_act = points2_detections[i].second;

      for (size_t j = 1; j <= max_allowed_jump_; j++) {
        cv::Point2d p_next = points2_detections[i-j].first;
        bool inlier_next = points2_detections[i-j].second;

        if (inlier_act && !inlier_next) {
          double dist = abs(p_act.y - p_next.y);
          if (dist < max_jump_idx_distance_) {
            points2_detections[i-j].second = true;
            points2.push_back(p_next);
            cv::circle(show_img, p_next, 0, cv::Scalar(0, 255, 255), 1);
          }
        }
      }
    }
  }

  if (show_debug_images_) {
    cv::namedWindow("Laser", 0);
    cv::imshow("Laser", show_img);
    cv::waitKey(3);
  }
  return points2;
}
