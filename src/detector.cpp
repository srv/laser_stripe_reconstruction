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

  // Split the image in channels
  vector<Mat> channels(3);
  split(img, channels);
  Mat green(img.size(), CV_8UC1);
  if (blue_)  green = channels[2];
  if (!blue_) green = channels[1];

  double min, max;
  cv::Point min_idx, max_idx;

  // Substract red channel to green channel
  // green = channels[1];  // - channels[2];

  Mat green_roi = green(cv::Rect(roi_x_, roi_y_, roi_width_, roi_height_));

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
  double max_integrated_value, actual_integrated_value;
  double greenness_acc, greenness_distance_acc, weight;
  double max_integrated_value_idx;
  double iws = static_cast<double>(integral_window_size_);

  for (int u = 0; u < green_roi.size().width;  u++) {
    max_integrated_value = 0;
    for (int vw = 0; vw < green_roi.rows - integral_window_size_; vw++) {
      actual_integrated_value = 0;
      greenness_acc = 0;
      greenness_distance_acc = 0;
      // Weighted integral for calculating the likelihood that the following
      // points are part of the laser line
      for (int v = vw; v < vw + integral_window_size_; v++) {
        weight = 1.0 - 2.0 * abs(vw + (iws - 1.0) / 2.0 - v) / iws;
        const int greenness = green_roi.at<unsigned char>(v, u);
        actual_integrated_value += weight*static_cast<double>(greenness);
        greenness_acc += greenness;
        greenness_distance_acc += (v-vw)*greenness;
      }
      if (actual_integrated_value > max_integrated_value) {
        // max_integrated_value:  highest integrated green value
        max_integrated_value = actual_integrated_value;
        // max_integrated_value_idx: row where max_integrated_value occurs
        max_integrated_value_idx = vw + (greenness_distance_acc/greenness_acc);
      }
    }
    // If 'true', there is a point in the current column,
    // which presumably belongs to the laser line
    if (max_integrated_value > min_integrated_value_*integral_window_size_/2) {
      // Create the point and add to Map Points
      cv::Point2d p2(u + roi_x_, max_integrated_value_idx + roi_y_);
      points2.push_back(p2);
      if (show_debug_images_) {
        cv::circle(show_img, p2, 1, cv::Scalar(0, 0, 255), 1);
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
