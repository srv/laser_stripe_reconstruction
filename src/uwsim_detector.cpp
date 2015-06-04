/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <laser_line_reconstruction/uwsim_detector.h>

using namespace cv;
using namespace std;

UWSimDetector::UWSimDetector(ros::NodeHandle nh,
                   ros::NodeHandle nhp)
                 : nh_(nh), nhp_(nhp) {

  // detection parameters
  nhp_.param("peak_window_size", peak_window_size_, 5);
  nhp_.param("max_laser_width", max_laser_width_, 40);
  nhp_.param("threshold_value", threshold_value_, 40);

  // Show debug images in another window
  nhp_.param("show_debug_images", show_debug_images_, false);

  ROS_INFO_STREAM("[UWSimDetector]: Parameters \n" <<
  "\t\t* threshold_value:       " << threshold_value_ << "\n" <<
  "\t\t* peak_window_size:      " << peak_window_size_ << "\n" <<
  "\t\t* max_laser_width:       " << max_laser_width_ << "\n" <<
  "\t\t* show_debug_images:     " << show_debug_images_);
}

vector<Point2d> UWSimDetector::detect(const Mat& img) {
  // BGR images expected
  assert(img.channels() == 3);

  cv::Mat show_img, show_im2;
  if (show_debug_images_) {
    show_img = img.clone();
  }

  // Prepare output vector
  vector<Point2d> points2;

  // Split the image in channels
  vector<Mat> channels(3);
  split(img, channels);
  Mat g = channels[1];

  double min, max;
  cv::Point min_idx, max_idx;

  // Threshold green channel
  cv::Mat th_g;
  cv::threshold(g, th_g, threshold_value_, 255, cv::THRESH_BINARY);
  for (size_t x = 0; x < g.cols; x++) {
    for (size_t y = 0; y < g.rows; y++) {
      unsigned char c = th_g.at<unsigned char>(y,x);
      if (c > 0) {
        double m = 0;
        double mx = 0;
        for (int s = -peak_window_size_/2; s < (peak_window_size_+1)/2; s++) {
          const unsigned char val = g.at<unsigned char>(y + s, x);
          mx = val*s;
          m += val;
        }
        points2.push_back(cv::Point2d(x, y + mx/m));
        if (show_debug_images_)
          cv::circle(show_img, cv::Point(x, y + mx/m), 5, cv::Scalar(0, 0, 255), 2);
        break;
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
