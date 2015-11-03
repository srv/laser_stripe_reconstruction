/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <laser_line_reconstruction/detector.h>

using namespace cv;
using namespace std;

Detector::Detector(ros::NodeHandle nh,
                   ros::NodeHandle nhp)
                 : nh_(nh), nhp_(nhp) {

  // detection parameters
  nhp_.param("peak_window_size", peak_window_size_, 5);
  nhp_.param("max_laser_width", max_laser_width_, 40);

  // Show debug images in another window
  nhp_.param("show_debug_images", show_debug_images_, false);

  ROS_INFO_STREAM("[Detector]: Parameters \n" <<
  "\t\t* peak_window_size:      " << peak_window_size_ << "\n" <<
  "\t\t* max_laser_width:       " << max_laser_width_ << "\n" <<
  "\t\t* show_debug_images:     " << show_debug_images_);
}

vector<Point2d> Detector::detect(const Mat& img) {
  // BGR images expected
  assert(img.channels() == 3);

  cv::Mat show_img;
  if (show_debug_images_) {
    show_img = img.clone();
  }

  // Prepare output vector
  vector<Point2d> points2;

  // Split the image in channels
  vector<Mat> channels(3);
  split(img, channels);
  Mat g(img.size(), CV_8UC1);
  Mat green = channels[1];

  double min, max;
  cv::Point min_idx, max_idx;

  // Substract red channel to green channel
  g = channels[1];  // - channels[2];
  cv::Mat g_double;
  g.convertTo(g_double, CV_32F);
  cv::minMaxLoc(g, &min, &max, &min_idx, &max_idx);
  g_double = g_double * 1.0/(max - min) - min * 1.0/(max - min);

  Mat d_double;
  Scharr(g_double, d_double, g_double.depth(),
         1,   // order of X derivative
         0);  // order of Y derivative
  Sobel(g_double, d_double, g_double.depth(), 1, 0, 5);
  cv::minMaxLoc(d_double, &min, &max, &min_idx, &max_idx);
  d_double = d_double * 1.0/(max - min) - min * 1.0/(max - min);
  show_img = d_double.clone();

  // Get max value per column
  for (size_t i = 0; i < d_double.rows; i++) {
    // Derivate column
    const float* Mi = d_double.ptr<float>(i);
    std::vector<float> row_i(Mi, Mi + d_double.cols);

    // Get max and min per column
    cv::minMaxLoc(row_i, &min, &max, &min_idx, &max_idx);
    // std::cout << "IDX = " << i
    //           << " Min is " << min
    //           << " at (" << min_idx.x
    //           << ", " << min_idx.y
    //           << ") Max is " << max
    //           << " at (" << max_idx.x
    //           << ", " << max_idx.y << ")\n" << std::endl;

    // Check distance between min and max
    int distance = (min_idx.x - max_idx.x);
    if (distance > 0 && distance < max_laser_width_) {
      // Use centre of mass
      int j = (max_idx.x + min_idx.x)/2;
      double peak_j = static_cast<double>(j);
      double m = 0;
      double mx = 0;
      for (int s = -peak_window_size_/2; s < (peak_window_size_+1)/2; s++) {
        const unsigned char val = green.at<unsigned char>(i, j+s);
        mx = val*s;
        m += val;
      }
      mx /= m;
      peak_j += mx;
      // If there's is green content in the pixel
      cv::Point p(peak_j, i);
      cv::Vec3b val = img.at<cv::Vec3b>(i, peak_j);
      if (val[1] >= val[0] && val[1] >= val[2] && val[1] >= 100) {
        points2.push_back(cv::Point2d(peak_j, i));
        if (show_debug_images_)
          cv::circle(show_img, cv::Point(peak_j, i), 5, cv::Scalar(0, 0, 255), 2);
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
