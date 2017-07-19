#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

bool blue_ = true;
bool show_debug_images_ = true;
double roi_width_ = 960;
double roi_height_ = 720;
double roi_x_ = 0;
double roi_y_ = 0;
double min_integrated_value_ = 5;
double integral_window_size_ = 10;
double integral_distance_value_ = 1;
int max_allowed_jump_ = 3;
double max_jump_idx_distance_ = 4.0;

vector<Point2d> detect(const Mat& img) {
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
  if (blue_)  laser_channel = channels[0];
  if (!blue_) laser_channel = channels[1];

  double min, max;
  cv::Point min_idx, max_idx;

  // Substract red channel to green channel
  // green = channels[1];  // - channels[2];
  Mat laser_channel_roi = laser_channel(cv::Rect(roi_x_, roi_y_, roi_width_, roi_height_));

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

    std::cout << max_integrated_value << ", ";

    if (max_integrated_value > min_integrated_value_*integral_window_size_/2) {
      // Create the point and add to Map Points
      points2.push_back(p2);
      inlier = true;
      if (show_debug_images_) {
        double dist = max_integrated_value - near_integrated_value;
        if (dist < integral_distance_value_) {
          cv::circle(show_img, p2, 0, cv::Scalar(0, 255, 255), 1);
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
            cv::circle(show_img, p_next, 0, cv::Scalar(255, 0, 255), 1);
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
            cv::circle(show_img, p_next, 0, cv::Scalar(255, 255, 255), 1);
          }
        }
      }
    }
  }



  std::cout << "\n\nCOUNT: " << points2.size() << std::endl;

  if (show_debug_images_) {
    cv::namedWindow("Laser", 0);
    cv::imshow("Laser", show_img);
    cv::waitKey(0);
  }
  return points2;
}

int main(int argc, char** argv) {
  Mat img = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  vector<Point2d> p = detect(img);
  return 0;
}