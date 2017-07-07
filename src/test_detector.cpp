#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

bool blue_ = true;
bool show_debug_images_ = true;
double roi_width_ = 960;
double roi_height_ = 200;
double roi_x_ = 0;
double roi_y_ = 350;
double min_integrated_value_ = 80;
double integral_window_size_ = 10;

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
    cv::waitKey(0);
  }
  return points2;
}

int main(int argc, char** argv) {
  cv::namedWindow("test", 0);
  Mat img = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  vector<Point2d> p = detect(img);
  return 0;
}