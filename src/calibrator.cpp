/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <laser_line_reconstruction/calibrator.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>
#include <string>

Calibrator::Calibrator(ros::NodeHandle nh,
                       ros::NodeHandle nhp):
                       nh_(nh), nhp_(nhp) {
  // Read the parameters from the parameter server (set defaults)
  nhp_.param("calibration_filename", calibration_filename_,
             std::string("calibration.yaml"));
  nhp_.param("chessboard_squares_x", chessboard_squares_x_, 8);
  nhp_.param("chessboard_squares_y", chessboard_squares_y_, 6);
  nhp_.param("chessboard_size", chessboard_size_, 0.04);
  nhp_.param("max_reproj_error", max_reproj_error_, 1.0);  // pixels
  images_taken_ = 0;
}

void Calibrator::setCameraInfo(
  const sensor_msgs::CameraInfoConstPtr& cam_info) {
  // method for external calls
  cm_.fromCameraInfo(cam_info);
}

bool Calibrator::detectChessboard(const cv::Mat& img,
                                  const std::vector<cv::Point2d>& points2d) {
  std::vector<double> plane;
  show_img_ = img.clone();
  bool success;
  ROS_INFO_STREAM("[Calibrator]: Looking for a " << chessboard_squares_x_
    << "x" << chessboard_squares_y_ << " chessboard calibration pattern...");
  success =  detectChessboardImpl(img, chessboard_squares_x_,
                                  chessboard_squares_y_,
                                  chessboard_size_,
                                  plane);
  if (success) {
    cv::Mat mask = removeChessboard(img);
    std::vector<cv::Point2d> filt_points2d;
    filt_points2d = extractPoints(points2d, mask);
    savePoints(plane, filt_points2d);
  }
  cv::namedWindow("Laser Calibration", 0);
  cv::imshow("Laser Calibration", show_img_);
  cv::waitKey(3);
  return success;
}

std::vector<cv::Point2d> Calibrator::extractPoints(
    const std::vector<cv::Point2d> points, const cv::Mat& mask) {
  std::vector<cv::Point2d> filt_points;
  for (size_t k = 0; k < points.size(); k++) {
    const unsigned char val = mask.at<unsigned char>(points[k].y, points[k].x);
    if (val>0) {
      filt_points.push_back(points[k]);
      // Draw point circles in the image
      cv::circle(show_img_, points[k], 5, cv::Scalar(0, 0, 255), 3);
    }
  }
  return filt_points;
}

cv::Mat Calibrator::removeChessboard(const cv::Mat& img) {
  cv::Mat mask = cv::Mat::zeros(img.size(), CV_8UC1);
  cv::Point points[1][4];
  for (size_t i = 0; i < 4; i++) {
    points[0][i] = borders_[i];
  }
  const cv::Point* ppt[1] = { points[0] };
  int npt[] = {4};
  cv::fillPoly(mask, ppt, npt, 1, cv::Scalar(255));
  // cv::namedWindow("gray", 0);
  // cv::imshow("gray", mask);
  // cv::waitKey(3);
  return mask;
}

void Calibrator::savePoints(const std::vector<double>& plane,
                            const std::vector<cv::Point2d>& points2d) {
  cv::Mat points3d;
  for (size_t i = 0; i < points2d.size(); i++) {
    cv::Point3d p = intersectRay(points2d[i], plane);
    cv::Mat p_mat = (cv::Mat_<double>(1, 3) << p.x, p.y, p.z);
    points3d.push_back(p_mat);
    points3d_pf_.push_back(p_mat);
  }
  if (images_taken_ > 4) {
    fitPlane();
  }
  images_taken_++;
}

void Calibrator::fitPlane() {
  ROS_INFO_STREAM("Plane fitting...");

  // Fit a plane for each line
  // substract the centroid from the point matrix
  cv::Mat centroid = cv::Mat::zeros(1, 3, CV_64FC1);
  size_t num_points = points3d_pf_.size();
  cv::Mat points(num_points, 3, CV_64FC1);
  for (size_t j = 0; j < num_points; j++) {
    centroid += points3d_pf_[j];
    // points3d_pf_[j].copyTo(points.row(j));
    for (size_t i = 0; i < 3; i++)
      points.at<double>(j, i) = points3d_pf_[j].at<double>(i);
  }
  centroid = centroid / static_cast<double>(num_points);
  ROS_INFO_STREAM("Centroid is in " << centroid);
  cv::Mat points_centered;
  points_centered = points - cv::Mat::ones(num_points, 1, CV_64FC1)*centroid;

  // Singular Value Decomposition
  ROS_INFO_STREAM("Running SVD...");
  cv::SVD svd;
  cv::Mat svd_u, svd_w, svd_vt;
  svd.compute(points_centered, svd_u, svd_w, svd_vt);

  // The eigenvector with the smallest eigenvalue is the normal of the plane
  // The solution is the last column of V
  cv::Mat svd_v;
  cv::transpose(svd_vt, svd_v);
  // zero based index. We want the third column
  cv::Mat normal = svd_v.col(svd_v.cols-1);

  // Calculate D
  double D = -1*(normal.at<double>(0, 0)*centroid.at<double>(0, 0)
            + normal.at<double>(1, 0)*centroid.at<double>(0, 1)
            + normal.at<double>(2, 0)*centroid.at<double>(0, 2));
  cv::Mat laser_plane(1, 4, CV_64FC1);
  laser_plane.at<double>(0) = normal.at<double>(0, 0);
  laser_plane.at<double>(1) = normal.at<double>(1, 0);
  laser_plane.at<double>(2) = normal.at<double>(2, 0);
  laser_plane.at<double>(3) = D;

  // Save the calibration to a file for later use
  // Default to ~/.ros/calibration.yaml
  ROS_INFO_STREAM("Saving the results to " << calibration_filename_);
  cv::FileStorage fs(calibration_filename_, cv::FileStorage::WRITE);
  time_t rawtime; time(&rawtime);
  fs << "calibration_date" << asctime(localtime(&rawtime));
  fs << "laser_plane" << laser_plane;
  fs << "centroid" << centroid;
  fs << "normal" << normal;
  fs << "points" << points;
  fs.release();
}

bool Calibrator::detectChessboardImpl(const cv::Mat& img,
                                      int chessboard_squares_x,
                                      int chessboard_squares_y,
                                      double chessboard_size,
                                      std::vector<double>& plane) {
  bool pattern_found = false;
  bool success = false;
  bool low_reprojection_error = false;

  // Generate the 3D points of the chessboard
  std::vector<cv::Point3d> chessboard_points;
  double cx = chessboard_size * chessboard_squares_x / 2;
  double cy = chessboard_size * chessboard_squares_y / 2;
  for (int i = 0; i < chessboard_squares_y; i++) {
    for (int j = 0; j < chessboard_squares_x; j++) {
      double x = j*chessboard_size - cx;
      double y = i*chessboard_size - cy;
      cv::Point3d p(x, y, 0.0);
      chessboard_points.push_back(p);
    }
  }

  // cornerSubPix only wants CV_8UC1 images
  cv::Mat gray;
  cv::Mat gray8u(img.size(), CV_8UC1);
  if (img.channels() > 2) {
    cv::cvtColor(img, gray, CV_BGR2GRAY);
  } else {
    gray = img;
  }
  gray.convertTo(gray8u, CV_8U);

  // Try to find the chessboard
  std::vector<cv::Point2f> corners;
  cv::Size pattern_size(chessboard_squares_x, chessboard_squares_y);
  ROS_WARN("[LaserCalibration:] findChessboardCorners");
  pattern_found = cv::findChessboardCorners(gray8u, pattern_size, corners);
  int vec_size = chessboard_squares_x*chessboard_squares_y;
  if (!pattern_found)
    ROS_WARN("[LaserCalibration:] Chessboard not detected");
  if (pattern_found && static_cast<int>(corners.size()) == vec_size) {
    // Refine
    ROS_WARN("[LaserCalibration:] cornerSubPix");
    cv::cornerSubPix(gray8u, corners, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    const cv::Mat K_prime(cm_.intrinsicMatrix());
    // Now solve the 2D-3D problem as usual
    cv::Mat rvec, tvec;
    ROS_WARN("[LaserCalibration:] solvePnP");
    success = cv::solvePnP(chessboard_points, corners,
                           K_prime, cv::Mat(), rvec, tvec);
    ROS_WARN("[LaserCalibration:] computeReprojectionError");
    double error = computeReprojectionError(chessboard_points, corners,
                                            rvec, tvec, K_prime, cv::Mat());
    low_reprojection_error = (error < max_reproj_error_);
    if (success && !low_reprojection_error) {
      ROS_WARN_STREAM("Reprojection error too high. " <<
                      "Max set at " << max_reproj_error_);
    } else if (success && low_reprojection_error) {
      // Get 3D plane
      cv::Mat rot(3, 3, CV_64FC1);
      ROS_WARN("[LaserCalibration:] Rodrigues");
      cv::Rodrigues(rvec, rot);
      // In a camera, Z is pointing out
      cv::Mat dir_vec = (cv::Mat_<double>(3, 1) << 0, 0, 1);
      dir_vec = rot*dir_vec;
      double A = dir_vec.at<double>(0);
      double B = dir_vec.at<double>(1);
      double C = dir_vec.at<double>(2);
      double D = dir_vec.dot(tvec);
      if (D*C > 0)
        D = -D;
      ROS_INFO_STREAM("Chessboard plane (A,B,C,D) = (" << A
                                               << ", " << B
                                               << ", " << C
                                               << ", " << D
                                               << ") with reproj error: "
                                               << error);
      // Store it
      plane.clear();
      plane.push_back(A);
      plane.push_back(B);
      plane.push_back(C);
      plane.push_back(D);

      // Draw chessboard circles in the image
      for (size_t i = 0; i < corners.size(); i++)
        cv::circle(show_img_, corners[i], 10, cv::Scalar(255, 0, 0), 3);

      // save the outer borders of the pattern for latter use
      borders_.clear();
      std::vector<cv::Point3d> borders3d;
      borders3d.push_back(cv::Point3d(- 0.275, - 0.225, 0));
      borders3d.push_back(cv::Point3d(- 0.275, + 0.175, 0));
      borders3d.push_back(cv::Point3d(+ 0.275, + 0.175, 0));
      borders3d.push_back(cv::Point3d(+ 0.275, - 0.225, 0));
      borders_ = projectPoints(borders3d, rvec, tvec, K_prime, cv::Mat());
    } else {
      ROS_WARN("[LaserCalibration:] SolvePnP could not find a valid tf.");
    }
  }
  return pattern_found && success && low_reprojection_error;
}

std::vector<cv::Point2d> Calibrator::projectPoints(
    const std::vector<cv::Point3d>& object_points,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const cv::Mat& camera_matrix , const cv::Mat& dist_coeffs){
  std::vector<cv::Point2d> image_points;
  cv::projectPoints(cv::Mat(object_points), rvec, tvec, camera_matrix,
                    dist_coeffs, image_points);
  return image_points;
}

double Calibrator::computeReprojectionError(
    const std::vector<cv::Point3d>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec, const cv::Mat& tvec,
    const cv::Mat& camera_matrix , const cv::Mat& dist_coeffs) {
  std::vector<cv::Point2d> image_points2;
  double err;
  // project
  ROS_WARN("[LaserCalibration:] projectPoints");
  cv::projectPoints(cv::Mat(object_points), rvec, tvec, camera_matrix,
                                       dist_coeffs, image_points2);
  ROS_WARN("[LaserCalibration:] difference");
  // difference
  cv::Mat image_points_mat(image_points);
  cv::Mat image_points_d;
  image_points_mat.convertTo(image_points_d, CV_64FC1);
  err = cv::norm(image_points_d, cv::Mat(image_points2), CV_L2);
  // calculate the arithmetical mean
  ROS_WARN("[LaserCalibration:] return");
  return std::sqrt(err*err/object_points.size());
}

cv::Point3d Calibrator::intersectRay(const cv::Point2d& p,
                                     const std::vector<double>& plane) {
  double t;
  double A = plane[0];
  double B = plane[1];
  double C = plane[2];
  double D = plane[3];
  t = A*(p.x-cm_.cx())/cm_.fx()+B*(p.y-cm_.cy())/cm_.fy()+C;
  t = -D/t;
  cv::Point3d q;
  q.x = (p.x-cm_.cx())/cm_.fx()*t;
  q.y = (p.y-cm_.cy())/cm_.fy()*t;
  q.z = t;
  return q;
}
