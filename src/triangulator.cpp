/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <laser_line_reconstruction/triangulator.h>
#include <vector>

using namespace cv;
using namespace std;

Triangulator::Triangulator(ros::NodeHandle nh,
                           ros::NodeHandle nhp)
                         : nh_(nh), nhp_(nhp) {
  nhp_.param("camera_frame_id", camera_frame_id_, std::string("/camera"));
  nhp_.param("laser_frame_id", laser_frame_id_, std::string("/laser"));

  string calibration_filename;
  nhp_.param("laser_calibration",
             calibration_filename,
             std::string("/home/miquel/.ros/calibration.yaml") );

  FileStorage fs;
  fs.open(calibration_filename, FileStorage::READ);
  fs["laser_plane"] >> laser_plane_;
  is_cm_init_ = false;

  ROS_INFO_STREAM("[Triangulator]: Parameters \n" <<
  "\t\t* Calibration filename:     " << calibration_filename << "\n" <<
  "\t\t* Laser planes:             " << laser_plane_.rows << " found\n");
}

void Triangulator::setCameraInfo(const sensor_msgs::CameraInfoConstPtr& info_msg) {
  if (!is_cm_init_) {
    cm_.fromCameraInfo(info_msg);
    is_cm_init_ = true;
  }
}

void Triangulator::setCameraInfo(const sensor_msgs::CameraInfo& info_msg) {
  if (!is_cm_init_) {
    cm_.fromCameraInfo(info_msg);
    is_cm_init_ = true;
  }
}

void Triangulator::lookupLaserTransform(void) {
  string error_msg;
  ros::Time now = ros::Time::now();
  if (tf_listener_.canTransform(camera_frame_id_,
                                laser_frame_id_,
                                now,
                                &error_msg)) {
    tf_listener_.lookupTransform(camera_frame_id_,
                                 laser_frame_id_,
                                 now,
                                 laser_to_cam_);
  } else {
    ROS_WARN("The tf from '%s' to '%s' does not seem to be available, "
             "will assume it as identity!",
             camera_frame_id_.c_str(),
             laser_frame_id_.c_str());
    laser_to_cam_.setIdentity();
    ROS_DEBUG("Transform error: %s", error_msg.c_str());
  }
}

vector<Point3d> Triangulator::triangulate(const vector<Point2d>& points2d) {
  vector<Point3d>  points3d;
  if (is_cm_init_) {
    for (size_t i = 0; i < points2d.size(); i++) {
      Point3d ray, point3d;
      ray = cm_.projectPixelTo3dRay(points2d[i]);
      double t;
      t = (-laser_plane_.at<double>(3)) /
           (laser_plane_.at<double>(0) * ray.x +
            laser_plane_.at<double>(1) * ray.y +
            laser_plane_.at<double>(2));
      point3d = cv::Point3d(ray.x * t, ray.y * t, t);
      if (point3d.z > 0) {
        points3d.push_back(point3d);
      } else if (point3d.z == 0) {
        ROS_DEBUG("[Triangulator]: Skipping point with z = 0");
      } else {
        ROS_DEBUG("[Triangulator]: Skipping point with z < 0");
      }
    }
    if (points3d.size() < points2d.size())
      ROS_WARN_STREAM("[Triangulator]: Some points have been skipped ("
          << points3d.size() << "/" << points2d.size() << ")");
  } else {
    ROS_ERROR("[Triangulator]: Cannot triangulate. Set camera info first!");
  }
  return points3d;
}
