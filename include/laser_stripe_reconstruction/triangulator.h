/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef TRIANGULATOR_H
#define TRIANGULATOR_H

#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <vector>

class Triangulator {
 public:
  Triangulator(ros::NodeHandle nh, ros::NodeHandle nhp);
  void setCameraInfo(const sensor_msgs::CameraInfoConstPtr& info_msg);
  void setCameraInfo(const sensor_msgs::CameraInfo& info_msg);
  std::vector<cv::Point3d> triangulate(const std::vector<cv::Point2d>& points2d);
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  tf::StampedTransform laser_to_cam_;
  tf::TransformListener tf_listener_;
  cv::Mat laser_plane_;
  bool is_cm_init_;
  std::string camera_frame_id_;
  std::string laser_frame_id_;
  image_geometry::PinholeCameraModel cm_;
  void lookupLaserTransform(void);
};

#endif  // TRIANGULATOR_H
