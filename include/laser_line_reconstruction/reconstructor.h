/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#ifndef RECONSTRUCTOR_H
#define RECONSTRUCTOR_H

#include <laser_line_reconstruction/detector.h>
#include <laser_line_reconstruction/triangulator.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Reconstructor {
 public:
  Reconstructor(ros::NodeHandle nh, ros::NodeHandle nhp);
 private:
  Detector* detector_;
  Triangulator* triangulator_;

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_;
  ros::Publisher point_cloud_pub_;

  std::string camera_frame_id_;

  void publishPoints(const std::vector<cv::Point3d>& points,
                     const ros::Time& stamp);
  void imageCallback(
    const sensor_msgs::ImageConstPtr      &image_msg,
    const sensor_msgs::CameraInfoConstPtr &info_msg);
};

#endif  // RECONSTRUCTOR_H
