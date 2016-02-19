/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

/// ROS includes
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

/// PCL includes
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std;
using namespace pcl;

typedef PointCloud<PointXYZRGB> CloudRGB;
typedef PointCloud<PointXYZ> Cloud;

string parent, child;
Cloud::Ptr acc(new Cloud);
bool init = false;
ros::Publisher pub;
tf::TransformListener listener;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Get the transform
  tf::StampedTransform transform;
  try
  {
    listener.lookupTransform(parent, child, ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // Convert incoming message to PCL
  PCLPointCloud2 cloud_pc2;
  pcl_conversions::toPCL(*cloud_msg, cloud_pc2);
  Cloud::Ptr in_cloud(new Cloud);
  fromPCLPointCloud2(cloud_pc2,*in_cloud);

  // Apply the transform
  Eigen::Affine3d eigen_tf;
  tf::transformTFToEigen(transform, eigen_tf);
  pcl::transformPointCloud(*in_cloud, *in_cloud, eigen_tf);
  if (!init) {
    // On first time, just copy the content
    copyPointCloud(*in_cloud, *acc);
    init = true;
  } else {
    // Acumulate cloud
    *acc += *in_cloud;
  }
  if (pub.getNumSubscribers() > 0 && acc->points.size() > 0)
    pub.publish(acc);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_registration");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  // Get parent and child IDs for the TF
  nhp.param("parent", parent, string());
  nhp.param("child", child, string());

  // Setup ROS Publishers and subscriber
  pub = nhp.advertise<Cloud>("/output", 1);
  ros::Subscriber sub = nhp.subscribe("/input", 1, callback);

  ros::spin();
  return 0;
};
