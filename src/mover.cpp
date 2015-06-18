/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

double y = 0;
double speed_;

void poseCallback(const ros::TimerEvent&) {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0, y, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "arm_base", "arm_tool2"));
  y -= 0.01*speed_;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_mover");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  nhp.param("speed", speed_, 0.01);  // m/s
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), poseCallback);
  ros::spin();
  return 0;
};
