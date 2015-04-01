/// Copyright 2015 Miquel Massot Campos
/// Systems, Robotics and Vision
/// University of the Balearic Islands
/// All rights reserved.

#include <laser_line_reconstruction/reconstructor.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_line_reconstruction_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  Reconstructor reconstructor(nh,nhp);
  ros::spin();
  return 0;
}
