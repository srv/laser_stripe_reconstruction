Laser line reconstruction
=========================

This ROS package subscribes to a camera image and camera info topics and detects
a laser line in the image. With the tf provided, the laser points are triangulated
in 3D space.

A pointcloud aggregator then concatenates all pointclouds in their corresponding
position in the world to build complete 3D scenes.