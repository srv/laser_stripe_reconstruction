cmake_minimum_required(VERSION 2.8.3)
project(laser_line_reconstruction)

find_package(catkin REQUIRED COMPONENTS
  camera_calibration_parsers
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  tf
  tf_conversions
  pcl_conversions
  std_srvs
)

## System dependencies
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES laser_line_reconstruction
  CATKIN_DEPENDS camera_calibration_parsers std_srvs cv_bridge geometry_msgs image_geometry image_transport pcl_ros roscpp rospy sensor_msgs tf
  DEPENDS OpenCV PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(laser_reconstruction src/node.cpp
                                    src/triangulator.cpp
                                    src/detector.cpp
                                    src/uwsim_detector.cpp
                                    src/calibrator.cpp
                                    src/reconstructor.cpp)

add_executable(mover src/mover.cpp)
target_link_libraries(mover
  ${catkin_LIBRARIES}
)

add_executable(laser_registration src/registration.cpp)
target_link_libraries(laser_registration
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(laser_reconstruction
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS laser_reconstruction
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  launch/uwsim_laser.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

