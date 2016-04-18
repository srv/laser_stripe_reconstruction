# Laser line reconstruction

This ROS package subscribes to a camera image and camera info topics and detects
a laser line in the image. With the tf provided, the laser points are triangulated
in 3D space.

A pointcloud aggregator then concatenates all pointclouds in their corresponding
position in the world to build complete 3D scenes.

## Installation

`roscd; cd ../src; git clone git@github.com:srv/laser_stripe_reconstruction.git`

## Usage

### How to reconstruct in 3d?
Run the node and subscribe to the pointcloud with either RViz or with the registration node.

**Note**: if _nobody_ is subscribed to the pointcloud, no processing is made.

### How to calibrate the laser?
In order to calibrate, you must project the laser on a known plane. To do that, a chessboard pattern has to be placed in the line of sight of the camera. Both the laser and the chessboard have to be easy to detect.

The calibrator will start adding calibration images to the memory. Then, once five or more images have been successful, you **MUST** call the calibration service in order to start the plane fitting process. This process will save a YAML file in the disk, with the plane equation `Ax + By + Cz + D = 0` as a 4 component vector `[A, B, C, D]` in the camera coordinate system.


## Subscriptions, publishers, services and parameters:

There are two nodes: **laser_stripe_reconstruction** and **laser_stripe_registration**

1. **laser_stripe_reconstruction**

  1. Subscribers:
    - *image*: color camera image
    - *camera_info*: calibration parameters

  2. Publishers:
    - *points2*: Output pointcloud

  3. Parameters:
    - Node:
      - *uwsim_detector*: True if we want to detect laser from UWSim, false otherwise. (Default: False)
      - *calibrate*: True if we want to start calibrating the tf between the camera and the laser. A checkerboard pattern must be in the field of view of the camera. (Default: False)

    - Detector:
      - *peak_window_size*: Width of window for subpixel peak detection. (Default: 5)
      - *max_laser_width*: Maximum allowed width of laser stripe in the binary image. (Default: 40)
      - *roi_x*, *roi_y*, *roi_width*, *roi_height*: ROI parameters
      - *show_debug_images*: True to show debug images. (Default: false)
    - Triangulator.
      - *camera_frame_id*: (Default "/camera")
      - *laser_frame_id*: (Default "/laser")
      - *laser_calibration*: Path to laser plane calibration (yaml file).
    - Calibrator:
      - *calibration_filename*: Path to store the resulting calibration.yaml
      - *chessboard_squares_x*, *chessboard_squares_y*, *chessboard_size*: Chessboard dimensions. (Default: 8, 6, 0.04)
      - *max_reproj_error*: Allowed reprojection error in pixels. (Default: 1.0)

  4. Services:
    - *calibrate*: Call this service to trigger the calibration (RANSAC plane fitting) of the gathered images.

2. **laser_stripe_registration**

  1. Subscribers:

  2. Publishers:

  3. Parameters:

