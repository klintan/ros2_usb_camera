# ROS2 USB Camera Node

Its based on both the image_tools cam2image demos for Ros2 as well as the libuvc project for ROS1. 

Features
- CameraInfo (not done)
- CompressedImage topic (done)
- Image topic (not done)


There will be major changes to the code as it is not done yet, WIP.

## Installation

Make sure to run setup.bash and local_setup.bash for all dependencies or symlink them into the repo.

Run

`colcon build` 

## Usage

`ros2 run usb_camera usb_camera_node`

## References
https://github.com/ros-perception/vision_opencv/tree/ros2