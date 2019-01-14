# ROS2 USB Camera Node

Its based on both the image_tools cam2image demos for Ros2 as well as the libuvc project for ROS1. 

Features
- CameraInfo (not done)
- CompressedImage topic (done)
- Image topic (not done)


There will be major changes to the code as it is not done yet, WIP.


#### Topics
`/camera_info` - topic for camera info
`/image_raw` - topic for raw image data

## Installation

Make sure to run setup.bash and local_setup.bash for all dependencies or symlink them into the repo.

Run

`colcon build` 

## Usage

`ros2 run usb_camera usb_camera_node`


### Compressed images

To get compressed images (works seamlessly with web streaming) republish the topic using image_transport which is available for ROS2.

`ros2 run image_transport republish raw in:=image_raw compressed out:=image_raw_compressed`

Make sure to link/install https://github.com/ros-perception/image_transport_plugins/tree/ros2 before to enable compressed image republishing using image_transport since its not included in the base package.

## References
https://github.com/ros-perception/vision_opencv/tree/ros2