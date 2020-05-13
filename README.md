# ROS2 USB Camera Node

Its based on both the image_tools cam2image demos for ROS2 as well as the libuvc and usb_cam project for ROS1.

Features
- CameraInfo available
- CompressedImage topic (see compressed images for republishing using image_transport)
- Image topic
- Select camera (running the node for each camera connected)

There might be major changes to the code as it is a WIP.


### Topics
- `/camera_info` - topic for camera info
- `/image_raw` - topic for raw image data

## Installation

Make sure to run setup.bash and local_setup.bash for all dependencies or symlink them into the repo.

Run

`colcon build`

## Usage

`ros2 run usb_camera_driver usb_camera_driver_node __ns:=/<your namespace> __params:=config.yaml`

Available parameters:
- `frame_id` -> transform frame_id of the camera, defaults to "camera"
- `image_width` -> image width (1280 default)
- `image_height` -> image height (720 default)
- `fps` -> video fps (10 default)
- `camera_id` -> id of camera to capture (0 - 100, 0 default)

### Calibration files
To use the camera info functionality you need to load a file from the camera_calibration (https://github.com/ros-perception/image_pipeline/tree/ros2/camera_calibration) library and put it in/name it `file:///Users/<youruser>/.ros/camera_info/camera.yaml`


### Compressed images

To get compressed images (works seamlessly with web streaming) republish the topic using image_transport which is available for ROS2.

`ros2 run image_transport republish raw in:=image_raw compressed out:=image_raw_compressed`

Make sure to link/install https://github.com/ros-perception/image_transport_plugins/tree/ros2 before to enable compressed image republishing using image_transport since its not included in the base package. More information here http://wiki.ros.org/image_transport, here http://wiki.ros.org/compressed_image_transport and here https://answers.ros.org/question/35183/compressed-image-to-image/

## References
https://github.com/ros-perception/vision_opencv/tree/ros2
