//
//  usb_camera_node.hpp
//  camera
//
//  Created by Andreas Klintberg on 11/17/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//

#ifndef usb_camera_node_hpp
#define usb_camera_node_hpp

#include <stdio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"

class Camera : public rclcpp::Node {
public:
    explicit Camera(const std::string & topic_name, const size_t & width, const size_t & height, double freq);
    ~Camera();
    
private:
    cv_bridge::CvImage img_bridge;
    void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg);

    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    
    /* using image_transport library instead of native Ros2 messages */
    //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    std::shared_ptr<sensor_msgs::msg::CompressedImage> compressed_image_msg_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_pub_;
    
    std::shared_ptr<sensor_msgs::msg::CameraInfo> camera_info_msg_;
    
    //camera_info_manager::CameraInfoManager cinfo_manager_;
    image_transport::CameraPublisher camera_info_pub_;

    //image_transport::ImageTransport image_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap;
    size_t i;
    bool show_camera;
    bool is_flipped;
};


#endif /* usb_camera_node_hpp */
