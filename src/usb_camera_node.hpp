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

class CameraNode {
public:
    explicit CameraNode(std::shared_ptr<rclcpp::Node> const nh);
    ~CameraNode();
    
    bool Start();
    void Stop();
    
    std::shared_ptr<rclcpp::Node> nh_;

private:
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap;
    
    cv_bridge::CvImage img_bridge;

    
    size_t i;
    bool show_camera;
    bool is_flipped;
    
    
    camera_info_manager::CameraInfoManager cinfo_manager_;
    image_transport::CameraPublisher camera_info_pub_;
    
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    std::shared_ptr<sensor_msgs::msg::CompressedImage> compressed_image_msg_;

    
    
    std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(const cv::Mat & frame, size_t frame_id);

    image_transport::ImageTransport image_pub_;
    
    void ImageCallback();
    

};

#endif /* usb_camera_node_hpp */
