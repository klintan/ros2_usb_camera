//
//  usb_camera_node.cpp
//  camera
//
//  Created by Andreas Klintberg on 11/17/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "usb_camera_node.hpp"


using namespace std::chrono_literals;

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
mat_type2encoding(int mat_type)
{
    switch (mat_type) {
        case CV_8UC1:
            return "mono8";
        case CV_8UC3:
            return "bgr8";
        case CV_16SC1:
            return "mono16";
        case CV_8UC4:
            return "rgba8";
        default:
            throw std::runtime_error("Unsupported encoding type");
    }
}

CameraNode::CameraNode(std::shared_ptr<rclcpp::Node>  const nh) : nh_(nh),
image_pub_(nh_),
cinfo_manager_(nh_.get()) {
    std::cout << "intialize camera info manager and advertiseCamera" << std::endl;
    camera_info_pub_ = image_pub_.advertiseCamera("image_raw", 1, false);
};



CameraNode::~CameraNode() {
    
}


std::shared_ptr<sensor_msgs::msg::Image> CameraNode::ConvertFrameToMessage(const cv::Mat & frame)
{
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, frame);
    image_msg_ = img_bridge.toImageMsg(); // from cv_bridge to sensor_msgs::msg::Image
    return image_msg_;
}


void CameraNode::ImageCallback() {
    
    cap >> frame;
    if (!frame.empty()) {
        // Convert to a ROS image
        if (!is_flipped) {
            image_msg_ = ConvertFrameToMessage(frame);
        } else {
            // Flip the frame if needed
            cv::flip(frame, flipped_frame, 1);
            image_msg_ = ConvertFrameToMessage(frame);
        }
        
        // Publish the image message and increment the frame_id.
        RCLCPP_INFO(nh_->get_logger(), "Publishing image");
        
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_(
                                                                 new sensor_msgs::msg::CameraInfo(cinfo_manager_.getCameraInfo()));
        
        camera_info_pub_.publish(image_msg_, camera_info_msg_);
    }
}

bool CameraNode::Start(){
    std::string url = "<path>/camera.yaml";
    cinfo_manager_.loadCameraInfo(url);
    
    cap.open(0);
    
    size_t width = 320;
    size_t height = 240;
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    
    std::cout << "set timer in start" << std::endl;
    timer_ = nh_->create_wall_timer(100ms, std::bind(&CameraNode::ImageCallback, this));
    
    return true;
}

int main(int argc, char * argv[])
{
    // based on libuvc
    std::cout << "Starting camera node" << std::endl;
    
    rclcpp::init(argc, argv);
    
    rclcpp::executors::SingleThreadedExecutor exec;
    
    auto const nh_ = std::make_shared<rclcpp::Node>("camera");
    
    auto camera_node = new CameraNode(nh_);
    
    camera_node->Start();
    
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::spin(nh_);
    
    rclcpp::shutdown();
    return 0;
}
