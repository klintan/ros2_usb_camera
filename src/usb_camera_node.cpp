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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "opencv2/highgui/highgui.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"


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

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void convert_frame_to_message(
                              const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
    // copy cv information into ros message
    msg->is_bigendian = false;
    msg->height = frame.rows;
    
    msg->width = frame.cols;
    msg->encoding = mat_type2encoding(frame.type());
    
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    
    size_t size = frame.step * frame.rows;
    msg->data.resize(size);
    memcpy(&msg->data[0], frame.data, size);
    msg->header.frame_id = std::to_string(frame_id);
}



class Camera : public rclcpp::Node
{
public:
    explicit Camera(const std::string & topic_name, const size_t & width, const size_t & height, double freq) : Node("camera")
    {
        // instanstiate message
        msg_ = std::make_shared<sensor_msgs::msg::Image>();
        
        // instanstiate publisher
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name);
        
        
        
        auto capture_frame =
        [this]() -> void {
            cap >> frame;
            if (!frame.empty()) {
                // Convert to a ROS image
                if (!is_flipped) {
                    
                    convert_frame_to_message(frame, i, msg_);
                } else {
                    // Flip the frame if needed
                    cv::flip(frame, flipped_frame, 1);
                    convert_frame_to_message(flipped_frame, i, msg_);
                }
                
                // Publish the image message and increment the frame_id.
                RCLCPP_INFO(this->get_logger(), "Publishing image #%zd", i);
                
                // Put the message into a queue to be processed by the middleware.
                // This call is non-blocking.
                pub_->publish(msg_);
                ++i;
            }
        };
        
        cap.open(0);
        
        cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
        
        //if (!cap.isOpened()) {
        //    RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
        //}
        
        // Create a function for when messages are to be sent.
        /*auto publish_message =
         [this](sensor_msgs::msg::Image::SharedPtr msg, size_t i) -> void
         {
         RCLCPP_INFO(this->get_logger(), "Publishing image #%zd", i);
         
         // Put the message into a queue to be processed by the middleware.
         // This call is non-blocking.
         pub_->publish(msg)
         };
         */
        
        // Initialize a shared pointer to an Image message.
        //msg_->is_bigendian = false;
        
        /*auto capture_frame = [this]() -> cv::Mat {
         return 0;
         }*/
        
        timer_ = this->create_wall_timer(100ms, capture_frame);
        
        
    }
    
private:
    std::shared_ptr<sensor_msgs::msg::Image> msg_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap;
    size_t i = 1;
    bool show_camera = false;
    bool is_flipped = false;
};


int main(int argc, char * argv[])
{
    
    std::cout << "Starting camera node" << std::endl;
    
    rclcpp::init(argc, argv);
    
    // Initialize default demo parameters
    
    double freq = 30.0;
    size_t width = 320;
    size_t height = 240;
    
    std::string topic("image");
    
    // Force flush of the stdout buffer.
    // This ensures a correct sync of all prints
    // even when executed simultaneously within a launch file.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    // Create a node.
    auto node = std::make_shared<Camera>(topic, width, height, freq);
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
