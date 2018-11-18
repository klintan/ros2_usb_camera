//
//  usb_camera_node.cpp
//  camera
//
//  Created by Andreas Klintberg on 11/17/18.
//  Copyright © 2018 Andreas Klintberg. All rights reserved.
//

#include <chrono>
#include "usb_camera_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Camera : public rclcpp::Node
{
public:
    explicit Camera(const std::string & topic_name)
    : Node("camera")
    {
        msg_ = std::make_shared<std_msgs::msg::String>();
        
        // Create a function for when messages are to be sent.
        auto publish_message =
        [this]() -> void
        {
            msg_->data = "Hello World: " + std::to_string(count_++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
            
            // Put the message into a queue to be processed by the middleware.
            // This call is non-blocking.
            pub_->publish(msg_);
        };
        
        // Create a publisher with a custom Quality of Service profile.
        rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
        custom_qos_profile.depth = 7;
        pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, custom_qos_profile);
        
        // Use a timer to schedule periodic message publishing.
        timer_ = this->create_wall_timer(1s, publish_message);
    }
    
private:
    size_t count_ = 1;
    std::shared_ptr<std_msgs::msg::String> msg_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    std::cout << "Hello world" << std::endl;
    rclcpp::init(argc, argv);
}
