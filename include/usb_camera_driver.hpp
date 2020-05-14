/*
Copyright (c) 2019 Andreas Klintberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#ifndef USB_CAMERA_DRIVER__CAMERA_DRIVER_HPP_
#define USB_CAMERA_DRIVER__CAMERA_DRIVER_HPP_

#include <stdio.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "sensor_msgs/image_encodings.hpp"

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc.hpp>

namespace usb_camera_driver
{

class CameraDriver : public rclcpp::Node {
public:
    explicit CameraDriver(const rclcpp::NodeOptions&);
    ~CameraDriver() {};
        
private:
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame;
    cv::Mat flipped_frame;
    cv::VideoCapture cap;
    
    bool is_flipped;

    std::string frame_id_;
    int image_height_;
    int image_width_;
    double fps_;
    int camera_id;

    std::chrono::steady_clock::time_point last_frame_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    image_transport::CameraPublisher camera_info_pub_;
    
    std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    
    std::shared_ptr<sensor_msgs::msg::Image> ConvertFrameToMessage(cv::Mat & frame);
    
    void ImageCallback();
    

};
} // namespace usb_camera_driver
#endif //USB_CAMERA_DRIVER__CAMERA_DRIVER_HPP_

