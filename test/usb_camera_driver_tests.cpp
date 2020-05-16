#include <opencv2/core/core.hpp>
#include <gtest/gtest.h>

#include "usb_camera_driver.hpp"

#include "rclcpp/rclcpp.hpp"

TEST(CameraDriverTests, ParametersTest) {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}

    const rclcpp::NodeOptions options;
    auto node = std::make_shared<usb_camera_driver::CameraDriver>(options);

    /* Test default parameter settings */
    ASSERT_EQ(node->get_parameter("image_width").as_int(), 1280);
    ASSERT_EQ(node->get_parameter("image_height").as_int(), 720);
    ASSERT_EQ(node->get_parameter("fps").as_double(), 10.0);
    ASSERT_EQ(node->get_parameter("camera_id").as_int(), 0);
}



int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}