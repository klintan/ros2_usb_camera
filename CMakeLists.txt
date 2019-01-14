cmake_minimum_required(VERSION 3.5.0)

project(usb_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()


find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(rmw REQUIRED)
find_package(rmw_opensplice_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_calibration_parsers REQUIRED)


find_package(OpenCV REQUIRED)

include_directories(include ${cv_bridge_INCLUDE_DIRS})


function(custom_executable target)
    add_executable(${target} src/${target}.cpp)
    ament_target_dependencies(${target}
    "example_interfaces"
    "rclcpp"
    "sensor_msgs"
    "rcutils"
    "std_msgs"
    "cv_bridge"
    "camera_info_manager"
    "image_transport"
    "camera_calibration_parsers"
    "OpenCV")
    install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME})
endfunction()

custom_executable(usb_camera_node)


ament_package()
