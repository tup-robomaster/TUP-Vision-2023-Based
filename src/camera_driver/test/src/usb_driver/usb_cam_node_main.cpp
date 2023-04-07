/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-26 23:18:27
 * @LastEditTime: 2022-12-26 23:18:45
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam_node_main.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    const rclcpp::NodeOptions options;
    auto usb_cam_node = std::make_shared<camera_driver::UsbCamNode>(options);
    exec.add_node(usb_cam_node);
    exec.spin();
    rclcpp::shutdown();

    return 0;
}