/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 20:32:53
 * @LastEditTime: 2022-09-29 20:04:12
 * @FilePath: /tup_2023/src/camera_driver/src/usb_driver/usb_cam_node_main.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto usb_cam_node = std::make_shared<camera_driver::usb_cam_node>(options);

    exec.add_node(usb_cam_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}