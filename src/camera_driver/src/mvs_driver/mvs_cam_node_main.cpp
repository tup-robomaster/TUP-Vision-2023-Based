/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-26 23:17:37
 * @LastEditTime: 2022-12-26 23:17:59
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/hik_driver/hik_cam_node_main.cpp
 */
#include "../../include/mvs_driver/mvs_cam_node.hpp"

int main(int argc, char** argv)
{
    // rclcpp::init(argc, argv);
    // const rclcpp::NodeOptions options;
    // auto cam_node = std::make_shared<camera_driver::HikCamNode>(options);
    // rclcpp::spin(cam_node);
    // rclcpp::shutdown();

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    const rclcpp::NodeOptions options;
    auto mvs_cam_node = std::make_shared<camera_driver::MvsCamNode>(options);
    exec.add_node(mvs_cam_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}