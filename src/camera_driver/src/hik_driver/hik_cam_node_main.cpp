/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 15:04:00
 * @LastEditTime: 2022-09-30 14:18:01
 * @FilePath: /tup_2023/src/camera_driver/src/hik_driver/hik_cam_node_main.cpp
 */
#include "../../include/hik_driver/hik_cam_node.hpp"

int main(int argc, char** argv)
{
    // rclcpp::init(argc, argv);
    // const rclcpp::NodeOptions options;
    // auto cam_node = std::make_shared<camera_driver::hik_cam_node>(options);
    // rclcpp::spin(cam_node);
    // rclcpp::shutdown();

    // printf("2\n");
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;

    auto hik_cam_node = std::make_shared<camera_driver::hik_cam_node>(options);

    exec.add_node(hik_cam_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
