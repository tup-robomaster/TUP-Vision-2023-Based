/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-26 23:16:39
 * @LastEditTime: 2023-02-26 12:19:02
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/daheng_driver/daheng_cam_node_main.cpp
 */
#include "../../include/daheng_driver/daheng_cam_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver::DahengCamNode>());
    rclcpp::shutdown();
    
    return 0;
}
