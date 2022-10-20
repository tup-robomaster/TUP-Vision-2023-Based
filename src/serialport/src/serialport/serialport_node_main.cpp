/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-27 19:41:13
 * @LastEditTime: 2022-09-28 00:43:11
 * @FilePath: /tup_2023/src/serialport/src/serialport/serialport_node_main.cpp
 */
#include "../../include/serialport/serialport_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialport::serial_driver>());
    rclcpp::shutdown();
    return 0;
}