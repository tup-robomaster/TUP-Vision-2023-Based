/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:03:45
 * @LastEditTime: 2022-09-30 09:27:27
 * @FilePath: /tup_2023/src/camera_driver/include/hik_driver/hik_cam_node.hpp
 */

#ifndef HIK_CAM_NODE_HPP
#define HIK_CAM_NODE_HPP

#include "hik_camera.hpp"
// #include "rmoss_master/rmoss_core/rmoss_cam/include/rmoss_cam/cam_server.hpp"

#include <sensor_msgs/image_encodings.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace camera_driver
{
    class hik_cam_node : public rclcpp::Node
    {
    public:
        hik_cam_node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~hik_cam_node(){};
    
    private:    
        cv::Mat frame;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

        std::shared_ptr<hik_camera> hik_cam;

        int hik_cam_id;
        std::string frame_id;
        int image_width;
        int image_height;

        std::chrono::steady_clock::time_point last_frame;

        std::shared_ptr<sensor_msgs::msg::Image> image_msg;

    public:
        std::unique_ptr<sensor_msgs::msg::Image> convert_frame_to_msg(cv::Mat frame);
        void image_callback();
        // std::shared_ptr<rmoss_cam::CamServer> cam_server;
    };
} // namespace camera_driver

#endif // HIK_CAM_NODE_HPP
