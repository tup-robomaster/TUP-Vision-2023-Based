/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:02:35
 * @LastEditTime: 2022-10-09 16:02:56
 * @FilePath: /tup_2023/src/camera_driver/include/daheng_driver/daheng_cam_node.hpp
 */
#ifndef DAHENG_CAM_NODE_HPP
#define DAHENG_CAM_NODE_HPP

#include "./daheng_camera.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace camera_driver
{
    class daheng_cam_node : public rclcpp::Node
    {
    public:
        daheng_cam_node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~daheng_cam_node(){};
    
    private:    
        cv::Mat frame;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;


        int daheng_cam_id;
        std::string frame_id;
        int image_width;
        int image_height;

        std::chrono::steady_clock::time_point last_frame;

        std::shared_ptr<sensor_msgs::msg::Image> image_msg;

    public:
        std::unique_ptr<sensor_msgs::msg::Image> convert_frame_to_msg(cv::Mat frame);
        void image_callback();
    
    private:
        std::unique_ptr<daheng_camera> daheng_cam;
        std::unique_ptr<daheng_camera> init_daheng_cam();
    };
} // namespace camera_driver


#endif