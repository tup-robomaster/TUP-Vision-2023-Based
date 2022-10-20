/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:40
 * @LastEditTime: 2022-09-29 00:24:21
 * @FilePath: /tup_2023/src/camera_driver/include/usb_cam_node.hpp
 */
#ifndef USB_CAM_NODE_HPP
#define USB_CAM_NODE_HPP

#include "./usb_cam.hpp"
#include <rclcpp/rclcpp.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace camera_driver
{
    class usb_cam_node : public rclcpp::Node
    {
    public:
        usb_cam_node(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        ~usb_cam_node(){};
    private:
        cv::Mat frame;
        cv::Mat filpped_frame;
        cv::VideoCapture cap;    
        bool is_filpped;

        // std::unique_ptr<usb_cam> usb_cam_;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;
        
        // int camera_id; 
        // std::string frame_id;
        // int image_height;
        // int image_width;
        // double fps;

        std::chrono::steady_clock::time_point last_frame;

        std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager;
        image_transport::CameraPublisher camera_info_pub;
        std::shared_ptr<sensor_msgs::msg::Image> image_msg;

        std::shared_ptr<sensor_msgs::msg::Image> convert_frame_to_message(cv::Mat &frame);

        void image_callback();
    
    private:
        std::unique_ptr<usb_cam> usb_cam_;
        std::unique_ptr<usb_cam> init_usb_cam();
    }; // usb_cam_node
} //namespace camera_driver

#endif