/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:03:45
 * @LastEditTime: 2022-11-14 10:42:44
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/hik_driver/hik_cam_node.hpp
 */

#ifndef HIK_CAM_NODE_HPP_
#define HIK_CAM_NODE_HPP_

#include "hik_camera.hpp"
// #include "rmoss_master/rmoss_core/rmoss_cam/include/rmoss_cam/cam_server.hpp"

#include <sensor_msgs/image_encodings.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

namespace camera_driver
{
    class HikCamNode : public rclcpp::Node
    {
    public:
        HikCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~HikCamNode(){};
    
    private:    
        cv::Mat frame;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

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
        
    private:
        HikCamParam hik_cam_params_;
        std::unique_ptr<HikCamera> hik_cam;
        std::unique_ptr<HikCamera> init_hik_cam();

    protected:
        /**
         * @brief 动态调参
         * @param 参数服务器参数
         * @return 是否修改参数成功
         */
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    };
} // namespace camera_driver

#endif // HIK_CAM_NODE_HPP_
