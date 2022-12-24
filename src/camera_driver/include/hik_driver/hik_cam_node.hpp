/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:03:45
 * @LastEditTime: 2022-12-24 17:46:49
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/hik_driver/hik_cam_node.hpp
 */

#ifndef HIK_CAM_NODE_HPP_
#define HIK_CAM_NODE_HPP_

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include "./hik_camera.hpp"
#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace camera_driver
{
    class HikCamNode : public rclcpp::Node
    {
    public:
        HikCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~HikCamNode();
    
    private:    
        cv::Mat frame;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;

        int hik_cam_id;
        std::string frame_id;
        int image_width;
        int image_height;

        // std::chrono::steady_clock::time_point last_frame;
        rclcpp::Time last_frame_;
        std::shared_ptr<sensor_msgs::msg::Image> image_msg;

    public:
        std::unique_ptr<sensor_msgs::msg::Image> convert_frame_to_msg(cv::Mat frame);
        void image_callback();
        
    private:
        bool save_video_;
        VideoRecordParam video_record_param_;
        HikCamParam hik_cam_params_;
        std::unique_ptr<HikCamera> hik_cam;
        std::unique_ptr<HikCamera> init_hik_cam();

    protected:
        //动态调参
        std::map<std::string, int> param_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    
    private:
        bool using_shared_memory_; //图像数据内存共享
        SharedMemoryParam shared_memory_param_;
        std::thread memory_write_thread_;
    };
} // namespace camera_driver

#endif // HIK_CAM_NODE_HPP_
