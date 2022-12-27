/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:02:35
 * @LastEditTime: 2022-12-27 17:32:29
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/daheng_driver/daheng_cam_node.hpp
 */
#ifndef DAHENG_CAM_NODE_HPP_
#define DAHENG_CAM_NODE_HPP_

#include "./daheng_camera.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
namespace camera_driver
{
    class DahengCamNode : public rclcpp::Node
    {
        typedef sensor_msgs::msg::Image ImageMsg;
    public:
        DahengCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~DahengCamNode();
    
    private:    
        cv::Mat frame;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<ImageMsg>::SharedPtr image_pub;

        int daheng_cam_id;
        std::string frame_id;
        int image_width;
        int image_height;

        // std::chrono::steady_clock::time_point last_frame;
        rclcpp::Time last_frame_;
        std::shared_ptr<ImageMsg> image_msg;
    public:
        std::unique_ptr<ImageMsg> convert_frame_to_msg(cv::Mat frame);
        void image_callback();
    
    private:
        bool save_video_;
        VideoRecordParam video_record_param_;
        DahengCamParam daheng_cam_param_;
        std::unique_ptr<DaHengCam> daheng_cam;
        std::unique_ptr<DaHengCam> init_daheng_cam();
    
    protected:
        std::map<std::string, int> param_map_;
        bool setParam(rclcpp::Parameter param);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);

        std::shared_ptr<rclcpp::ParameterEventHandler> debug_param_sub_;
        std::shared_ptr<rclcpp::ParameterCallbackHandle> debug_cb_handle_;
    protected:
        bool using_shared_memory_; //图像数据内存共享
        SharedMemoryParam shared_memory_param_;
        std::thread memory_write_thread_;
    };
} // namespace camera_driver


#endif