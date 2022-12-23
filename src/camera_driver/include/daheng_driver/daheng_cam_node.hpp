/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:02:35
 * @LastEditTime: 2022-12-23 23:47:34
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/daheng_driver/daheng_cam_node.hpp
 */
#ifndef DAHENG_CAM_NODE_HPP_
#define DAHENG_CAM_NODE_HPP_

#include "./daheng_camera.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
// #include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

//linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define IMAGE_WIDTH 1280
#define IMAGE_HEIGHT 1024

namespace camera_driver
{
    class DahengCamNode : public rclcpp::Node
    {
    public:
        DahengCamNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~DahengCamNode();
    
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
        int frame_cnt; 
        bool save_video_;
        bool is_first_loop;
        std::shared_ptr<cv::VideoWriter> video_writer_;
        std::future<void> writer_video_;

        DahengCamParam daheng_cam_param_;
        std::unique_ptr<DaHengCam> daheng_cam;
        std::unique_ptr<DaHengCam> init_daheng_cam();
    
    protected:
        /**
         * @brief 动态调参
         * @param 参数服务器参数
         * @return 是否修改参数成功
         */
        std::map<std::string, int> param_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);

    protected:
        //图像数据内存共享
        bool using_shared_memory;
        //生成一个key
        key_t key_;
        //共享内存的id
        int shared_memory_id_;
        //映射共享内存，得到虚拟地址
        void* shared_memory_ptr = nullptr;

        std::thread memory_write_thread_;
    };
} // namespace camera_driver


#endif