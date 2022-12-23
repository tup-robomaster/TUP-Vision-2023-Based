/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 02:03:45
 * @LastEditTime: 2022-12-24 00:07:25
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/hik_driver/hik_cam_node.hpp
 */

#ifndef HIK_CAM_NODE_HPP_
#define HIK_CAM_NODE_HPP_

#include "./hik_camera.hpp"

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

#define IMAGE_WIDTH 1440
#define IMAGE_HEIGHT 1080

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

        std::chrono::steady_clock::time_point last_frame;

        std::shared_ptr<sensor_msgs::msg::Image> image_msg;

    public:
        std::unique_ptr<sensor_msgs::msg::Image> convert_frame_to_msg(cv::Mat frame);
        void image_callback();
        // std::shared_ptr<rmoss_cam::CamServer> cam_server;
        
    private:
        int frame_cnt; 
        bool save_video_;
        bool is_first_loop;
        std::shared_ptr<cv::VideoWriter> video_writer_;
        std::future<void> writer_video_;
        
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
        bool using_shared_memory;           //图像数据内存共享
        key_t key_;                         //生成一个key
        int shared_memory_id_;              //共享内存的id
        void* shared_memory_ptr = nullptr;  //映射共享内存，得到虚拟地址
        std::thread memory_write_thread_;
    };
} // namespace camera_driver

#endif // HIK_CAM_NODE_HPP_
