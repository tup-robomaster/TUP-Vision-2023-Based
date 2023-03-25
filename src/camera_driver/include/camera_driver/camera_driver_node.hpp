/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 00:29:49
 * @LastEditTime: 2023-03-22 03:30:42
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/camera_driver/camera_driver_node.hpp
 */
#ifndef CAMERA_DRIVER_NODE_HPP_
#define CAMERA_DRIVER_NODE_HPP_

//ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

//opencv
#include <opencv2/opencv.hpp>

//c++
#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>

#include "../usb_driver/usb_cam.hpp"
#include "../hik_driver/hik_camera.hpp"
#include "../daheng_driver/daheng_camera.hpp"
#include "../../global_user/include/global_user/global_user.hpp"

using namespace std;
using namespace global_user;
using namespace ament_index_cpp;
using std::placeholders::_1;
namespace camera_driver
{
    template<class T>
    class CameraBaseNode : public rclcpp::Node
    {
    public:
        // explicit CameraBaseNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        CameraBaseNode(string node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~CameraBaseNode();
        
        void imageCallback();
        virtual std::unique_ptr<T> initCamDriver();
        void cameraWatcher();

    public:
        // Update params.
        virtual bool setParam(rclcpp::Parameter param)
        {
            return true;
        }

        // Params callback.
        virtual rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }

    public:
        ImageInfo image_info_;
        ImageSize image_size_;
        CameraParam camera_params_;
        std::unique_ptr<T> cam_driver_;
        rclcpp::TimerBase::SharedPtr camera_watcher_timer_;
        rclcpp::TimerBase::SharedPtr img_callback_timer_;
        std::map<std::string, int> param_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        image_transport::CameraPublisher camera_pub_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;
        sensor_msgs::msg::Image image_msg_;
        cv::Mat frame_;
        bool is_cam_open_;
        int camera_type_;

        // 图像保存
        bool save_video_;
        bool show_img_;
        VideoRecordParam video_record_param_;
    };

    template<class T>
    CameraBaseNode<T>::~CameraBaseNode()
    {
    }
    
    template<class T>
    void CameraBaseNode<T>::cameraWatcher()
    {
        if (!is_cam_open_)
        {
            // Reopen camera.
            auto status = cam_driver_->close();
            status = cam_driver_->init();
            if (!cam_driver_->open() && !status)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Open failed!");
            }
            else
            {
                is_cam_open_ = true;
            }
        }
    }

    template<class T>
    CameraBaseNode<T>::CameraBaseNode(string node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options)
    {
        RCLCPP_WARN(this->get_logger(), "Camera driver node...");
        try
        {   // Camera params initialize.
            cam_driver_ = initCamDriver();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing camera: %s", e.what());
        }

        if(save_video_)
        {   // Video save.
            videoRecorder(video_record_param_);
            RCLCPP_INFO(this->get_logger(), "Saving video...");
        }
        else
            RCLCPP_INFO(this->get_logger(), "No save video...");

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.durability();
        // qos.reliable();
        // qos.transient_local();
        // qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // Camera type.
        this->declare_parameter<int>("camera_type", DaHeng);
        camera_type_ = this->get_parameter("camera_type").as_int();

        // Subscriptions transport type.
        string transport_type = "raw";
    
        image_size_ = image_info_.image_size_map[camera_type_];
        string camera_topic = image_info_.camera_topic_map[camera_type_];

        // Create img publisher.
        this->camera_pub_ = image_transport::create_camera_publisher(this, camera_topic, rmw_qos);
        
        // Open camera.
        if(!cam_driver_->open())
            RCLCPP_ERROR(this->get_logger(), "Open failed!");
        else
            is_cam_open_ = true;

        image_msg_.header.frame_id = camera_topic;
        image_msg_.encoding = "bgr8";
        img_callback_timer_ = this->create_wall_timer(1ms, std::bind(&CameraBaseNode::imageCallback, this));
        camera_watcher_timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, std::bind(&CameraBaseNode::cameraWatcher, this));
    }

    template<class T>
    void CameraBaseNode<T>::imageCallback()
    {
        if (!cam_driver_->get_frame(frame_, image_msg_))
        {
            RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
            is_cam_open_ = false;
            return;
        }

        image_msg_.header.stamp = this->get_clock()->now();
        camera_info_msg_.header = image_msg_.header;
        image_msg_.width = this->image_size_.width;
        image_msg_.height = this->image_size_.height;
        camera_pub_.publish(image_msg_, camera_info_msg_);
            
        if (save_video_)
        {   // Video recorder.
            videoRecorder(video_record_param_, &frame_);
        }
        if (show_img_)
            {
                cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
                cv::imshow("frame", frame_);
                cv::waitKey(1);
        }
    }

    template<class T>
    std::unique_ptr<T> CameraBaseNode<T>::initCamDriver()
    {
        param_map_ = 
        {
            {"exposure_time", 0},
            {"exposure_gain", 1},
            {"balance_b", 2},
            {"balance_g", 3},
            {"balance_r", 4}
        };

        this->declare_parameter("cam_id", 1);
        this->declare_parameter("image_width", 1280);
        this->declare_parameter("image_height", 1024);
        this->declare_parameter("width_scale", 1);
        this->declare_parameter("height_scale", 1);
        this->declare_parameter("exposure_time", 6000);
        this->declare_parameter("exposure_gain", 14);
        this->declare_parameter("exposure_gain_b", 0);
        this->declare_parameter("exposure_gain_g", 0);
        this->declare_parameter("exposure_gain_r", 0);
        this->declare_parameter("auto_balance", false);
        this->declare_parameter("balance_b", 1.56);
        this->declare_parameter("balance_g", 1.0); 
        this->declare_parameter("balance_r", 1.548);
        this->declare_parameter("show_img", false);
        this->declare_parameter("using_video", false);
        this->declare_parameter("fps", 30);
        this->declare_parameter("video_path", "/config/camera_ros.yaml");
        this->declare_parameter<bool>("save_video", false);

        camera_params_.cam_id = this->get_parameter("cam_id").as_int();
        camera_params_.image_width = this->get_parameter("image_width").as_int();
        camera_params_.image_height = this->get_parameter("image_height").as_int();
        camera_params_.width_scale = this->get_parameter("width_scale").as_int();
        camera_params_.height_scale = this->get_parameter("height_scale").as_int();
        camera_params_.exposure_time = this->get_parameter("exposure_time").as_int();
        camera_params_.exposure_gain = this->get_parameter("exposure_gain").as_int();
        camera_params_.exposure_gain_b = this->get_parameter("exposure_gain_b").as_int();
        camera_params_.exposure_gain_g = this->get_parameter("exposure_gain_g").as_int();
        camera_params_.exposure_gain_r = this->get_parameter("exposure_gain_r").as_int();
        camera_params_.auto_balance = this->get_parameter("auto_balance").as_bool();
        camera_params_.balance_b = this->get_parameter("balance_b").as_double();
        camera_params_.balance_g = this->get_parameter("balance_g").as_double();
        camera_params_.balance_r = this->get_parameter("balance_r").as_double();
        camera_params_.using_video = this->get_parameter("using_video").as_bool();
        camera_params_.fps = this->get_parameter("fps").as_int();
        show_img_ = this->get_parameter("show_img").as_bool();
        save_video_ = this->get_parameter("save_video").as_bool();

        string pkg_share_pth = get_package_share_directory("global_user");
        camera_params_.video_path = pkg_share_pth + this->get_parameter("video_path").as_string();

        return std::make_unique<T>(camera_params_);
    }
} //namespace camera_driver
#endif