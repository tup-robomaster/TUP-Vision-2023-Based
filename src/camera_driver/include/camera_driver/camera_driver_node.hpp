/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 00:29:49
 * @LastEditTime: 2023-03-14 22:48:00
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

using namespace global_user;
using namespace std;
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
        
        void image_callback();
        void convert_frame_to_msg(cv::Mat& frame);
        virtual std::unique_ptr<T> init_cam_driver();

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
        string frame_id_;
        ImageInfo image_info_;
        ImageSize image_size_;
        CameraParam camera_params_;
        std::unique_ptr<T> cam_driver_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::map<std::string, int> param_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        image_transport::CameraPublisher camera_pub_;
        sensor_msgs::msg::Image image_msg_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;
        // std::thread img_pub_thread_;
        cv::Mat frame_;

        // 图像保存
        bool save_video_;
        VideoRecordParam video_record_param_;
    };

    template<class T>
    CameraBaseNode<T>::~CameraBaseNode()
    {
        // if(img_pub_thread_.joinable())
        //     img_pub_thread_.join();
    }

    template<class T>
    CameraBaseNode<T>::CameraBaseNode(string node_name, const rclcpp::NodeOptions& options)
    : Node(node_name, options)
    {
        RCLCPP_WARN(this->get_logger(), "Camera driver node...");
        try
        {   // Camera params initialize.
            cam_driver_ = init_cam_driver();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing camera: %s", e.what());
        }

        this->declare_parameter<bool>("save_video", false);
        save_video_ = this->get_parameter("save_video").as_bool();
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
        // qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // Camera type.
        this->declare_parameter<int>("camera_type", DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();

        // Subscriptions transport type.
        string transport_type = "raw";
    
        image_size_ = image_info_.image_size_map[camera_type];
        string camera_topic = image_info_.camera_topic_map[camera_type];

        // Create img publisher.
        // this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_topic, rclcpp::SensorDataQoS());
        this->camera_pub_ = image_transport::create_camera_publisher(this, camera_topic, rmw_qos);
        
        // Open camera.
        if(!cam_driver_->open())
            RCLCPP_ERROR(this->get_logger(), "Open failed!");

        image_msg_.header.frame_id = this->frame_id_;
        image_msg_.encoding = "bgr8";
        // img_pub_thread_ = std::thread(&CameraBaseNode::image_callback, this);    
        timer_ = this->create_wall_timer(1ms, std::bind(&CameraBaseNode::image_callback, this));
        // RCLCPP_INFO(this->get_logger(), "Using image callback func...");
    }

    template<class T>
    void CameraBaseNode<T>::image_callback()
    {
        // while (1)
        // {
            // rclcpp::Time start = this->get_clock()->now();

            // cv::Mat frame;
            if (!cam_driver_->get_frame(frame_, image_msg_))
            {
                RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
                // Reopen camera.
                auto status = cam_driver_->close();
                // status = cam_driver_->init();
                // cam_driver_ = std::make_unique<T>();
                cam_driver_->cam_param_.cam_id = (cam_driver_->cam_param_.cam_id < 5) ? (cam_driver_->cam_param_.cam_id + 1) : 0;
                status = cam_driver_->deviceReset();
                if (!cam_driver_->open())
                {
                    RCLCPP_ERROR(this->get_logger(), "Open failed!");
                    sleep(1);
                }
                return;
            }

            // rclcpp::Time now = this->get_clock()->now();
            // convert_frame_to_msg(frame_);
            // msg->header.stamp = now;
            // image_pub_->publish(std::move(msg));

            image_msg_.header.stamp = this->get_clock()->now();
            camera_info_msg_.header = image_msg_.header;
            image_msg_.width = this->image_size_.width;
            image_msg_.height = this->image_size_.height;
            // image_msg_.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);  
            // image_msg_.is_bigendian = false;
            camera_pub_.publish(std::move(image_msg_), camera_info_msg_);
                
            save_video_ = this->get_parameter("save_video").as_bool();
            if (save_video_)
            {   // Video recorder.
                videoRecorder(video_record_param_, &frame_);
            }
            
            bool show_img = this->get_parameter("show_img").as_bool();
            if (show_img)
            {
                cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
                cv::imshow("frame", frame_);
                cv::waitKey(1);
            }
            // rclcpp::Time now = this->get_clock()->now();
            // RCLCPP_WARN(this->get_logger(), "Dur_delay:%.3fms", (now.nanoseconds() - start.nanoseconds()) / 1e6);
        // }
    }

    template<class T>
    void CameraBaseNode<T>::convert_frame_to_msg(cv::Mat& frame)
    {
        // std_msgs::msg::Header header;
        // sensor_msgs::msg::Image ros_image;

        if(frame.size().width != this->image_size_.width || frame.size().height != this->image_size_.height)
        {
            cv::resize(frame, frame, cv::Size(this->image_size_.width, this->image_size_.height));
            RCLCPP_WARN(this->get_logger(), "Resize frame: width:%d height:%d", this->image_size_.width, this->image_size_.height);
        }

        image_msg_.header.frame_id = this->frame_id_;
        image_msg_.header.stamp = this->get_clock()->now();
        image_msg_.width = this->image_size_.width;
        image_msg_.height = this->image_size_.height;
        image_msg_.encoding = "bgr8";
        
        image_msg_.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);  
        image_msg_.is_bigendian = false;
        image_msg_.data.assign(frame.datastart, frame.dataend);

        // auto msg_ptr = std::make_unique<sensor_msgs::msg::Image>(ros_image);      
        // return msg_ptr;
    }

    template<class T>
    std::unique_ptr<T> CameraBaseNode<T>::init_cam_driver()
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
        this->declare_parameter("video_path", "\0");

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
        camera_params_.video_path = this->get_parameter("video_path").as_string();

        return std::make_unique<T>(camera_params_);
    }
} //namespace camera_driver
#endif