/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:40
 * @LastEditTime: 2023-04-16 14:37:37
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/usb_driver/usb_cam_node.hpp
 */
#ifndef USB_CAM_NODE_HPP_
#define USB_CAM_NODE_HPP_

#include <memory>

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

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
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include "./usb_cam.hpp"
#include "../../global_user/include/global_user/global_user.hpp"

using namespace global_user;
using namespace ament_index_cpp;
namespace camera_driver
{
    class UsbCamNode : public rclcpp::Node
    {
        typedef rclcpp::ParameterEventHandler ParamSubscriber;
        typedef rclcpp::ParameterCallbackHandle ParamCbHandle;
        typedef rclcpp::ParameterEventHandler::ParameterCallbackType ParamCallbackType;
        
    public:
        UsbCamNode(const rclcpp::NodeOptions& option = rclcpp::NodeOptions());
        ~UsbCamNode();
    
    private:
        cv::Mat frame_;
        cv::Mat filpped_frame_;
        cv::VideoCapture cap_;    
        bool is_filpped_;

        // rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

        // std::unique_ptr<usb_cam> usb_cam_;
        rclcpp::TimerBase::SharedPtr img_pub_timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_msg_pub_;

        rclcpp::Time last_time_;
        // std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
        // image_transport::CameraPublisher camera_info_pub_;
        // std::shared_ptr<sensor_msgs::msg::Image> image_msg_;

    public:
        void image_callback();
        std::shared_ptr<sensor_msgs::msg::Image> convert_frame_to_message(cv::Mat &frame);
    
    private:
        bool save_video_;
        string save_path_;
        bool using_video_;
        std::string video_path_;
        int frame_cnt_;
        std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
        std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
        bool using_ros2bag_;
        sensor_msgs::msg::Image image_msg_;

        UsbCamParam usb_cam_params_;
        std::unique_ptr<UsbCam> usb_cam_;
        std::unique_ptr<UsbCam> init_usb_cam();
    
    protected:
        // std::shared_ptr<ParamSubscriber> param_subscriber_;
        // std::shared_ptr<ParamCbHandle> param_cb_handle_;
        
        // params callback.
        std::map<std::string, int> param_map_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    }; // UsbCamNode
} //namespace camera_driver

#endif