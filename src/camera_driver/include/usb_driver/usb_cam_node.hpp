/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:40
 * @LastEditTime: 2023-03-20 10:18:35
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
        int frame_cnt;
        cv::Mat frame;
        cv::Mat filpped_frame;
        cv::VideoCapture cap;    
        bool is_filpped;

        // std::unique_ptr<usb_cam> usb_cam_;
        rclcpp::TimerBase::SharedPtr img_pub_timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;

        rclcpp::Time last_frame_;
        std::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_manager;
        image_transport::CameraPublisher camera_info_pub;
        std::shared_ptr<sensor_msgs::msg::Image> image_msg;

    public:
        void image_callback();
        std::shared_ptr<sensor_msgs::msg::Image> convert_frame_to_message(cv::Mat &frame);
    
    private:
        bool save_video_;
        bool using_video_;
        std::string video_path_;
        VideoRecordParam video_record_param_;        
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