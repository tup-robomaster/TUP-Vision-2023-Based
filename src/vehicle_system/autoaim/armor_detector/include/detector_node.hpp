/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 16:49:59
 * @LastEditTime: 2023-01-27 21:49:46
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/detector_node.hpp
 */
#include "../../global_user/include/global_user/global_user.hpp"
#include "./armor_detector/armor_detector.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>

//custom message
#include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/imu.hpp"

using namespace global_user;
using namespace coordsolver;
namespace armor_detector
{
    class DetectorNode : public rclcpp::Node
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef global_interface::msg::Imu ImuMsg;

    public:
        DetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~DetectorNode();
        
    private:
        rclcpp::Time time_start_;
        int image_width_;
        int image_height_;

        // pub target armor msg.
        rclcpp::Publisher<AutoaimMsg>::SharedPtr armor_info_pub_;
        // ArmorsMsg armors_info_;
    
    private:    
        // Params callback.
        std::map<std::string, int> params_map_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        
    private:
        // Subscribe img. 
        std::shared_ptr<image_transport::Subscriber> img_sub_;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
        // std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info;
        // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;

        // Image subscriptions transport type.
        std::string transport_;
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        // std::vector<Armor> detect_armors(const sensor_msgs::msg::Image::SharedPtr& img);

        ImuMsg imu_msg_;
        rclcpp::Subscription<ImuMsg>::SharedPtr imu_info_sub_;
        void sensorMsgCallback(const ImuMsg& imu_msg);
        
    public:
        DetectorParam detector_params_;
        DebugParam debug_;
        GyroParam gyro_params_;
        PathParam path_params_;

        // rclcpp::Node handle;
        // image_transport::ImageTransport it;
        std::unique_ptr<Detector> detector_;
        std::unique_ptr<Detector> init_detector();

    protected:
        bool using_shared_memory_;
        SharedMemoryParam shared_memory_param_;
        std::thread read_memory_thread_; //共享内存读线程
        void run();
    };
} //namespace detector