/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 16:49:59
 * @LastEditTime: 2023-05-21 23:24:30
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/detector_node.hpp
 */
// #include "../../global_user/include/global_user/global_user.hpp"
#include "./armor_detector/armor_detector.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

//custom message
#include "global_interface/msg/gimbal.hpp" 
#include "global_interface/msg/autoaim.hpp"
#include "global_interface/msg/serial.hpp"
#include "global_interface/msg/obj_hp.hpp"
#include "global_interface/msg/decision.hpp"

using namespace global_user;
using namespace coordsolver;
using namespace ament_index_cpp;
namespace armor_detector
{
    class DetectorNode : public rclcpp::Node
    {
        typedef global_interface::msg::Autoaim AutoaimMsg;
        typedef global_interface::msg::Serial SerialMsg;
    public:
        DetectorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~DetectorNode();
        
    private:
        // Pub target armor msg.
        rclcpp::Publisher<AutoaimMsg>::SharedPtr armor_msg_pub_;

    private:    
        // Params callback.
        bool updateParam();
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        
    private:
        // Subscribe img. 
        std::shared_ptr<image_transport::Subscriber> img_msg_sub_;
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg);

        // Subscribe serial msg.
        SerialMsg serial_msg_;
        Mutex serial_msg_mutex_;
        rclcpp::Subscription<SerialMsg>::SharedPtr serial_msg_sub_;
        void sensorMsgCallback(const SerialMsg& serial_msg);

    public:
        Mutex param_mutex_;
        DetectorParam detector_params_;
        GyroParam gyro_params_;
        PathParam path_params_;
        DebugParam debug_;
        atomic<int> mode_ = 1;

        std::unique_ptr<Detector> detector_;
        std::unique_ptr<Detector> initDetector();
    };
} //namespace detector