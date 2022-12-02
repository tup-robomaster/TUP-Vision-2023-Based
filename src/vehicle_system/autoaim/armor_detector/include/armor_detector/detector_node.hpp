/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 16:49:59
 * @LastEditTime: 2022-12-01 21:13:09
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/include/armor_detector/detector_node.hpp
 */
#include "../armor_processor/armor_processor.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
// #include "./detector.hpp"

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
#include "global_interface/msg/armor.hpp"
#include "global_interface/msg/armors.hpp"
#include "global_interface/msg/target.hpp"

//linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define DAHENG_IMAGE_WIDTH 1280
#define DAHENG_IMAGE_HEIGHT 1024

namespace armor_detector
{
    class detector_node : public rclcpp::Node
    {
        typedef std::chrono::_V2::steady_clock::time_point TimePoint;
        typedef global_interface::msg::Armors ArmorsMsg;
        typedef global_interface::msg::Target TargetMsg;

    public:
        detector_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~detector_node();
        
    private:
        //订阅图像  
        std::shared_ptr<image_transport::Subscriber> img_sub;
        // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;

        // Image subscriptions transport type
        std::string transport_;
        TimePoint time_start;
        
        // std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info;
        // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;

        //发布装甲信息
        ArmorsMsg armors_info;
        rclcpp::Publisher<TargetMsg>::SharedPtr armors_pub;

        TargetInfoPtr target_ptr;

        //
        rclcpp::Publisher<TargetMsg>::SharedPtr predict_info_pub;
        rclcpp::Publisher<global_interface::msg::Gimbal>::SharedPtr gimbal_info_pub_;
    
    private:    
        //params callback
        rclcpp::TimerBase::SharedPtr param_timer_;
        void param_callback();
        
    public:
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
        // std::vector<Armor> detect_armors(const sensor_msgs::msg::Image::SharedPtr& img);
  
    public:
        detector_params detector_params_;
        debug_params debug_;
        gyro_params gyro_params_;
        void getParameters();

        // rclcpp::Node handle;
        // image_transport::ImageTransport it;
        std::unique_ptr<detector> detector_;
        std::unique_ptr<detector> init_detector();
    
    private:
        PredictParam predict_param_;
        // SingerModelParam singer_model_param_;
        SingerModel singer_model_param_;
        DebugParam debug_param_;
        std::string filter_param_path_;
        std::string coord_param_path_;
        std::string coord_param_name_;

        std::unique_ptr<Processor> processor_;
        std::unique_ptr<Processor> init_armor_processor();
    protected:
        /**
         * @brief 共享图像数据内存
         * 
         */
        //
        bool using_shared_memory;
        
        //生成key键
        key_t key_;

        //获取共享内存id
        int shared_memory_id_;

        //映射共享内存，得到虚拟地址
        void* shared_memory_ptr_ = nullptr;

        //共享内存读线程
        std::thread read_memory_thread_;

        void run();

    };
} //namespace detector