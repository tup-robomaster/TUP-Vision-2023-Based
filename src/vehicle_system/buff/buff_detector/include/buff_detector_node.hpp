/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 22:57:12
 * @LastEditTime: 2022-12-19 23:07:33
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/include/buff_detector_node.hpp
 */
#include "./buff_detector/buff_detector.hpp"

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
#define HIK_IMAGE_WIDTH 1440
#define HIK_IAMGE_HEIGHT 1080
#define USB_IMAGE_WIDTH 640
#define USB_IAMGE_HEIGHT 480

namespace buff_detector
{
    class BuffDetectorNode : public rclcpp::Node
    {
    public:
        BuffNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~BuffNode();
    
    private:
        std::unique_ptr<Buff> buff_detector_;
        std::unique_ptr<Buff> init_buff_detector();
    
    private:
        std::string transport_;
        std::shared_ptr<image_transport::Subscriber> img_sub_; //Subscribe images from camera node.
        void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
    
    private:
        rclcpp::Time time_start_;
        rclcpp::Publisher<TargetMsg>::SharedPtr buff_info_pub_; //buff msgs pub.
    
    protected:
        //params callback
        rclcpp::TimerBase::SharedPtr param_timer_;
        void param_callback();
    
    private:
        // Shared memory.
        void run();                         //
        bool using_shared_memory_;          //
        key_t key_;                         //生成key键
        int shared_memory_id_;              //获取共享内存id
        void* shared_memory_ptr_ = nullptr; //映射共享内存，得到虚拟地址
        std::thread read_memory_thread_;    //共享内存读线程
    };
} // namespace buff_detector