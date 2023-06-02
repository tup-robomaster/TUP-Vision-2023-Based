/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 00:29:49
 * @LastEditTime: 2023-06-02 22:53:59
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
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

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
#include "../mvs_driver/mvs_camera.hpp"
#include "../../global_user/include/global_user/global_user.hpp"
#include "global_interface/msg/decision.hpp"
#include "global_interface/msg/serial.hpp"

using namespace std;
using namespace global_user;
using namespace ament_index_cpp;
using std::placeholders::_1;
namespace camera_driver
{
    template<class T>
    class CameraBaseNode : public rclcpp::Node
    {
        typedef global_interface::msg::Decision DecisionMsg;
        typedef global_interface::msg::Serial SerialMsg;

    public:
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
        CameraParam camera_params_;
        std::unique_ptr<T> cam_driver_;
        std::thread img_callback_thread_;
        rclcpp::TimerBase::SharedPtr camera_watcher_timer_;
        // rclcpp::TimerBase::SharedPtr img_callback_timer_;

        Mutex cam_mutex_;
        std::map<std::string, int> param_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_msg_pub_;
        image_transport::CameraPublisher camera_pub2buff_node_;
        image_transport::CameraPublisher camera_pub2armor_node_;

        sensor_msgs::msg::CameraInfo camera_info_msg_;
        sensor_msgs::msg::Image image_msg_;

        string camera_topic_;
        cv::Mat frame_;
        atomic<bool> is_cam_open_;

        // 图像保存
        bool save_video_;
        bool show_img_;
        bool using_ros2bag_;
        int frame_cnt_;
        std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;

        void serialMsgCallback(SerialMsg::SharedPtr msg);
        rclcpp::Subscription<SerialMsg>::SharedPtr serial_msg_sub_; 
        SerialMsg serial_msg_;
        mutex serial_mutex_;
        bool use_serial_;
    };

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

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.best_effort();
        qos.durability();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 5;

        // Camera topic.
        this->declare_parameter<string>("camera_topic", "daheng_img");
        camera_topic_ = this->get_parameter("camera_topic").as_string();

        // Subscriptions transport type.
        string transport_type = "raw";
        
        if(save_video_)
        {   // Video save.
            RCLCPP_WARN_ONCE(this->get_logger(), "Saving video...");
            time_t tmpcal_ptr;
            tm *tmp_ptr = nullptr;
            tmpcal_ptr = time(nullptr);
            tmp_ptr = localtime(&tmpcal_ptr);
            char now[64];
            strftime(now, 64, "%Y-%m-%d_%H_%M_%S", tmp_ptr);  // 以时间为名字
            std::string now_string(now);
            string pkg_path = get_package_share_directory("camera_driver");
            string save_path = this->declare_parameter("save_path", "/recorder/video/gyro_video.webm");
            std::string path = pkg_path + save_path + now_string;

            writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
            rosbag2_storage::StorageOptions storage_options({path, "sqlite3"});
            rosbag2_cpp::ConverterOptions converter_options({
                rmw_get_serialization_format(),
                rmw_get_serialization_format()
            });
            writer_->open(storage_options, converter_options);
            writer_->create_topic({
                camera_topic_,
                "sensor_msgs::msg::Image",
                rmw_get_serialization_format(),
                ""
            });
        }
        else
            RCLCPP_WARN_ONCE(this->get_logger(), "No save video...");

        // Create img publisher.
        string camera_pub2armor_topic = camera_topic_ + "_armor_node";
        string camera_pub2buff_topic = camera_topic_ + "_buff_node";
        this->camera_pub2armor_node_ = image_transport::create_camera_publisher(this, camera_pub2armor_topic, rmw_qos);
        this->camera_pub2buff_node_ = image_transport::create_camera_publisher(this, camera_pub2buff_topic, rmw_qos);

        // Open camera.
        if(!cam_driver_->open())
        {
            RCLCPP_ERROR(this->get_logger(), "Open failed!");
            is_cam_open_ = false;
        }
        else
        {
            is_cam_open_ = true;
        }

        image_msg_.header.frame_id = camera_topic_;
        image_msg_.encoding = "bgr8";
        camera_watcher_timer_ = rclcpp::create_timer(
            this, 
            this->get_clock(), 
            100ms, 
            std::bind(&CameraBaseNode::cameraWatcher, this)
        );

        img_callback_thread_ = std::thread(
            std::bind(&CameraBaseNode::imageCallback, this)
        );

        this->declare_parameter("use_port", false);
        use_serial_ = this->get_parameter("use_port").as_bool();
        serial_msg_.mode = AUTOAIM_NORMAL;
        if (use_serial_)
        {
            //串口消息订阅
            serial_msg_sub_ = this->create_subscription<SerialMsg>(
                "/serial_msg",
                qos,
                std::bind(&CameraBaseNode::serialMsgCallback, this, _1)
            );
        }
    }

    template<class T>
    CameraBaseNode<T>::~CameraBaseNode()
    {
    }
    
    template<class T>
    void CameraBaseNode<T>::serialMsgCallback(SerialMsg::SharedPtr msg)
    {
        serial_mutex_.lock();
        serial_msg_ = *msg;
        serial_mutex_.unlock();
    }

    template<class T>
    void CameraBaseNode<T>::cameraWatcher()
    {
        if (!is_cam_open_)
        {
            cam_mutex_.lock();
            // Reopen camera.
            auto status = cam_driver_->close();
            status = cam_driver_->init();
            if (!cam_driver_->open() && !status)
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Open failed!");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Open Success!");
                is_cam_open_ = true;
            }
            cam_mutex_.unlock();
        }
    }

    template<class T>
    void CameraBaseNode<T>::imageCallback()
    {
        while (1)
        {
            cam_mutex_.lock();
            is_cam_open_ = cam_driver_->getImage(frame_, image_msg_);
            cam_mutex_.unlock();
            if (!is_cam_open_)
            {
                RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
                sleep(1);
                continue;
            }

            rclcpp::Time now = this->get_clock()->now();
            image_msg_.header.stamp = now;
            camera_info_msg_.header = image_msg_.header;
            image_msg_.width = frame_.size().width;
            image_msg_.height = frame_.size().height;

            serial_mutex_.lock();
            int mode = serial_msg_.mode;
            serial_mutex_.unlock();

            if (mode == AUTOAIM_TRACKING || mode == AUTOAIM_NORMAL ||
                mode == AUTOAIM_SLING || mode == OUTPOST_ROTATION_MODE ||
                mode == SENTRY_NORMAL
            )
            {
                camera_pub2armor_node_.publish(image_msg_, camera_info_msg_);
            }
            else if (mode == SMALL_BUFF || mode == BIG_BUFF)
            {
                camera_pub2buff_node_.publish(image_msg_, camera_info_msg_);
            }

            if (save_video_)
            {   // Video recorder.
                ++frame_cnt_;
                if (frame_cnt_ % 25 == 0)
                {
                    sensor_msgs::msg::Image image_msg = image_msg_;
                    auto serializer = rclcpp::Serialization<sensor_msgs::msg::Image>();
                    auto serialized_msg = rclcpp::SerializedMessage();
                    serializer.serialize_message(&image_msg, &serialized_msg);
                    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                    bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                        new rcutils_uint8_array_t,
                        [this](rcutils_uint8_array_t* msg)
                        {
                            if (rcutils_uint8_array_fini(msg) != RCUTILS_RET_OK)
                            {
                                RCLCPP_ERROR(this->get_logger(), "RCUTILS_RET_INVALID_ARGUMENT OR RCUTILS_RET_ERROR");
                            }
                            delete msg;
                        }
                    );
                    *bag_msg->serialized_data = serialized_msg.release_rcl_serialized_message();
                    bag_msg->topic_name = camera_topic_;
                    bag_msg->time_stamp = now.nanoseconds();
                    writer_->write(bag_msg);
                }
            }
            if (show_img_)
            {
                cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
                cv::imshow("frame", frame_);
                cv::waitKey(1);
            }
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
        this->declare_parameter<string>("config_path", "/config/daheng_cam_param.ini");

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
        
        string pkg_prefix = get_package_share_directory("camera_driver");
        string param_config_path = this->get_parameter("config_path").as_string();
        string param_full_path = pkg_prefix + param_config_path;
        camera_params_.config_path = param_full_path;
        
        show_img_ = this->get_parameter("show_img").as_bool();
        save_video_ = this->get_parameter("save_video").as_bool();

        string pkg_share_pth = get_package_share_directory("global_user");
        camera_params_.video_path = pkg_share_pth + this->get_parameter("video_path").as_string();

        return std::make_unique<T>(camera_params_);
    }
} //namespace camera_driver
#endif