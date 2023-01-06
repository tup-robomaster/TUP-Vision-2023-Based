/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:08:00
 * @LastEditTime: 2023-01-06 23:36:33
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/src/buff_detector_node.cpp
 */
#include "../include/buff_detector_node.hpp"

using namespace std::placeholders;
namespace buff_detector
{
    BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("buff_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "buff detector node...");

        try
        {
            detector_ = init_detector();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing detector class: %s", e.what());
        }
        
        if(!detector_->is_initialized_)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing detector class");
            detector_->buff_detector_.initModel(path_param_.network_path);
            detector_->coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            detector_->is_initialized_ = true;
        }

        time_start_ = detector_->steady_clock_.now();

        // QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();

        // buff info pub.
        buff_info_pub_ = this->create_publisher<BuffMsg>("buff_info", qos);

        if(debug_param_.using_imu)
        {
            RCLCPP_INFO(this->get_logger(), "Using imu...");
            imu_msg_.header.frame_id = "imu_link";
            imu_msg_.bullet_speed = this->declare_parameter<double>("bullet_speed", 28.0);
            imu_msg_.mode = this->declare_parameter<int>("buff_mode", 3);
            // imu msg sub.
            imu_info_sub_ = this->create_subscription<ImuMsg>("/imu_msg", rclcpp::SensorDataQoS(),
                std::bind(&BuffDetectorNode::sensorMsgCallback, this, _1));
        }

        this->declare_parameter<int>("camera_type", DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();

        transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

        if(camera_type == DaHeng)
        {
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/daheng_img",
                std::bind(&BuffDetectorNode::image_callback, this, _1), transport_));
        }
        else if(camera_type == HikRobot)
        {
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
                std::bind(&BuffDetectorNode::image_callback, this, _1), transport_));
        }
        else if(camera_type == USBCam)
        {
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/usb_img",
                std::bind(&BuffDetectorNode::image_callback, this, _1), transport_));
        }
        else if(camera_type == MVSCam)
        {
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/mvs_img",
                std::bind(&BuffDetectorNode::image_callback, this, _1), transport_));
        }

        bool debug = false;
        this->declare_parameter<bool>("debug", true);
        debug = this->get_parameter("debug").as_bool();
        if(debug)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BuffDetectorNode::paramsCallback, this, _1));
        }
    }

    BuffDetectorNode::~BuffDetectorNode()
    {
        
    }

    void BuffDetectorNode::sensorMsgCallback(const ImuMsg& imu_msg)
    {
        imu_msg_.header.stamp = this->get_clock()->now();

        if(imu_msg.bullet_speed > 10)
            imu_msg_.bullet_speed = imu_msg.bullet_speed;
        if(imu_msg.mode == 3 || imu_msg.mode == 4)
            imu_msg_.mode = imu_msg.mode;
        imu_msg_.quat = imu_msg.quat;
        imu_msg_.twist = imu_msg.twist;

        RCLCPP_INFO(this->get_logger(), "bullet speed: %lfm/s mode: %d", imu_msg_.bullet_speed, imu_msg_.mode);
        return;
    }
    
    void BuffDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {   
        // RCLCPP_INFO(this->get_logger(), "Image callback...");

        TaskData src;
        if(!img_info)
            return;
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);
        
        rclcpp::Time time_img_sub = detector_->steady_clock_.now();
        src.timestamp = (time_img_sub - time_start_).nanoseconds();

        TargetInfo target_info;
        BuffMsg buff_msg;
        if(debug_param_.using_imu)
        {
            double dt = (this->get_clock()->now() - imu_msg_.header.stamp).nanoseconds();
            if(abs(dt / 1e9) > 0.1)
            {
                detector_->setDebugParam(false, 7);
                RCLCPP_WARN(this->get_logger(), "latency: %lf", dt);
            }
            else
            {
                src.bullet_speed = imu_msg_.bullet_speed;
                src.mode = imu_msg_.mode;
                src.quat.w() = imu_msg_.quat.w;
                src.quat.x() = imu_msg_.quat.x;
                src.quat.y() = imu_msg_.quat.y;
                src.quat.z() = imu_msg_.quat.z; 
            }
        }

        if(detector_->run(src, target_info))
        {
            buff_msg.header.frame_id = "gimbal_link";
            buff_msg.header.stamp = this->get_clock()->now();
            buff_msg.r_center.x = target_info.r_center[0];
            buff_msg.r_center.y = target_info.r_center[1];
            buff_msg.r_center.z = target_info.r_center[2];
            buff_msg.rotate_speed = target_info.rotate_speed;
            buff_msg.target_switched = target_info.target_switched;
            
            // publish buff info.
            buff_info_pub_->publish(std::move(buff_msg));
        }

        bool show_img;
        this->get_parameter("show_img", show_img);
        if(show_img)
        {
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
    }

    rcl_interfaces::msg::SetParametersResult BuffDetectorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            result.successful = setParam(std::move(param));
        }
        return result;
    }

    bool BuffDetectorNode::setParam(const rclcpp::Parameter& param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            detector_->setDetectorParam(std::move(param.as_double()), 1);
            break;
        case 1:
            detector_->setDetectorParam(std::move(param.as_double()), 2);
            break;
        case 2:
            detector_->setDetectorParam(std::move(param.as_double()), 3);
            break;
        case 3:
            detector_->setDetectorParam(std::move(param.as_double()), 4);
            break;
        case 4:
            detector_->setDetectorParam(std::move(param.as_double()), 5);
            break;
        case 5:
            detector_->setDebugParam(std::move(param.as_bool()), 1);
            break;
        case 6:
            detector_->setDebugParam(std::move(param.as_bool()), 2);
            break;
        case 7:
            detector_->setDebugParam(std::move(param.as_bool()), 3);
            break;
        case 8:
            detector_->setDebugParam(std::move(param.as_bool()), 4);
            break;
        case 9:
            detector_->setDebugParam(std::move(param.as_bool()), 5);
            break;
        case 10:
            detector_->setDebugParam(std::move(param.as_bool()), 6);
            break;
        case 11:
            detector_->setDebugParam(std::move(param.as_bool()), 7);
            break;
        case 12:
            detector_->setDebugParam(std::move(param.as_bool()), 8);
            break;
        default:
            break;
        }
        return true;
    }

    std::unique_ptr<Detector> BuffDetectorNode::init_detector()
    {
        param_map_ = 
        {
            {"fan_length", 0},
            {"max_delta_t", 1},
            {"max_lost_cnt", 2},
            {"max_v", 3},
            {"no_crop_thres", 4},
            {"assist_label", 5},
            {"detect_red", 6},
            {"print_target_info", 7},
            {"show_all_fans", 8},
            {"show_fps", 9},
            {"using_imu", 10},
            {"using_roi", 11}
        };

        this->declare_parameter<double>("fan_length", 0.7);
        this->declare_parameter<double>("max_delta_t", 100.0);
        this->declare_parameter<int>("max_lost_cnt", 4);
        this->declare_parameter<double>("max_v", 4.0);
        this->declare_parameter<double>("no_crop_thres", 2e-3);

        this->declare_parameter<std::string>("camera_name", "KE0200110075");
        this->declare_parameter<std::string>("camera_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter<std::string>("network_path", "src/vehicle_system/buff/model/buff.xml");
        this->declare_parameter<std::string>("path_prefix", "src/vehicle_system/buff/dataset/");

        this->declare_parameter<bool>("assist_label", false);
        this->declare_parameter<bool>("detect_red", true);
        this->declare_parameter<bool>("prinf_latency", false);
        this->declare_parameter<bool>("print_target_info", false);
        this->declare_parameter<bool>("show_all_fans", true);
        this->declare_parameter<bool>("show_fps", true);
        this->declare_parameter<bool>("using_imu", false);
        this->declare_parameter<bool>("using_roi", false);
        this->declare_parameter<bool>("show_img", false);

        this->get_parameter("max_v", this->buff_param_.max_v);
        this->get_parameter("fan_length", this->buff_param_.fan_length);
        this->get_parameter("max_delta_t", this->buff_param_.max_delta_t);
        this->get_parameter("max_lost_cnt", this->buff_param_.max_lost_cnt);
        this->get_parameter("no_crop_thres", this->buff_param_.no_crop_thres);

        this->get_parameter("camera_name", this->path_param_.camera_name);
        this->get_parameter("camera_param_path", this->path_param_.camera_param_path);
        this->get_parameter("network_path", this->path_param_.network_path);
        this->get_parameter("path_prefix", this->path_param_.path_prefix);

        this->get_parameter("assist_label", this->debug_param_.assist_label);
        this->get_parameter("detect_red", this->debug_param_.detect_red);
        this->get_parameter("prinf_latency", this->debug_param_.prinf_latency);
        this->get_parameter("print_target_info", this->debug_param_.print_target_info);
        this->get_parameter("show_all_fans", this->debug_param_.show_all_fans);
        this->get_parameter("show_fps", this->debug_param_.show_fps);
        this->get_parameter("using_imu", this->debug_param_.using_imu);
        this->get_parameter("using_roi", this->debug_param_.using_roi);

        return std::make_unique<Detector>(buff_param_, path_param_, debug_param_);
    }

} // namespace buff_detector

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<buff_detector::BuffDetectorNode>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(buff_detector::BuffDetectorNode)