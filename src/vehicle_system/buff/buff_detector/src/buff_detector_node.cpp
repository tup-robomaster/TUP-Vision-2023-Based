/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:08:00
 * @LastEditTime: 2022-12-22 23:29:38
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/src/buff_detector_node.cpp
 */
#include "../include/buff_detector_node.hpp"

using namespace std::placeholders;
namespace buff_detector
{
    BuffDetectorNode::BuffDetectorNode(const rclcpp::NodeOptions& options)
    : Node("buff_detector", options)
    {
        RCLCPP_INFO("buff detector node...");
        
        time_start_ = this->get_clock()->now();

        try
        {
            detector_ = init_detector();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        // QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();

        // buff info pub.
        buff_info_pub_ = this->create_publisher<BuffMsg>("buff_info", qos);

        if(!detector_->is_init)
        {
            detector_->buff_detector_.initModel(path_param_.network_path);
            detector_->coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            detector_->is_initialized_ = true;
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
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BuffDetectorNode::paramsCallback, this, _1));
        }
    }

    BuffDetectorNode::~BuffDetectorNode()
    {
        
    }

    void BuffDetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {   
        RCLCPP_INFO(this->get_logger(), "Image callback...");

        TaskData src;
        if(!img_info)
            return;
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);
        
        rclcpp::Time time_img_sub = this->get_clock()->now();
        src.timestamp = time_img_sub.nanoseconds() - time_start_.nanoseconds();

        TargetInfo target_info;
        BuffMsg buff_msg;
        if(detector_->run(src, target_info))
        {
            buff_msg.header.frame_id = "buff_detector";
            buff_msg.header.stamp = this->get_clock()->now();
            buff_msg.r_center = target_info.r_center;
            buff_msg.rotate_speed = target_info.rotate_speed;
            buff_msg.target_switched = target_info.target_switched;
            
            // publish buff info.
            buff_info_pub_->publish(buff_msg);
        }

        cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
        cv::imshow("dst", src.img);
        cv::waitKey(1);
    }

    rcl_interfaces::msg::SetParametersResult BuffDetectorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            result.successful = setParam(param);
        }
        return result;
    }

    bool BuffDetectorNode::setParam(rclcpp::Parameter& param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            detector_->setDetectorParam(param.as_double(), 1);
            break;
        case 1:
            detector_->setDetectorParam(param.as_double(), 2);
            break;
        case 2:
            detector_->setDetectorParam(param.as_double(), 3);
            break;
        case 3:
            detector_->setDetectorParam(param.as_double(), 4);
            break;
        case 4:
            detector_->setDetectorParam(param.as_double(), 5);
            break;
        case 5:
            detector_->setDebugParam(param.as_bool(), 1);
            break;
        case 6:
            detector_->setDebugParam(param.as_bool(), 2);
            break;
        case 7:
            detector_->setDebugParam(param.as_bool(), 3);
            break;
        case 8:
            detector_->setDebugParam(param.as_bool(), 4);
            break;
        case 9:
            detector_->setDebugParam(param.as_bool(), 5);
            break;
        case 10:
            detector_->setDebugParam(param.as_bool(), 6);
            break;
        case 11:
            detector_->setDebugParam(param.as_bool(), 7);
            break;
        case 12:
            detector_->setDebugParam(param.as_bool(), 8);
            break;
        default:
            break;
        }
        return true;
    }

    std::unique_ptr<Buff> BuffDetectorNode::init_detector()
    {
        param_map_ = 
        {
            {"fan_length", 0}
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
        this->declare_parameter<int>("max_delta_t", 100);
        this->declare_parameter<int>("max_lost_cnt", 4);
        this->declare_parameter<double>("max_v", 4);
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