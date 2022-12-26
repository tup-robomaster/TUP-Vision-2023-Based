/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 17:11:03
 * @LastEditTime: 2022-12-26 23:47:38
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/detector_node.cpp
 */
#include "../include/detector_node.hpp"

using namespace std::placeholders;
namespace armor_detector
{
    DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
    {
        RCLCPP_WARN(this->get_logger(), "Starting detector node...");

        try
        {   //detector类初始化
            this->detector_ = init_detector();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();
        
        // target info pub.
        armor_info_pub_ = this->create_publisher<TargetMsg>("/armor_info", qos);

        time_start_ = detector_->steady_clock_.now();
        
        // CameraType camera_type;
        this->declare_parameter<int>("camera_type", DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();

        // Subscriptions transport type.
        transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        
        // Using shared memory.
        this->declare_parameter("using_shared_memory", false);
        using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();
        if(using_shared_memory_)
        {
            sleep(5);
            try
            {
                if(!getSharedMemory(shared_memory_param_, 5))
                    RCLCPP_ERROR(this->get_logger(), "Shared memory init failed...");
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }

            // img process thread.
            this->read_memory_thread_ = std::thread(&DetectorNode::run, this);
            // this->read_memory_thread_.join();
        }
        else
        {
            // image sub.
            if(camera_type == DaHeng)
            {
                this->image_width_ = DAHENG_IMAGE_WIDTH;
                this->image_height_ = DAHENG_IMAGE_HEIGHT;
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/daheng_img",
                    std::bind(&DetectorNode::image_callback, this, _1), transport_));
            }
            else if(camera_type == HikRobot)
            {
                this->image_width_ = HIK_IMAGE_WIDTH;
                this->image_height_ = HIK_IMAGE_HEIGHT;
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
                    std::bind(&DetectorNode::image_callback, this, _1), transport_));
            }
            else if(camera_type == USBCam)
            {
                this->image_width_ = USB_IMAGE_WIDTH;
                this->image_height_ = USB_IMAGE_HEIGHT;
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/usb_img",
                    std::bind(&DetectorNode::image_callback, this, _1), transport_));
            }
            else if(camera_type == MVSCam)
            {
                this->image_width_ = MVS_IMAGE_WIDTH;
                this->image_height_ = MVS_IMAGE_HEIGHT;
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/mvs_img",
                    std::bind(&DetectorNode::image_callback, this, _1), transport_));
            }
        }

        bool debug = false;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug);
        if(debug)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DetectorNode::paramsCallback, this, _1));
        }
    }

    DetectorNode::~DetectorNode()
    {
        if(using_shared_memory_)
        {
            if(!destorySharedMemory(shared_memory_param_))
                RCLCPP_ERROR(this->get_logger(), "Destory shared memory failed...");
        }
    }

    void DetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        TaskData src;
        std::vector<Armor> armors;

        if(!img_info)
            return;
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);

        auto img_sub_time = detector_->steady_clock_.now();
        src.timestamp = (img_sub_time - time_start_).nanoseconds();
        
        if(detector_->armor_detect(src))
        {   
            // RCLCPP_INFO(this->get_logger(), "armors detector...");
            TargetMsg target_info;
            if(detector_->gyro_detector(src, target_info))
            {
                target_info.header.frame_id = "armor_detector";
                target_info.header.stamp = this->get_clock()->now();
                target_info.timestamp = src.timestamp;
                // Publish target's information containing 3d point and timestamp.
                armor_info_pub_->publish(std::move(target_info));
            }

        }

        if(debug_.show_img)
        {
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
    }

    void DetectorNode::run()
    {
        TaskData src;
        std::vector<Armor> armors;

        Mat img = Mat(this->image_height_, this->image_width_, CV_8UC3);
        while(1)
        {
            //读取共享内存图像数据
            memcpy(img.data, shared_memory_param_.shared_memory_ptr, this->image_height_ * this->image_width_ * 3);
            img.copyTo(src.img);

            auto img_sub_time = detector_->steady_clock_.now();
            src.timestamp = (img_sub_time - time_start_).nanoseconds();
            if(detector_->armor_detect(src))
            {   
                // RCLCPP_INFO(this->get_logger(), "armors detector...");
                TargetMsg target_info;
                Eigen::Vector3d aiming_point;

                // Target spinning detector. 
                if(detector_->gyro_detector(src, target_info))
                {
                    target_info.header.frame_id = "armor_detector";
                    target_info.header.stamp = this->get_clock()->now();
                    target_info.timestamp = src.timestamp;

                    // Publish target's information containing 3d point and timestamp.
                    armor_info_pub_->publish(std::move(target_info));
                }
                
            }
            if(debug_.show_img)
            {
                cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
                cv::imshow("src", src.img);
                cv::waitKey(1);
            }
        }
    }

    rcl_interfaces::msg::SetParametersResult DetectorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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
    
    bool DetectorNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = params_map_[param.get_name()];
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
            detector_->setDetectorParam(std::move(param.as_double()), 6);
            break;
        case 6:
            detector_->setDetectorParam(std::move(param.as_double()), 7);
            break;
        case 7:
            detector_->setDetectorParam(std::move(param.as_double()), 8);
            break;
        case 8:
            detector_->setDetectorParam(std::move(param.as_double()), 9);
            break;
        case 9:
            detector_->setDetectorParam(std::move(param.as_double()), 11);
            break;
        case 10:
            detector_->setDetectorParam(std::move(param.as_double()), 12);
            break;
        case 11:
            detector_->setDetectorParam(std::move(param.as_double()), 13);
            break;
        case 12:
            detector_->setDetectorParam(std::move(param.as_double()), 14);
            break;
        case 13:
            detector_->setDetectorParam(std::move(param.as_double()), 15);
            break;
        case 14:
            detector_->setDetectorParam(std::move(param.as_double()), 16);
            break;
        case 15:
            detector_->setDetectorParam(std::move(param.as_double()), 17);
            break;
        case 16:
            detector_->setDebugParam(std::move(param.as_bool()), 1);
            break;
        case 17:
            detector_->setDebugParam(std::move(param.as_bool()), 2);
            break;
        case 18:
            detector_->setDebugParam(std::move(param.as_bool()), 3);
            break;
        case 19:
            detector_->setDebugParam(std::move(param.as_bool()), 4);
            break;
        case 20:
            detector_->setDebugParam(std::move(param.as_bool()), 5);
            break;
        case 21:
            detector_->setDebugParam(std::move(param.as_bool()), 6);
            break;
        case 22:
            detector_->setDebugParam(std::move(param.as_bool()), 7);
            break;
        case 23:
            detector_->setDebugParam(std::move(param.as_bool()), 8);
            break;
        case 24:
            detector_->setDebugParam(std::move(param.as_bool()), 9);
            break;
        case 25:
            detector_->setDebugParam(std::move(param.as_bool()), 10);
            break;
        case 26:
            detector_->setDebugParam(std::move(param.as_bool()), 11);
            break;
        default:
            break;
        }
        return true;
    }

    std::unique_ptr<Detector> DetectorNode::init_detector()
    {
        params_map_ = 
        {
            {"armor_type_wh_thres", 0},
            {"max_lost_cnt", 1},
            {"max_armors_cnt", 2},
            {"max_v", 3},
            {"max_delta_t", 4},
            {"no_crop_thres", 5},
            {"hero_danger_zone", 6},
            {"color", 7},
            {"no_crop_ratio", 8},
            {"full_crop_ratio", 9},
            {"armor_roi_expand_ratio_width", 10},
            {"armor_roi_expand_ratio_height", 11},
            {"armor_conf_high_thres", 12},
            {"anti_spin_judge_high_thres", 13},
            {"anti_spin_judge_low_thres", 14},
            {"anti_spin_max_r_multiple", 15},
            {"max_dead_buffer", 16},
            {"max_delta_dist", 17},
            {"debug_without_com", 18},
            {"using_imu", 19},
            {"using_roi", 20},
            {"show_aim_cross", 21},
            {"show_img", 22},
            {"detect_red", 23},
            {"show_fps", 24},
            {"print_letency", 25},
            {"print_target_info", 26},
        };
        
        // detector params.
        this->declare_parameter<int>("armor_type_wh_thres", 3);
        this->declare_parameter<int>("max_lost_cnt", 5);
        this->declare_parameter<int>("max_armors_cnt", 8);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("max_delta_t", 100);
        this->declare_parameter<double>("no_crop_thres", 1e-2);
        this->declare_parameter<int>("hero_danger_zone", 4);
        this->declare_parameter<bool>("color", true);
        this->declare_parameter<double>("no_crop_ratio", 2e-3);
        this->declare_parameter<double>("full_crop_ratio", 1e-4);
        this->declare_parameter<double>("armor_roi_expand_ratio_width", 1.1);
        this->declare_parameter<double>("armor_roi_expand_ratio_height", 1.5);
        this->declare_parameter<double>("armor_conf_high_thres", 0.82);
        
        //TODO:Set by your own path.
        this->declare_parameter("camera_name", "00J90630561"); //相机型号
        this->declare_parameter("camera_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter("network_path", "src/vehicle_system/autoaim/armor_detector/model/opt-0527-002.xml");
        
        //debug
        this->declare_parameter("debug_without_com", true);
        this->declare_parameter("using_imu", false);
        this->declare_parameter("using_roi", true);
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_img", false);
        this->declare_parameter("detect_red", true);
        this->declare_parameter("show_fps", false);
        this->declare_parameter("print_letency", false);
        this->declare_parameter("print_target_info", false);
        
        //
        this->declare_parameter("anti_spin_judge_high_thres", 2e4);
        this->declare_parameter("anti_spin_judge_low_thres", 2e3);
        this->declare_parameter("anti_spin_max_r_multiple", 4.5);
        this->declare_parameter("max_dead_buffer", 2) ;
        this->declare_parameter("max_delta_dist", 0.3);
        
        detector_params_.armor_type_wh_thres = this->get_parameter("armor_type_wh_thres").as_int();
        detector_params_.max_lost_cnt = this->get_parameter("max_lost_cnt").as_int();
        detector_params_.max_armors_cnt = this->get_parameter("max_armors_cnt").as_int();
        detector_params_.max_v = this->get_parameter("max_v").as_int();
        detector_params_.max_delta_t = this->get_parameter("max_delta_t").as_int();
        detector_params_.no_crop_thres = this->get_parameter("no_crop_thres").as_double();
        detector_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        bool det_red = this->get_parameter("color").as_bool();
        if(det_red)
            detector_params_.color = RED;
        else
            detector_params_.color = BLUE;
        this->get_parameter("no_crop_ratio", detector_params_.no_crop_ratio);
        this->get_parameter("full_crop_ratio", detector_params_.full_crop_ratio);
        this->get_parameter("armor_roi_expand_ratio_width", detector_params_.armor_roi_expand_ratio_width);
        this->get_parameter("armor_roi_expand_ratio_height", detector_params_.armor_roi_expand_ratio_height);
        this->get_parameter("armor_conf_high_thres", detector_params_.armor_conf_high_thres);


        debug_.detect_red = this->get_parameter("detect_red").as_bool();
        debug_.debug_without_com  = this->get_parameter("debug_without_com").as_bool();
        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        debug_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_.using_roi = this->get_parameter("using_roi").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        debug_.print_letency = this->get_parameter("print_letency").as_bool();
        debug_.print_target_info = this->get_parameter("print_target_info").as_bool();

        gyro_params_.anti_spin_judge_high_thres = this->get_parameter("anti_spin_judge_high_thres").as_double();
        gyro_params_.anti_spin_judge_low_thres = this->get_parameter("anti_spin_judge_low_thres").as_double();
        gyro_params_.anti_spin_max_r_multiple = this->get_parameter("anti_spin_max_r_multiple").as_double();
        gyro_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        gyro_params_.max_dead_buffer = this->get_parameter("max_dead_buffer").as_int() ;
        gyro_params_.max_delta_dist = this->get_parameter("max_delta_dist").as_double();
        gyro_params_.max_delta_t = this->get_parameter("max_delta_t").as_int();

        std::string camera_name = this->get_parameter("camera_name").as_string();
        std::string camera_param_path = this->get_parameter("camera_param_path").as_string();
        std::string network_path = this->get_parameter("network_path").as_string();

        return std::make_unique<Detector>(camera_name, camera_param_path, network_path, detector_params_, debug_, gyro_params_);
    }

} //namespace detector

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_detector::DetectorNode>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::DetectorNode)