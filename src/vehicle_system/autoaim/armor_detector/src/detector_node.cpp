/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 17:11:03
 * @LastEditTime: 2023-06-02 22:32:42
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
            this->detector_ = initDetector();
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(this->get_logger(), "Fatal while initializing detector class: %s", e.what());
        }

        if (!detector_->is_init_)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing network model...");
            detector_->armor_detector_.initModel(path_params_.network_path);
            detector_->coordsolver_.loadParam(path_params_.camera_param_path, path_params_.camera_name);
            if(detector_->is_save_data_)
            {
                detector_->data_save_.open(path_params_.save_path, ios::out | ios::trunc);
                detector_->data_save_ << fixed;
            }
            detector_->is_init_ = true;
        }
        
        // QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.reliable();
        qos.transient_local();
        qos.durability_volatile();
        // qos.lifespan();
        // qos.deadline();
        // qos.best_effort();
        // qos.durability();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // armor_msg pub.
        armor_msg_pub_ = this->create_publisher<AutoaimMsg>("/armor_detector/armor_msg", qos);

        // initialize serial msg
        serial_msg_.imu.header.frame_id = "imu_link";
        this->declare_parameter<int>("mode", 1);
        serial_msg_.mode = this->get_parameter("mode").as_int();
        serial_msg_.bullet_speed = 0.0;
        serial_msg_.shoot_delay = 0.0;

        // serial msg sub.
        serial_msg_sub_ = this->create_subscription<SerialMsg>(
            "/serial_msg",
            qos,
            std::bind(&DetectorNode::sensorMsgCallback, this, _1)
        );

        // Subscriptions transport type.
        std::string transport_type = "raw";
        std::string camera_topic = "/image";
        img_msg_sub_ = std::make_shared<image_transport::Subscriber>(
            image_transport::create_subscription(
                this, 
                camera_topic,
                std::bind(&DetectorNode::imageCallback, this, _1), 
                transport_type, 
                rmw_qos
            )
        );

        bool debug = false;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug);
        if (debug)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "debug...");
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DetectorNode::paramsCallback, this, _1));
        }
    }

    DetectorNode::~DetectorNode()
    {
    }

    /**
     * @brief 传感器消息回调（目前是陀螺仪数据）
     * 
     * @param serial_msg 
     */
    void DetectorNode::sensorMsgCallback(const SerialMsg& serial_msg)
    {
        serial_msg_mutex_.lock();
        serial_msg_ = serial_msg;
        serial_msg_.header.stamp = this->get_clock()->now();
        serial_msg_mutex_.unlock();
        mode_ = serial_msg.mode;
    }

    /**
     * @brief 图像数据回调
     * 
     * @param img_msg 图像传感器数据
     */
    void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        int mode = mode_;
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            200, 
            "Autoaim mode: %d",
            mode
        );

        if(!img_msg || (mode_ != AUTOAIM_NORMAL && mode_ != AUTOAIM_SLING &&
            mode_ != AUTOAIM_TRACKING && mode_ != OUTPOST_ROTATION_MODE &&
            mode_ != SENTRY_NORMAL
        ))
        {
            return;
        }

        rclcpp::Time img_stamp = img_msg->header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        double duration = (now.nanoseconds() - img_stamp.nanoseconds()) / 1e6;
        if (duration > 20.0)
            return;

        TaskData src;
        AutoaimMsg armor_msg;
        double bullet_speed = 0.0; 
        double shoot_delay = 0.0;
        Eigen::Vector2d tracking_angle = {0.0, 0.0};

        rclcpp::Time stamp = img_msg->header.stamp;
        src.timestamp = stamp.nanoseconds();
        src.img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();
        src.quat = Eigen::Quaterniond(rmat_imu);

        serial_msg_mutex_.lock();
        rclcpp::Time serial_stamp = serial_msg_.header.stamp;
        src.mode = serial_msg_.mode;
        bullet_speed = serial_msg_.bullet_speed;
        shoot_delay = serial_msg_.shoot_delay;
        if (debug_.use_imu)
        {
            src.quat.w() = serial_msg_.imu.orientation.w;
            src.quat.x() = serial_msg_.imu.orientation.x;
            src.quat.y() = serial_msg_.imu.orientation.y;
            src.quat.z() = serial_msg_.imu.orientation.z;
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Using imu data...");
        }
        serial_msg_mutex_.unlock(); 

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            100, 
            "mode:%d bulletSpd:%.2f shoot_delay:%.2f",
            src.mode, bullet_speed, shoot_delay
        );
        
        param_mutex_.lock();
        if (detector_->armor_detect(src, armor_msg.is_target_lost))
        {   
            if (detector_->gyro_detector(src, armor_msg))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Spinning detecting...");
                rmat_imu = src.quat.toRotationMatrix();
                Eigen::Vector3d armor_3d_cam = {armor_msg.armors.front().point3d_cam.x, armor_msg.armors.front().point3d_cam.y, armor_msg.armors.front().point3d_cam.z};
                tracking_angle = detector_->coordsolver_.getAngle(armor_3d_cam, rmat_imu);
                
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(), 
                    500, 
                    "armor3d_cam: (%.3f %.3f %.3f) armor3d_world: (%.3f %.3f %.3f)", 
                    armor_msg.armors.front().point3d_cam.x, armor_msg.armors.front().point3d_cam.y, armor_msg.armors.front().point3d_cam.z,
                    armor_msg.armors.front().point3d_world.x, armor_msg.armors.front().point3d_world.y, armor_msg.armors.front().point3d_world.z
                );
            }
        }
        param_mutex_.unlock();

        armor_msg.header.frame_id = "gimbal_link";
        armor_msg.header.stamp = img_msg->header.stamp;
        armor_msg.quat_imu.w = src.quat.w();
        armor_msg.quat_imu.x = src.quat.x();
        armor_msg.quat_imu.y = src.quat.y();
        armor_msg.quat_imu.z = src.quat.z();
        armor_msg.mode = mode;
        armor_msg.bullet_speed = bullet_speed;
        armor_msg.shoot_delay = shoot_delay;

        rclcpp::Time end = this->get_clock()->now();
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            100, 
            "detect_delay:%.2fms",
            (end - now).nanoseconds() / 1e6
        );

        armor_msg_pub_->publish(std::move(armor_msg));

        debug_.show_img = this->get_parameter("show_img").as_bool();
        if (debug_.show_img)
        {
            char ch1[25];
            sprintf(ch1, "img_trans_delay:%.2fms", duration);
            std::string delay_str1 = ch1;
            putText(src.img, delay_str1, {src.img.size().width / 5 - 40, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 125, 255});
        
            char ch2[50];
            sprintf(ch2, "pitch_angle:%.2f yaw_angle:%.2f", tracking_angle[1], tracking_angle[0]);
            std::string angle_str2 = ch2;
            putText(src.img, angle_str2, {src.img.size().width / 2 + 50, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255});

            double dt = (now.nanoseconds() - serial_stamp.nanoseconds()) / 1e6;
            putText(src.img, "IMU_DELAY:" + to_string(dt) + "ms", cv::Point2i(50, 80), cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255});

            cv::namedWindow("armor_dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("armor_dst", src.img);
            cv::waitKey(1);
        }
    }

    /**
     * @brief 参数回调函数
     * 
     * @param params 参数服务器参数（发生改变的参数）
     * @return rcl_interfaces::msg::SetParametersResult 
     */
    rcl_interfaces::msg::SetParametersResult DetectorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    { 
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        result.successful = updateParam();
        
        param_mutex_.lock();
        detector_->detector_params_ = this->detector_params_;
        detector_->spinning_detector_.gyro_params_ = this->gyro_params_;
        detector_->debug_params_ = this->debug_;
        param_mutex_.unlock();
        return result;
    }

    /**
     * @brief 初始化detector类
     * 
     * @return std::unique_ptr<Detector> 
     */
    std::unique_ptr<Detector> DetectorNode::initDetector()
    {
        //Detector params.
        this->declare_parameter<int>("color", 1);
        this->declare_parameter<int>("max_lost_cnt", 5);
        this->declare_parameter<int>("max_armors_cnt", 8);
        this->declare_parameter<int>("max_dead_buffer", 5) ;
        this->declare_parameter<int>("max_delta_t", 100);
        
        this->declare_parameter<double>("armor_type_wh_high_thresh", 3.0);
        this->declare_parameter<double>("armor_type_wh_low_thresh", 2.5);
        this->declare_parameter<double>("hero_danger_zone", 4.0);
        this->declare_parameter<double>("no_crop_thresh", 1e-2);
        this->declare_parameter<double>("no_crop_ratio", 2e-3);
        this->declare_parameter<double>("full_crop_ratio", 1e-4);
        this->declare_parameter<double>("armor_roi_expand_ratio_width", 1.1);
        this->declare_parameter<double>("armor_roi_expand_ratio_height", 1.5);
        this->declare_parameter<double>("armor_conf_high_thresh", 0.82);
        
        //TODO:Set by your own path.
        this->declare_parameter("camera_name", "KE0200110075"); //相机型号
        this->declare_parameter("camera_param_path", "/config/camera.yaml");
        this->declare_parameter("network_path", "/model/opt-0527-002.xml");
        this->declare_parameter("save_path", "/data/info.txt");
        
        //Debug.
        this->declare_parameter("use_imu", true);
        this->declare_parameter("use_roi", true);
        this->declare_parameter("show_img", false);
        this->declare_parameter("show_crop_img", false);
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_fps", false);
        this->declare_parameter("print_latency", false);
        this->declare_parameter("print_target_info", false);
        this->declare_parameter("show_all_armors", false);
        this->declare_parameter("save_data", false);
        this->declare_parameter("save_dataset", false);
        
        //Gyro params.
        this->declare_parameter<double>("switch_max_dt", 10000.0);
        this->declare_parameter<double>("max_delta_dist", 0.3);
        this->declare_parameter<double>("anti_spin_judge_high_thresh", 2e4);
        this->declare_parameter<double>("anti_spin_judge_low_thresh", 2e3);
        this->declare_parameter<double>("anti_spin_max_r_multiple", 4.5);
        
        //Update param from param server.
        updateParam();

        return std::make_unique<Detector>(path_params_, detector_params_, debug_, gyro_params_);
    }

    /**
     * @brief 更新参数
     * 
     * @return true 
     * @return false 
     */
    bool DetectorNode::updateParam()
    {
        detector_params_.color = this->get_parameter("color").as_int();
        gyro_params_.max_delta_t = this->get_parameter("max_delta_t").as_int();
        detector_params_.max_lost_cnt = this->get_parameter("max_lost_cnt").as_int();
        gyro_params_.max_dead_buffer = this->get_parameter("max_dead_buffer").as_int();
        detector_params_.max_armors_cnt = this->get_parameter("max_armors_cnt").as_int();

        detector_params_.no_crop_thresh = this->get_parameter("no_crop_thresh").as_double();
        detector_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_double();
        detector_params_.armor_type_wh_high_thresh = this->get_parameter("armor_type_wh_high_thresh").as_double();
        detector_params_.armor_type_wh_low_thresh = this->get_parameter("armor_type_wh_low_thresh").as_double();
        detector_params_.no_crop_ratio = this->get_parameter("no_crop_ratio").as_double();
        detector_params_.full_crop_ratio = this->get_parameter("full_crop_ratio").as_double();
        detector_params_.armor_conf_high_thresh = this->get_parameter("armor_conf_high_thresh").as_double();
        detector_params_.armor_roi_expand_ratio_width = this->get_parameter("armor_roi_expand_ratio_width").as_double();
        detector_params_.armor_roi_expand_ratio_height = this->get_parameter("armor_roi_expand_ratio_height").as_double();

        debug_.use_imu = this->get_parameter("use_imu").as_bool();
        debug_.use_roi = this->get_parameter("use_roi").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        debug_.show_crop_img = this->get_parameter("show_crop_img").as_bool();
        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        debug_.print_latency = this->get_parameter("print_latency").as_bool();
        debug_.print_target_info = this->get_parameter("print_target_info").as_bool();
        debug_.show_all_armors = this->get_parameter("show_all_armors").as_bool();
        debug_.save_data = this->get_parameter("save_data").as_bool();
        debug_.save_dataset = this->get_parameter("save_dataset").as_bool();

        gyro_params_.anti_spin_judge_high_thresh = this->get_parameter("anti_spin_judge_high_thresh").as_double();
        gyro_params_.anti_spin_judge_low_thresh = this->get_parameter("anti_spin_judge_low_thresh").as_double();
        gyro_params_.anti_spin_max_r_multiple = this->get_parameter("anti_spin_max_r_multiple").as_double();
        gyro_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_double();
        gyro_params_.max_delta_dist = this->get_parameter("max_delta_dist").as_double();
        gyro_params_.switch_max_dt = this->get_parameter("switch_max_dt").as_double();

        string pkg_share_directory[2] = 
        {
            {get_package_share_directory("global_user")}, 
            {get_package_share_directory("armor_detector")}
        };
        path_params_.camera_name = this->get_parameter("camera_name").as_string();
        path_params_.camera_param_path = pkg_share_directory[0] + this->get_parameter("camera_param_path").as_string();
        path_params_.network_path = pkg_share_directory[1] + this->get_parameter("network_path").as_string();
        path_params_.save_path = pkg_share_directory[0] + this->get_parameter("save_path").as_string();

        return true;
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