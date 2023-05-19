/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 17:11:03
 * @LastEditTime: 2023-05-19 02:44:14
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/detector_node.cpp
 */
#include "../include/detector_node.hpp"

using namespace message_filters;
using namespace std::placeholders;
namespace armor_detector
{
    DetectorNode::DetectorNode(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options), my_sync_policy_(MySyncPolicy(3))
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
        
        // 同步通信/异步通信
        this->declare_parameter<bool>("sync_transport", false);
        bool sync_transport = this->get_parameter("sync_transport").as_bool();

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

        // target info pub.
        armor_info_pub_ = this->create_publisher<AutoaimMsg>("/armor_detector/armor_msg", qos);
        if (debug_.use_serial)
        {
            RCLCPP_INFO(this->get_logger(), "Using serial...");
            serial_msg_.imu.header.frame_id = "imu_link";
            this->declare_parameter<int>("mode", 1);
            serial_msg_.mode = this->get_parameter("mode").as_int();
            this->declare_parameter<double>("bullet_speed", 28.0);
            serial_msg_.bullet_speed = this->get_parameter("bullet_speed").as_double();
            detector_->coordsolver_.setBulletSpeed(serial_msg_.bullet_speed);            

            if (!sync_transport)
            {
                // Imu msg sub.
                serial_msg_sub_ = this->create_subscription<SerialMsg>("/serial_msg",
                    qos,
                    std::bind(&DetectorNode::sensorMsgCallback, this, _1)
                );
            }
        }

        // tf2
        // Initialize the transform broadcaster
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
         
        // Subscriptions transport type.
        std::string transport_type = "raw";
        std::string camera_topic = "/image";

        // image sub.
        if (sync_transport)
        {
            // Create serial msg subscriber.
            serial_msg_sync_sub_ = std::make_shared<message_filters::Subscriber<SerialMsg>>(this, "/serial_msg", rmw_qos);

            // Create image subscriber.
            img_msg_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, camera_topic, rmw_qos);

            // Create synchronous timer.
            my_sync_policy_.setInterMessageLowerBound(0, rclcpp::Duration(0, 1e7));
            my_sync_policy_.setInterMessageLowerBound(1, rclcpp::Duration(0, 1e7));
            my_sync_policy_.setMaxIntervalDuration(rclcpp::Duration(0, 3e7));
            // sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, SerialMsg>>(*img_msg_sync_sub_, *serial_msg_sync_sub_, 0.005);
            sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(my_sync_policy_), *img_msg_sync_sub_, *serial_msg_sync_sub_);

            // Register a callback function to process.
            sync_->registerCallback(std::bind(&DetectorNode::syncCallback, this, _1, _2));

            RCLCPP_WARN(this->get_logger(), "Synchronously...");
        }
        else
        {
            img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, camera_topic,
                std::bind(&DetectorNode::imageCallback, this, _1), transport_type, rmw_qos));
        }

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

    void DetectorNode::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const SerialMsg::ConstSharedPtr& serial_msg)
    {
        rclcpp::Time time = img_msg->header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        double dura = (now.nanoseconds() - time.nanoseconds()) / 1e6;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "delay:%.2fms", dura);
        // if ((dura) > 20.0)
        //     return;

        TaskData src;
        AutoaimMsg target_info;
        bool is_target_lost = true;

        // Convert the image to opencv format.
        // cv_bridge::CvImagePtr cv_ptr;
        try
        {
            src.img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            src.timestamp = img_msg->header.stamp.nanosec;
            src.bullet_speed = serial_msg->bullet_speed;
            target_info.shoot_delay = serial_msg->shoot_delay;
            src.mode = serial_msg->mode;
            src.quat.w() = serial_msg->imu.orientation.w;
            src.quat.x() = serial_msg->imu.orientation.x;
            src.quat.y() = serial_msg->imu.orientation.y;
            src.quat.z() = serial_msg->imu.orientation.z;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        
        try
        {
            param_mutex_.lock();
            // Target detector. 
            if (!detector_->armor_detect(src, is_target_lost))
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No target...");
            }
            else
            {   // Target spinning detector. 
                if (!detector_->gyro_detector(src, target_info))
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Not spinning...");
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Spinning detecting...");
            }
            param_mutex_.unlock();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Detector node errror: %s", e.what());
        }

        target_info.header.frame_id = "gimbal_link";
        target_info.header.stamp = img_msg->header.stamp;
        target_info.mode = src.mode;
        target_info.bullet_speed = src.bullet_speed;
        // target_info.timestamp = src.timestamp;
        target_info.quat_imu = serial_msg->imu.orientation;
        target_info.is_target_lost = is_target_lost;
        armor_info_pub_->publish(std::move(target_info));
        
        debug_.show_img = this->get_parameter("show_img").as_bool();
        if(debug_.show_img)
        {
            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
    }

    void DetectorNode::detect(TaskData& src, rclcpp::Time stamp)
    {
        AutoaimMsg target_info;
        Eigen::Vector2d tracking_angle = {0.0, 0.0};
        Eigen::Matrix3d rmat_imu = Eigen::Matrix3d::Identity();

        rclcpp::Time now = this->get_clock()->now();
        serial_msg_mutex_.lock();
        if (debug_.use_serial)
        {
            src.mode = serial_msg_.mode;
            src.bullet_speed = serial_msg_.bullet_speed;
            target_info.shoot_delay = serial_msg_.shoot_delay;
            if (debug_.use_imu)
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500, "Using imu data...");
                src.quat.w() = serial_msg_.imu.orientation.w;
                src.quat.x() = serial_msg_.imu.orientation.x;
                src.quat.y() = serial_msg_.imu.orientation.y;
                src.quat.z() = serial_msg_.imu.orientation.z;

                // std::cout << 111 << std::endl;
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 500, "Not use imu data...");
                Eigen::Matrix3d rmat = Eigen::Matrix3d::Identity();
                src.quat = Eigen::Quaterniond(rmat);
            }
            
            auto dt = (now - serial_msg_.imu.header.stamp).nanoseconds() / 1e6;
            putText(src.img, "IMU_DELAY:" + to_string(dt) + "ms", cv::Point2i(50, 80), cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255});
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "using serial...");
        }
        else 
        {
            Eigen::Matrix3d rmat = Eigen::Matrix3d::Identity();
            src.quat = Eigen::Quaterniond(rmat);
        }
        serial_msg_mutex_.unlock(); 

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "mode:%d bulletSpd:%.2f", src.mode, src.bullet_speed);
        
        param_mutex_.lock();
        if (detector_->armor_detect(src, target_info.is_target_lost))
        {   
            if (detector_->gyro_detector(src, target_info))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Spinning detecting...");
                rmat_imu = src.quat.toRotationMatrix();
                Eigen::Vector3d armor_3d_cam = {target_info.armors.front().point3d_cam.x, target_info.armors.front().point3d_cam.y, target_info.armors.front().point3d_cam.z};
                tracking_angle = detector_->coordsolver_.getAngle(armor_3d_cam, rmat_imu);
            }
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "target info_cam: %lf %lf %lf", target_info.armors.front().point3d_cam.x, target_info.armors.front().point3d_cam.y, target_info.armors.front().point3d_cam.z);
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "target info_world: %lf %lf %lf", target_info.aiming_point_world.x, target_info.aiming_point_world.y, target_info.aiming_point_world.z);
        }
        param_mutex_.unlock();

        // if (!target_info.is_target_lost)
        // {
        //     Eigen::Vector3d rpy_raw = {0, 0, 0};
        //     Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(rpy_raw[2], Eigen::Vector3d::UnitX()));
        //     Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(rpy_raw[1], Eigen::Vector3d::UnitY()));
        //     Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(rpy_raw[0], Eigen::Vector3d::UnitZ()));
        //     Eigen::Matrix3d rmat = yawAngle * pitchAngle * rollAngle;
        // }

        target_info.header.frame_id = "gimbal_link";
        target_info.header.stamp = stamp;
        target_info.quat_imu.w = src.quat.w();
        target_info.quat_imu.x = src.quat.x();
        target_info.quat_imu.y = src.quat.y();
        target_info.quat_imu.z = src.quat.z();
        target_info.bullet_speed = detector_->coordsolver_.getBulletSpeed();
        // RCLCPP_INFO(this->get_logger(), "timestamp:%.8f", target_info.timestamp / 1e9);

        // if (target_info.spinning_switched)
            // cout << "spinning_switched" << endl;
        rclcpp::Time end = this->get_clock()->now();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "detect_delay:%.2fms", (end - now).nanoseconds() / 1e6);

        armor_info_pub_->publish(std::move(target_info));

        debug_.show_img = this->get_parameter("show_img").as_bool();
        if (debug_.show_img)
        {
            char ch[50];
            sprintf(ch, "pitch_angle:%.2f yaw_angle:%.2f", tracking_angle[1], tracking_angle[0]);
            std::string angle_str = ch;
            putText(src.img, angle_str, {src.img.size().width / 2 + 50, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255});

            cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
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
        return;
    }

    /**
     * @brief 图像数据回调
     * 
     * @param img_info 图像传感器数据
     */
    void DetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        if(!img_info)
            return;
        rclcpp::Time time = img_info->header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        double dura = (now.nanoseconds() - time.nanoseconds()) / 1e6;
        // RCLCPP_WARN(this->get_logger(), "delay:%.2fms", dura);
        if ((dura) > 20.0)
            return;

        TaskData src;
        rclcpp::Time stamp = img_info->header.stamp;
        src.timestamp = stamp.nanoseconds();
        src.img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        // img.copyTo(src.img);
        // RCLCPP_INFO(this->get_logger(), "src_timestamp:%.8f", src.timestamp / 1e9);
        
        if (debug_.show_img)
        {
            char ch[25];
            sprintf(ch, "img_trans_delay:%.2fms", dura);
            std::string delay_str = ch;
            putText(src.img, delay_str, {src.img.size().width / 5 - 40, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 125, 255});
        }

        // 目标检测接口函数
        detect(src, stamp);
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
        this->declare_parameter<int>("armor_type_wh_thres", 3);
        this->declare_parameter<int>("max_lost_cnt", 5);
        this->declare_parameter<int>("max_armors_cnt", 8);
        this->declare_parameter<int>("hero_danger_zone", 4);
        this->declare_parameter<double>("no_crop_thres", 1e-2);
        this->declare_parameter<double>("no_crop_ratio", 2e-3);
        this->declare_parameter<double>("full_crop_ratio", 1e-4);
        this->declare_parameter<double>("armor_roi_expand_ratio_width", 1.1);
        this->declare_parameter<double>("armor_roi_expand_ratio_height", 1.5);
        this->declare_parameter<double>("armor_conf_high_thres", 0.82);
        
        //TODO:Set by your own path.
        this->declare_parameter("camera_name", "KE0200110075"); //相机型号
        this->declare_parameter("camera_param_path", "/config/camera.yaml");
        this->declare_parameter("network_path", "/model/opt-0527-002.xml");
        this->declare_parameter("save_path", "/data/info.txt");
        
        //Debug.
        this->declare_parameter("detect_red", true);
        this->declare_parameter("use_serial", true);
        this->declare_parameter("use_imu", true);
        this->declare_parameter("use_roi", true);
        this->declare_parameter("show_img", false);
        this->declare_parameter("show_crop_img", false);
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_fps", false);
        this->declare_parameter("print_letency", false);
        this->declare_parameter("print_target_info", false);
        this->declare_parameter("show_all_armors", false);
        this->declare_parameter("save_data", false);
        this->declare_parameter("save_dataset", false);
        
        //Gyro params.
        this->declare_parameter<int>("max_dead_buffer", 2) ;
        this->declare_parameter<int>("max_delta_t", 100);
        this->declare_parameter<double>("switch_max_dt", 10000.0);
        this->declare_parameter<double>("max_delta_dist", 0.3);
        this->declare_parameter<double>("anti_spin_judge_high_thres", 2e4);
        this->declare_parameter<double>("anti_spin_judge_low_thres", 2e3);
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
        detector_params_.max_lost_cnt = this->get_parameter("max_lost_cnt").as_int();
        detector_params_.max_armors_cnt = this->get_parameter("max_armors_cnt").as_int();
        detector_params_.no_crop_thres = this->get_parameter("no_crop_thres").as_double();
        detector_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        detector_params_.armor_type_wh_thres = this->get_parameter("armor_type_wh_thres").as_int();
        
        detector_params_.no_crop_ratio = this->get_parameter("no_crop_ratio").as_double();
        detector_params_.full_crop_ratio = this->get_parameter("full_crop_ratio").as_double();
        detector_params_.armor_conf_high_thres = this->get_parameter("armor_conf_high_thres").as_double();
        detector_params_.armor_roi_expand_ratio_width = this->get_parameter("armor_roi_expand_ratio_width").as_double();
        detector_params_.armor_roi_expand_ratio_height = this->get_parameter("armor_roi_expand_ratio_height").as_double();

        debug_.detect_red = this->get_parameter("detect_red").as_bool();
        debug_.use_serial = this->get_parameter("use_serial").as_bool();
        debug_.use_imu = this->get_parameter("use_imu").as_bool();
        debug_.use_roi = this->get_parameter("use_roi").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        debug_.show_crop_img = this->get_parameter("show_crop_img").as_bool();
        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        debug_.print_letency = this->get_parameter("print_letency").as_bool();
        debug_.print_target_info = this->get_parameter("print_target_info").as_bool();
        debug_.show_all_armors = this->get_parameter("show_all_armors").as_bool();
        debug_.save_data = this->get_parameter("save_data").as_bool();
        debug_.save_dataset = this->get_parameter("save_dataset").as_bool();

        gyro_params_.anti_spin_judge_high_thres = this->get_parameter("anti_spin_judge_high_thres").as_double();
        gyro_params_.anti_spin_judge_low_thres = this->get_parameter("anti_spin_judge_low_thres").as_double();
        gyro_params_.anti_spin_max_r_multiple = this->get_parameter("anti_spin_max_r_multiple").as_double();
        gyro_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        gyro_params_.max_dead_buffer = this->get_parameter("max_dead_buffer").as_int() ;
        gyro_params_.max_delta_dist = this->get_parameter("max_delta_dist").as_double();
        gyro_params_.max_delta_t = this->get_parameter("max_delta_t").as_int();
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