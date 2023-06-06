/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:08:00
 * @LastEditTime: 2023-06-06 21:14:07
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
            detector_ = initDetector();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing detector class: %s", e.what());
        }

        detector_->is_initialized_ = false;
        if (!detector_->is_initialized_)
        {
            RCLCPP_INFO(this->get_logger(), "Initializing detector class");
            detector_->buff_detector_.initModel(path_param_.network_path);
            detector_->coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            detector_->is_initialized_ = true;
        }

        // QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.reliable();
        qos.durability();
        // qos.best_effort();
        // qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // buff info pub.
        buff_msg_pub_ = this->create_publisher<BuffMsg>("/buff_detector/buff_msg", qos);

        // initialize serial msg. 
        serial_msg_.imu.header.frame_id = "imu_link2";
        this->declare_parameter<int>("buff_mode", 3);
        serial_msg_.mode = this->get_parameter("buff_mode").as_int();
        serial_msg_.bullet_speed = 0.0;
        serial_msg_.shoot_delay = 0.0;
        mode_ = serial_msg_.mode;

        // serial msg sub.
        serial_msg_sub_= this->create_subscription<SerialMsg>(
            "/serial_msg", 
            rclcpp::SensorDataQoS(),
            std::bind(&BuffDetectorNode::sensorMsgCallback, this, _1)
        );

        std::string transport_type = "raw";
        std::string image_topic = "/image";

        // image sub.
        img_msg_sub_ = std::make_shared<image_transport::Subscriber>(
            image_transport::create_subscription(
                this, 
                image_topic,
                std::bind(&BuffDetectorNode::imageCallback, this, _1), 
                transport_type, 
                rmw_qos
            )
        );

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

    void BuffDetectorNode::sensorMsgCallback(const SerialMsg& serial_msg)
    {
        serial_mutex_.lock();
        serial_msg_ = serial_msg;
        serial_msg_.header.stamp = this->get_clock()->now();
        serial_mutex_.unlock();
        mode_ = serial_msg.mode;
        return;
    }
    
    void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {   
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            100, 
            "buff_mode: %d",
            mode_
        );

        if (!img_msg || (mode_ != SMALL_BUFF && mode_ != BIG_BUFF))
            return;
        
        TaskData src;
        auto img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        img.copyTo(src.img);
        
        rclcpp::Time stamp = img_msg->header.stamp;
        src.timestamp = stamp.nanoseconds();

        BuffMsg buff_msg;
        double shoot_delay = 0.0;
        double bullet_speed = 0.0;

        serial_mutex_.lock();
        src.mode = serial_msg_.mode;
        bullet_speed = serial_msg_.bullet_speed;
        shoot_delay = serial_msg_.shoot_delay;
        if (debug_param_.using_imu)
        {
            src.quat.w() = serial_msg_.imu.orientation.w;
            src.quat.x() = serial_msg_.imu.orientation.x;
            src.quat.y() = serial_msg_.imu.orientation.y;
            src.quat.z() = serial_msg_.imu.orientation.z;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Using imu...");
        }
        else
        {
            Eigen::Matrix3d rmat = Eigen::Matrix3d::Identity();
            src.quat = Eigen::Quaterniond(rmat);
        }
        serial_mutex_.unlock();
        
        TargetInfo target_info;
        param_mutex_.lock();
        if (detector_->run(src, target_info))
        {
            buff_msg.timestamp = src.timestamp;
            buff_msg.rotate_speed = target_info.rotate_speed;
            buff_msg.target_switched = target_info.target_switched;

            Eigen::Quaterniond quat_world = Eigen::Quaterniond(target_info.rmat);
            buff_msg.quat_world.w = quat_world.w();
            buff_msg.quat_world.x = quat_world.x();
            buff_msg.quat_world.y = quat_world.y();
            buff_msg.quat_world.z = quat_world.z();

            buff_msg.r_center.x = target_info.r_center[0];
            buff_msg.r_center.y = target_info.r_center[1];
            buff_msg.r_center.z = target_info.r_center[2];
            buff_msg.armor3d_world.x = target_info.armor3d_world[0];
            buff_msg.armor3d_world.y = target_info.armor3d_world[1];
            buff_msg.armor3d_world.z = target_info.armor3d_world[2];
            buff_msg.armor3d_cam.x = target_info.armor3d_cam[0];
            buff_msg.armor3d_cam.y = target_info.armor3d_cam[1];
            buff_msg.armor3d_cam.z = target_info.armor3d_cam[2];

            // point2d
            buff_msg.points2d[0].x = target_info.points2d[0].x;
            buff_msg.points2d[0].y = target_info.points2d[0].y;
            buff_msg.points2d[1].x = target_info.points2d[1].x;
            buff_msg.points2d[1].y = target_info.points2d[1].y;
            buff_msg.points2d[2].x = target_info.points2d[2].x;
            buff_msg.points2d[2].y = target_info.points2d[2].y;
            buff_msg.points2d[3].x = target_info.points2d[3].x;
            buff_msg.points2d[3].y = target_info.points2d[3].y;
            buff_msg.points2d[4].x = target_info.points2d[4].x;
            buff_msg.points2d[4].y = target_info.points2d[4].y;

            last_rotate_speed_ = buff_msg.rotate_speed;
            last_center3d_ = target_info.r_center;
            last_point3d_cam_ = target_info.armor3d_cam;
            last_point3d_world_ = target_info.armor3d_world;
        }
        else
        {
            buff_msg.rotate_speed = last_rotate_speed_;
            buff_msg.r_center.x = last_center3d_(0);
            buff_msg.r_center.y = last_center3d_(1);
            buff_msg.r_center.z = last_center3d_(2);
            buff_msg.armor3d_cam.x = last_point3d_cam_(0);
            buff_msg.armor3d_cam.y = last_point3d_cam_(1);
            buff_msg.armor3d_cam.z = last_point3d_cam_(2);
            buff_msg.armor3d_world.x = last_point3d_world_(0);
            buff_msg.armor3d_world.y = last_point3d_world_(1);
            buff_msg.armor3d_world.z = last_point3d_world_(2);

            last_rotate_speed_ = 0.0;
            last_center3d_ = {0.0, 0.0, 0.0};
            last_point3d_cam_ = {0.0, 0.0, 0.0};
            last_point3d_world_ = {0.0, 0.0, 0.0};
        }
        param_mutex_.unlock();
        
        // Publish buff msg.
        buff_msg.header.frame_id = "gimbal_link2";
        buff_msg.header.stamp = img_msg->header.stamp;
        buff_msg.mode = src.mode;
        buff_msg.timestamp = src.timestamp;
        buff_msg.bullet_speed = bullet_speed;
        buff_msg.shoot_delay = shoot_delay;
        buff_msg.quat_imu.w = src.quat.w();
        buff_msg.quat_imu.x = src.quat.x();
        buff_msg.quat_imu.y = src.quat.y();
        buff_msg.quat_imu.z = src.quat.z();
        buff_msg.is_target_lost = target_info.find_target ? false : true;
        buff_msg_pub_->publish(std::move(buff_msg));

        bool show_img = this->get_parameter("show_img").as_bool();
        if (show_img)
        {
            putText(
                src.img, 
                target_info.find_target ? "State:Detected" : "State:Lost" , 
                {5, 55}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0}
            );
            cv::namedWindow("dst", cv::WINDOW_NORMAL);
            cv::imshow("dst", src.img);
            cv::waitKey(1);
        }
    }

    rcl_interfaces::msg::SetParametersResult BuffDetectorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        result.successful = updateParam();
        param_mutex_.lock();
        detector_->buff_param_ = this->buff_param_;
        detector_->debug_param_ = this->debug_param_;
        param_mutex_.unlock();
        
        return result;
    }


    std::unique_ptr<Detector> BuffDetectorNode::initDetector()
    {
        this->declare_parameter<int>("color", 1);
        this->declare_parameter<int>("max_lost_cnt", 4);
        this->declare_parameter<double>("fan_length", 0.7);
        this->declare_parameter<double>("max_delta_t", 100.0);
        this->declare_parameter<double>("max_v", 4.0);
        this->declare_parameter<double>("no_crop_thres", 2e-3);

        this->declare_parameter<std::string>("camera_name", "KE0200110075");
        this->declare_parameter<std::string>("camera_param_path", "/config/camera.yaml");
        this->declare_parameter<std::string>("network_path", "/model/buff.xml");
        this->declare_parameter<std::string>("path_prefix", "/recorder/buff_dataset/");
        
        string pkg_share_pth[3] = 
        {
            get_package_share_directory("global_user"), 
            get_package_share_directory("buff_detector"), 
            get_package_share_directory("camera_driver")
        };
        this->path_param_.camera_name = this->get_parameter("camera_name").as_string();
        this->path_param_.camera_param_path = pkg_share_pth[0] + this->get_parameter("camera_param_path").as_string();
        this->path_param_.network_path = pkg_share_pth[1] + this->get_parameter("network_path").as_string();
        this->path_param_.path_prefix = pkg_share_pth[2] + this->get_parameter("path_prefix").as_string();

        this->declare_parameter<bool>("use_imu", false);
        this->declare_parameter<bool>("use_roi", false);
        this->declare_parameter<bool>("show_img", false);
        this->declare_parameter<bool>("show_all_fans", true);
        this->declare_parameter<bool>("show_fps", true);
        this->declare_parameter<bool>("assist_label", false);
        this->declare_parameter<bool>("prinf_latency", false);
        this->declare_parameter<bool>("print_target_info", false);

        bool success = updateParam();
        if(success)
            RCLCPP_INFO(this->get_logger(), "Update param!");
       
        return std::make_unique<Detector>(buff_param_, path_param_, debug_param_);
    }

    /**
     * @brief 更新参数
     * 
     * @return true 
     * @return false 
     */
    bool BuffDetectorNode::updateParam()
    {
        //Buff param.
        this->get_parameter("color", this->buff_param_.color);
        this->get_parameter("max_v", this->buff_param_.max_v);
        this->get_parameter("fan_length", this->buff_param_.fan_length);
        this->get_parameter("max_delta_t", this->buff_param_.max_delta_t);
        this->get_parameter("max_lost_cnt", this->buff_param_.max_lost_cnt);
        this->get_parameter("no_crop_thres", this->buff_param_.no_crop_thres);

        //Debug param.
        this->get_parameter("use_imu", this->debug_param_.using_imu);
        this->get_parameter("use_roi", this->debug_param_.using_roi);
        this->get_parameter("show_img", this->debug_param_.show_img);
        this->get_parameter("show_all_fans", this->debug_param_.show_all_fans);
        this->get_parameter("show_fps", this->debug_param_.show_fps);
        this->get_parameter("assist_label", this->debug_param_.assist_label);
        this->get_parameter("prinf_latency", this->debug_param_.prinf_latency);
        this->get_parameter("print_target_info", this->debug_param_.print_target_info);

        return true;
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