/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:08:00
 * @LastEditTime: 2023-03-20 12:13:43
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/test/src/buff_detector_node.cpp
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
        qos.reliable();
        qos.durability();
        // qos.durability_volatile();
        // qos.best_effort();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // tf2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // buff info pub.
        buff_info_pub_ = this->create_publisher<BuffMsg>("/buff_detector/buff_msg", qos);

        if(debug_param_.use_serial)
        {
            RCLCPP_INFO(this->get_logger(), "Using imu...");
            this->declare_parameter<double>("bullet_speed", 28.0);
            serial_msg_.bullet_speed = this->get_parameter("bullet_speed").as_double();
            
            this->declare_parameter<int>("buff_mode", 3);
            serial_msg_.mode = this->get_parameter("buff_mode").as_int();
            
            // imu msg sub.
            imu_info_sub_ = this->create_subscription<SerialMsg>("/serial_msg", rclcpp::SensorDataQoS(),
                std::bind(&BuffDetectorNode::sensorMsgCallback, this, _1));
        }

        std::string transport_type = "raw";
        std::string image_topic = "/image";

        // image sub.
        img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, image_topic,
            std::bind(&BuffDetectorNode::imageCallback, this, _1), transport_type, rmw_qos));

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

        RCLCPP_INFO(this->get_logger(), "bullet speed: %lfm/s mode: %d", serial_msg_.bullet_speed, serial_msg_.mode);
        return;
    }
    
    void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {   
        // RCLCPP_INFO(this->get_logger(), "Image callback...");
        rclcpp::Time time = img_msg->header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        double dura = (now.nanoseconds() - time.nanoseconds()) / 1e6;
        if ((dura) > 20.0)
            return;

        TaskData src;
        BuffMsg buff_msg;
        Eigen::Vector3d translation = {0.0, 0.0, 0.0};
        Eigen::Quaterniond quat_gimbal;
        Eigen::Matrix3d rmat_gimbal;
        double bullet_speed = 0.0;
        double shoot_delay = 0.0;
        rclcpp::Time serial_msg_stamp;

        src.img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        rclcpp::Time stamp = img_msg->header.stamp;
        src.timestamp = stamp.nanoseconds();

        // compute transformations
        std::string fromFrameRel = "camera_link";
        std::string toFrameRel = "base_link";

        // Look up for the transformation between target_frame and turtle2 frames
        // and send velocity commands for turtle2 to reach target_frame
        geometry_msgs::msg::TransformStamped t;
        try 
        {
            t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        } 
        catch (const tf2::TransformException & ex) 
        {
            RCLCPP_INFO(
                this->get_logger(), 
                "Could not transform %s to %s: %s",
                toFrameRel.c_str(), 
                fromFrameRel.c_str(), 
                ex.what()
            );
            return;
        }
        
        quat_gimbal.x() = t.transform.rotation.x;
        quat_gimbal.y() = t.transform.rotation.y;
        quat_gimbal.z() = t.transform.rotation.z;
        quat_gimbal.w() = t.transform.rotation.w;
        // rotation
        rmat_gimbal = quat_gimbal.toRotationMatrix();
        // translation
        translation = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};
        
        src.rmat_gimbal = rmat_gimbal;
        src.translation = translation;
        
       if (debug_param_.use_serial)
        {
            serial_mutex_.lock();
            src.mode = serial_msg_.mode;
            bullet_speed = serial_msg_.bullet_speed;
            shoot_delay = serial_msg_.shoot_delay;
            serial_msg_stamp = serial_msg_.header.stamp;
            serial_mutex_.unlock(); 
        }
        
        TargetInfo buff_info;
        param_mutex_.lock();
        if(detector_->run(src, buff_info))
        {
            buff_msg.r_center.x = buff_info.r_center[0];
            buff_msg.r_center.y = buff_info.r_center[1];
            buff_msg.r_center.z = buff_info.r_center[2];
            buff_msg.timestamp = src.timestamp;
            buff_msg.angle = buff_info.angle;
            buff_msg.angle_offset = buff_info.angle_offset;
            buff_msg.target_switched = buff_info.target_switched;
            buff_msg.delta_angle = buff_info.delta_angle;
            buff_msg.bullet_speed = bullet_speed;
            buff_msg.shoot_delay = shoot_delay;
            buff_msg.transform_gimbal = t;

            buff_msg.quat_cam.w = buff_info.quat_cam.w();
            buff_msg.quat_cam.x = buff_info.quat_cam.x();
            buff_msg.quat_cam.y = buff_info.quat_cam.y();
            buff_msg.quat_cam.z = buff_info.quat_cam.z();
            
            buff_msg.armor3d_world.x = buff_info.armor3d_world[0];
            buff_msg.armor3d_world.y = buff_info.armor3d_world[1];
            buff_msg.armor3d_world.z = buff_info.armor3d_world[2];
            buff_msg.armor3d_cam.x = buff_info.armor3d_cam[0];
            buff_msg.armor3d_cam.y = buff_info.armor3d_cam[1];
            buff_msg.armor3d_cam.z = buff_info.armor3d_cam[2];
            
            buff_msg.points2d[0].x = buff_info.points2d[0].x;
            buff_msg.points2d[0].y = buff_info.points2d[0].y;
            buff_msg.points2d[1].x = buff_info.points2d[1].x;
            buff_msg.points2d[1].y = buff_info.points2d[1].y;
            buff_msg.points2d[2].x = buff_info.points2d[2].x;
            buff_msg.points2d[2].y = buff_info.points2d[2].y;
            buff_msg.points2d[3].x = buff_info.points2d[3].x;
            buff_msg.points2d[3].y = buff_info.points2d[3].y;
            buff_msg.points2d[4].x = buff_info.points2d[4].x;
            buff_msg.points2d[4].y = buff_info.points2d[4].y;
        }
        param_mutex_.unlock();

        //Publish buff msg.
        buff_msg.header.frame_id = "gimbal_link1";
        buff_msg.header.stamp = this->get_clock()->now();
        buff_msg.is_target_lost = buff_info.find_target ? false : true;
        buff_info_pub_->publish(std::move(buff_msg));

        bool show_img = this->get_parameter("show_img").as_bool();
        if (show_img)
        {
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
        this->declare_parameter<double>("fan_length", 0.7);
        this->declare_parameter<double>("max_delta_t", 100.0);
        this->declare_parameter<int>("max_lost_cnt", 4);
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

        this->declare_parameter<bool>("assist_label", false);
        this->declare_parameter<bool>("detect_red", true);
        this->declare_parameter<bool>("prinf_latency", false);
        this->declare_parameter<bool>("print_target_info", false);
        this->declare_parameter<bool>("show_all_fans", true);
        this->declare_parameter<bool>("show_fps", true);
        this->declare_parameter<bool>("use_serial", false);
        this->declare_parameter<bool>("use_roi", false);
        this->declare_parameter<bool>("show_img", false);

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
        this->get_parameter("max_v", this->buff_param_.max_v);
        this->get_parameter("fan_length", this->buff_param_.fan_length);
        this->get_parameter("max_delta_t", this->buff_param_.max_delta_t);
        this->get_parameter("max_lost_cnt", this->buff_param_.max_lost_cnt);
        this->get_parameter("no_crop_thres", this->buff_param_.no_crop_thres);

        //Debug param.
        this->get_parameter("assist_label", this->debug_param_.assist_label);
        this->get_parameter("detect_red", this->debug_param_.detect_red);
        this->get_parameter("prinf_latency", this->debug_param_.prinf_latency);
        this->get_parameter("print_target_info", this->debug_param_.print_target_info);
        this->get_parameter("show_all_fans", this->debug_param_.show_all_fans);
        this->get_parameter("show_fps", this->debug_param_.show_fps);
        this->get_parameter("use_serial", this->debug_param_.use_serial);
        this->get_parameter("use_roi", this->debug_param_.use_roi);
        this->get_parameter("show_img", this->debug_param_.show_img);

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