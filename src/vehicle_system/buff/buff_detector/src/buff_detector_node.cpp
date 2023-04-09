/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:08:00
 * @LastEditTime: 2023-03-28 18:29:49
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

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // buff info pub.
        buff_info_pub_ = this->create_publisher<BuffMsg>("buff_detector", qos);

        if(debug_param_.using_imu)
        {
            RCLCPP_INFO(this->get_logger(), "Using imu...");
            serial_msg_.imu.header.frame_id = "imu_link";
            serial_msg_.bullet_speed = this->declare_parameter<double>("bullet_speed", 28.0);
            serial_msg_.mode = this->declare_parameter<int>("buff_mode", 3);
            // imu msg sub.
            serial_msg_sub_ = this->create_subscription<SerialMsg>("/serial_msg", rclcpp::SensorDataQoS(),
                std::bind(&BuffDetectorNode::sensorMsgCallback, this, _1));
        }

        this->declare_parameter<int>("camera_type", DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();
        std::string transport = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        
        image_size_ = image_info_.image_size_map[camera_type];
        // image sub.
        std::string camera_topic = image_info_.camera_topic_map[camera_type];
        img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, camera_topic,
            std::bind(&BuffDetectorNode::imageCallback, this, _1), transport, rmw_qos));

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
        serial_msg_.imu.header.stamp = this->get_clock()->now();
        if(serial_msg.bullet_speed > 10)
            serial_msg_.bullet_speed = serial_msg.bullet_speed;
        if(serial_msg.mode == 3 || serial_msg.mode == 4)
            serial_msg_.mode = serial_msg.mode;
        serial_msg_.imu = serial_msg.imu;
        serial_mutex_.unlock();

        RCLCPP_INFO(this->get_logger(), "bullet speed: %lfm/s mode: %d", serial_msg_.bullet_speed, serial_msg_.mode);
        return;
    }
    
    void BuffDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
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

        serial_mutex_.lock();
        if(debug_param_.using_imu)
        {
            double dt = (this->get_clock()->now() - serial_msg_.imu.header.stamp).nanoseconds();
            if(abs(dt / 1e9) > 0.1)
            {
                detector_->debug_param_.using_imu = false;
                Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
                buff_msg.quat_imu.w = q.w();
                buff_msg.quat_imu.x = q.x();
                buff_msg.quat_imu.y = q.y();
                buff_msg.quat_imu.z = q.z();
                RCLCPP_WARN(this->get_logger(), "latency: %lf", dt);
            }
            else
            {
                src.bullet_speed = serial_msg_.bullet_speed;
                src.mode = serial_msg_.mode;
                src.quat.w() = serial_msg_.imu.orientation.w;
                src.quat.x() = serial_msg_.imu.orientation.x;
                src.quat.y() = serial_msg_.imu.orientation.y;
                src.quat.z() = serial_msg_.imu.orientation.z;
                buff_msg.quat_imu = serial_msg_.imu.orientation;
                detector_->debug_param_.using_imu = true;
            }
        }
        serial_mutex_.unlock();
        
        if(!debug_param_.using_imu)
        {
            //debug
            buff_msg.mode = this->get_parameter("debug_mode").as_int(); //小符
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "debug mode is: %d", (int)buff_msg.mode);
        }

        param_mutex_.lock();
        if(detector_->run(src, target_info))
        {
            param_mutex_.unlock();
            buff_msg.header.frame_id = "gimbal_link";
            buff_msg.header.stamp = img_info->header.stamp;
            buff_msg.r_center.x = target_info.r_center[0];
            buff_msg.r_center.y = target_info.r_center[1];
            buff_msg.r_center.z = target_info.r_center[2];
            buff_msg.timestamp = src.timestamp;
            buff_msg.rotate_speed = target_info.rotate_speed;
            buff_msg.target_switched = target_info.target_switched;
            Eigen::Quaterniond quat(target_info.rmat);
            buff_msg.quat_cam.w = quat.w();
            buff_msg.quat_cam.x = quat.x();
            buff_msg.quat_cam.y = quat.y();
            buff_msg.quat_cam.z = quat.z();
            buff_msg.armor3d_world.x = target_info.armor3d_world[0];
            buff_msg.armor3d_world.y = target_info.armor3d_world[1];
            buff_msg.armor3d_world.z = target_info.armor3d_world[2];
            
            // publish buff info.
            buff_info_pub_->publish(std::move(buff_msg));
        }
        else
            param_mutex_.unlock();

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
        this->declare_parameter<std::string>("camera_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter<std::string>("network_path", "src/vehicle_system/buff/model/buff.xml");
        this->declare_parameter<std::string>("path_prefix", "src/vehicle_system/buff/dataset/");
        this->get_parameter("camera_name", this->path_param_.camera_name);
        this->get_parameter("camera_param_path", this->path_param_.camera_param_path);
        this->get_parameter("network_path", this->path_param_.network_path);
        this->get_parameter("path_prefix", this->path_param_.path_prefix);

        this->declare_parameter<int>("debug_mode", 3);
        this->declare_parameter<bool>("assist_label", false);
        this->declare_parameter<bool>("detect_red", true);
        this->declare_parameter<bool>("prinf_latency", false);
        this->declare_parameter<bool>("print_target_info", false);
        this->declare_parameter<bool>("show_all_fans", true);
        this->declare_parameter<bool>("show_fps", true);
        this->declare_parameter<bool>("using_imu", false);
        this->declare_parameter<bool>("using_roi", false);
        this->declare_parameter<bool>("show_img", false);

        bool success = updateParam();
        if(success)
            RCLCPP_INFO(this->get_logger(), "Update param!");

        return std::make_unique<Detector>(buff_param_, path_param_, debug_param_);
    }

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
        this->get_parameter("using_imu", this->debug_param_.using_imu);
        this->get_parameter("using_roi", this->debug_param_.using_roi);
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