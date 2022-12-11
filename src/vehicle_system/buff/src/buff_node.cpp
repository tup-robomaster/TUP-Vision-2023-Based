/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-11 12:19:24
 * @LastEditTime: 2022-12-11 21:37:28
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/src/buff_node.cpp
 */
#include "../include/buff_node.hpp"

using namespace std::placeholders;
using namespace global_user;

namespace buff
{
    BuffNode::BuffNode(const rclcpp::NodeOptions& options)
    : Node("buff", options)
    {
        RCLCPP_INFO(this->get_logger(), "Buff node is starting...");

        try
        {
            buff_ = init_buff();
        }
        catch(const exception& e)
        {
            std::cout << e.what() << std::endl;
        }

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();

        // buff's msgs pub
        buff_info_pub_ = this->create_publisher<TargetMsg>("/buff_info", qos);
        predict_info_pub_ = this->create_publisher<TargetMsg>("/predict_buff_info", qos);

        // gimbal info pub
        gimbal_info_pub_ = this->create_publisher<GimbalMsg>("/gimbal_info", qos);

        //
        time_start_ = this->get_clock()->now();

        this->declare_parameter<int>("camera_type", DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();

        transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

        //
        this->declare_parameter<bool>("using_shared_memory", false);
        using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();

        if(!buff_->is_init_)
        {
            // buff_->detector_.initModel(network_path_);
            // buff_->coordsolver_.loadParam(camera_param_path_, camera_name_);
            // if(buff_->is_save_data)
            // {
            //     buff_->data_save.open("src/data/dis_info_1.txt", ios::out | ios::trunc);
            //     buff_->data_save << fixed;
            // }
            buff_->is_init_ = true;
        }

        if(using_shared_memory_)
        {
            //共享内存读线程
            this->key_ = ftok("./", 5);

            //获取共享内存id
            shared_memory_id_ = shmget(key_, 0, 0);
            if(shared_memory_id_ == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Get shared memory id failed...");
            }

            //映射共享内存，得到虚拟地址
            shared_memory_ptr_ = shmat(shared_memory_id_, 0, 0);
            if(shared_memory_ptr_ == (void*)-1)
            {
                RCLCPP_ERROR(this->get_logger(), "Remapping shared memory failed...");
            }

            //(s)
            this->read_memory_thread_ = std::thread(&BuffNode::run, this);
            // this->read_memory_thread_.join();
        }
        else 
        {
            // image sub
            if(camera_type == DaHeng)
            {
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/daheng_img",
                    std::bind(&BuffNode::image_callback, this, _1), transport_));
            }
            else if(camera_type == HikRobot)
            {
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
                    std::bind(&BuffNode::image_callback, this, _1), transport_));
            }
            else if(camera_type == MVSCam)
            {
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/mvs_img",
                    std::bind(&BuffNode::image_callback, this, _1), transport_));
            }
            else if(camera_type == USBCam)
            {
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/usb_img",
                    std::bind(&BuffNode::image_callback, this, _1), transport_));
            }
        }
    }

    BuffNode::~BuffNode()
    {
        if(using_shared_memory_)
            if(shared_memory_ptr_)
                if(shmdt(shared_memory_ptr_) == -1)
                    RCLCPP_ERROR(this->get_logger(), "Dissolution shared memory failed...");
    }


    void BuffNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        TaskData src;
        if(!img_info)
        {
            RCLCPP_INFO(this->get_logger(), "Image is empty...");
            return;
        }

        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);

        rclcpp::Time img_sub_time = this->get_clock()->now();

        //(ns)
        src.timestamp = img_sub_time.nanoseconds() - time_start_.nanoseconds();
        double sleep_time = 0;

        TargetInfo target_info;
        if(buff_->run(src, target_info))
        {

        }
        cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
        imshow("dst",src.img);
        waitKey(1);
    }

    void BuffNode::run()
    {

    }

    void BuffNode::param_callback()
    {
        
    }

    std::unique_ptr<Buff> BuffNode::init_buff()
    {
        this->declare_parameter<int>("max_lost_cnt", 4);
        this->declare_parameter<double>("max_v", 4.0);
        this->declare_parameter<double>("max_delta_t", 100.0);
        this->declare_parameter<double>("fan_length", 0.7);
        this->declare_parameter<double>("no_crop_thres", 2e-3);

        this->get_parameter("max_lost_cnt", this->buff_param_.max_lost_cnt);
        this->get_parameter("max_v", this->buff_param_.max_v);
        this->get_parameter("max_delta_t", this->buff_param_.max_delta_t);
        this->get_parameter("fan_length", this->buff_param_.fan_length);
        this->get_parameter("no_crop_thres", this->buff_param_.no_crop_thres);

        this->declare_parameter<std::string>("pf_path", "src/global_user/config/filter_param.yaml");
        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<double>("max_timespan", 20000);
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<double>("max_a", 8.0);
        this->declare_parameter<int>("history_deque_len_cos", 250);
        this->declare_parameter<int>("history_deque_len_phase", 100);
        this->declare_parameter<int>("history_deque_len_uniform", 100);
        this->declare_parameter<double>("delay_small", 175.0);
        this->declare_parameter<double>("delay_big", 100.0);
        this->declare_parameter<int>("window_size", 2);

        this->get_parameter("max_lost_cnt", this->predictor_param_.pf_path);
        this->get_parameter("bullet_speed", this->predictor_param_.bullet_speed);
        this->get_parameter("max_timespan", this->predictor_param_.max_timespan);
        this->get_parameter("max_v", this->predictor_param_.max_v);
        this->get_parameter("max_a", this->predictor_param_.max_a);
        this->get_parameter("history_deque_len_cos", this->predictor_param_.history_deque_len_cos);
        this->get_parameter("history_deque_len_phase", this->predictor_param_.history_deque_len_phase);
        this->get_parameter("history_deque_len_uniform", this->predictor_param_.history_deque_len_uniform);
        this->get_parameter("delay_small", this->predictor_param_.delay_small);
        this->get_parameter("delay_big", this->predictor_param_.delay_big);
        this->get_parameter("window_size", this->predictor_param_.window_size);

        this->declare_parameter<std::string>("network_path", "src/vehicle_system/buff/model/buff.xml");
        this->declare_parameter<std::string>("camera_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter<std::string>("camera_name", "KE0200110075");
        this->declare_parameter<std::string>("path_prefix", "src/vehicle_system/buff/dataset/");

        this->get_parameter("network_path", this->path_param_.network_path);
        this->get_parameter("camera_param_path", this->path_param_.camera_param_path);
        this->get_parameter("camera_name", this->path_param_.camera_name);
        this->get_parameter("path_prefix", this->path_param_.path_prefix);

        this->declare_parameter<bool>("debug_without_com", true);
        this->declare_parameter<bool>("using_imu", false);
        this->declare_parameter<bool>("using_roi", false);
        this->declare_parameter<bool>("show_img", false);
        this->declare_parameter<bool>("detect_red", false);
        this->declare_parameter<bool>("show_all_fans", true);
        this->declare_parameter<bool>("show_fps", true);
        this->declare_parameter<bool>("print_target_info", false);
        this->declare_parameter<bool>("assist_label", false);
        this->declare_parameter<bool>("detect_buff_red", true);
        this->declare_parameter<bool>("detect_buff_blue", false);
        this->declare_parameter<bool>("show_predict", false);
        this->declare_parameter<bool>("prinf_latency", false);

        this->get_parameter("debug_without_com", this->debug_param_.debug_without_com);
        this->get_parameter("using_imu", this->debug_param_.using_imu);
        this->get_parameter("using_roi", this->debug_param_.using_roi);
        this->get_parameter("show_img", this->debug_param_.show_img);
        this->get_parameter("detect_red", this->debug_param_.detect_red);
        this->get_parameter("show_all_fans", this->debug_param_.show_all_fans);
        this->get_parameter("show_fps", this->debug_param_.show_fps);
        this->get_parameter("print_target_info", this->debug_param_.print_target_info);
        this->get_parameter("assist_label", this->debug_param_.assist_label);
        this->get_parameter("detect_buff_red", this->debug_param_.detect_buff_red);
        this->get_parameter("detect_buff_blue", this->debug_param_.detect_buff_blue);
        this->get_parameter("show_predict", this->debug_param_.show_predict);
        this->get_parameter("prinf_latency", this->debug_param_.prinf_latency);

        return std::make_unique<Buff>(buff_param_, predictor_param_, path_param_, debug_param_);
    }

} //namespace buff

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<buff::BuffNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(buff::BuffNode)