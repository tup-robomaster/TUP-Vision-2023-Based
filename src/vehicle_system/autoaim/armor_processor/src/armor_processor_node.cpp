/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2022-11-26 15:45:45
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor_node.cpp
 */
#include "../include/armor_processor_node.hpp"

using namespace std::placeholders;
namespace armor_processor
{
    ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions& options)
    : Node("armor_processor", options)
    {
        RCLCPP_WARN(this->get_logger(), "Starting processor node...");
        
        processor_ = init_armor_processor();
        if(!processor_->is_initialized)
        {
            processor_->coordsolver_.loadParam(processor_->coord_param_path_, processor_->coord_param_name_);
            processor_->is_initialized = true;
        }
        
        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();

        // 发布云台转动信息（pitch、yaw角度）
        gimbal_info_pub_ = this->create_publisher<global_interface::msg::Gimbal>("/gimbal_info", qos);

        // 订阅目标装甲板信息
        target_info_sub_ = this->create_subscription<global_interface::msg::Target>("/armor_info", rclcpp::SensorDataQoS(),
            std::bind(&ArmorProcessorNode::target_info_callback, this, std::placeholders::_1));

        // 参数服务器参数声明、获取
        this->declare_parameter<bool>("using_shared_memory", false);
        using_shared_memory = this->get_parameter("using_shared_memory").as_bool();
        
        // global_user::CameraType camera_type;
        this->declare_parameter<int>("camera_type", global_user::DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();
        
        // Subscriptions transport type
        transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        
        // if(processor_->armor_predictor_.debug_param_.using_imu)
        // {
        //     // declare and acquire 'target_frame' parameter
        //     this->declare_parameter("target_frame", "gyro");
        //     this->target_frame_ = this->get_parameter("target_frame").as_string();

        //     // buffer timeout
        //     SecondsType buffer_timeout(1);

        //     tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        //     // create the timer interface before call waitForTransform
        //     auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>
        //     (
        //         this->get_node_base_interface(),
        //         this->get_node_timers_interface()
        //     );

        //     tf2_buffer_->setCreateTimerInterface(timer_interface);
        //     tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        //     target_point_sub_.subscribe(this, "/armor_detector/armor_info");

        //     tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<global_interface::msg::Target>>
        //     (
        //         target_point_sub_,
        //         *tf2_buffer_,
        //         target_frame_,
        //         10,
        //         this->get_node_logging_interface(),
        //         this->get_node_clock_interface(),
        //         buffer_timeout
        //     );

        //     //register a callback
        //     tf2_filter_->registerCallback(&ArmorProcessorNode::target_info_callback, this); //获取tf关系后进入回调
        // }
        // else
        // {
        //     spin_info_sub = this->create_subscription<global_interface::msg::SpinInfo>("/spin_info", 10,
        //         std::bind(&ArmorProcessorNode::spin_info_callback, this, std::placeholders::_1));
        //     target_info_sub_ = this->create_subscription<global_interface::msg::Target>("/armor_info", rclcpp::SensorDataQoS(),
        //         std::bind(&ArmorProcessorNode::target_info_callback, this, std::placeholders::_1));
        // }
        
        bool debug_ = false;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ArmorProcessorNode::paramsCallback, this, std::placeholders::_1));
            
            if(using_shared_memory)
            {
                /**
                 * @brief 共享内存配置
                 * 
                 */
                sleep(5);

                this->key_ = ftok("./", 9);

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

                //
                this->read_memory_thread_ = std::thread(&ArmorProcessorNode::img_callback, this);
                // this->read_memory_thread_.join();
            }
            else
            {
                
                // Subscriptions transport type
                // transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

                if(camera_type == global_user::DaHeng)
                {
                    // image sub
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/daheng_img",
                    std::bind(&ArmorProcessorNode::image_callback, this, _1), transport_));
                }
                else
                {
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
                    std::bind(&ArmorProcessorNode::image_callback, this, std::placeholders::_1), transport_));
                }
            }
        }
    }

    ArmorProcessorNode::~ArmorProcessorNode()
    {
        if(using_shared_memory)
        {
            //解除共享内存映射
            if(this->shared_memory_ptr_)
            {
                if(shmdt(this->shared_memory_ptr_) == -1)
                {
                    RCLCPP_ERROR(this->get_logger(), "Dissolution remapping failed...");
                }
            }
            
            //销毁共享内存
            if(shmctl(shared_memory_id_, IPC_RMID, NULL) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Destroy shared memory failed...");        
            }
        }
    }

    void ArmorProcessorNode::img_callback()
    {
        cv::Mat img = cv::Mat(DAHENG_IMAGE_HEIGHT, DAHENG_IMAGE_WIDTH, CV_8UC3);

        while(1)
        {
            //读取共享内存图像数据
            memcpy(img.data, shared_memory_ptr_, DAHENG_IMAGE_HEIGHT * DAHENG_IMAGE_WIDTH * 3);
            // img.copyTo(src.img);
            if(this->debug_param_.show_predict)
            {
                if(!img.empty())
                {
                    // RCLCPP_INFO(this->get_logger(), "Show prediction...");
                    if(predict_point_ == last_predict_point_)
                    {}
                    else
                    {
                        last_predict_point_ = predict_point_;
                        cv::Point2f point_2d = processor_->coordsolver_.reproject(predict_point_);
                        // for(int i = 0; i < 4; i++)
                        // {
                        //     cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {255, 255, 0}, 4);
                        // }
                        std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
                        cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
                        cv::Rect rect = points_pic_rrect.boundingRect();
                        cv::rectangle(img, rect, {255, 0, 255}, 5);
                        cv::circle(img, point_2d, 8, {255, 255, 0}, -1);
                    }
                    cv::namedWindow("ekf_predict", cv::WINDOW_AUTOSIZE);
                    cv::imshow("ekf_predict", img);
                    cv::waitKey(1);
                }
            }
        }
    }

    void ArmorProcessorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        if(!img_info)
        {
            return;
        }
        // RCLCPP_INFO(this->get_logger(), "...");

        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        // img.copyTo(src.img);
        if(this->debug_param_.show_predict)
        {
            if(!img.empty())
            {
                // RCLCPP_INFO(this->get_logger(), "Show prediction...");
                if(predict_point_ == last_predict_point_)
                {}
                else
                {
                    last_predict_point_ = predict_point_;
                    cv::Point2f point_2d = processor_->coordsolver_.reproject(predict_point_);
                    // for(int i = 0; i < 4; i++)
                    // {
                    //     cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {0, 255, 0}, 4);
                    // }
                    std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
                    cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
                    cv::Rect rect = points_pic_rrect.boundingRect();
                    cv::rectangle(img, rect, {255, 0, 255}, 5);
                    cv::circle(img, point_2d, 8, {255, 255, 0}, -1);
                }
                cv::namedWindow("ekf_predict", cv::WINDOW_AUTOSIZE);
                cv::imshow("ekf_predict", img);
                cv::waitKey(1);
            }
        }
    }

    std::unique_ptr<Processor> ArmorProcessorNode::init_armor_processor()
    {
        std::string pName[PARAM_NUM] = 
        {
            "bullet_speed",
            "max_time_delta",
            "max_cost",
            "min_fitting_lens",
            "max_v",
            "shoot_delay",
            "alpha",
            "a_max",
            "p_max",
            "p0",
            "sigma",
            "dt",
            "p",
            "r",
            "disable_fitting",
            "draw_predict",
            "using_imu",
            "show_predict",
            "show_transformed_info"  
        };   
        param_names_ = pName;

        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<int>("max_time_delta", 1000);
        this->declare_parameter<int>("max_cost", 509);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("min_fitting_lens", 10);
        this->declare_parameter<int>("shoot_delay", 100);
        this->declare_parameter<int>("window_size", 3);
        predict_param_.bullet_speed = this->get_parameter("bullet_speed").as_double();
        predict_param_.max_time_delta = this->get_parameter("max_time_delta").as_int();
        predict_param_.max_cost = this->get_parameter("max_cost").as_int();
        predict_param_.max_v = this->get_parameter("max_v").as_int();
        predict_param_.min_fitting_lens = this->get_parameter("min_fitting_lens").as_int();
        predict_param_.shoot_delay = this->get_parameter("shoot_delay").as_int();
        predict_param_.window_size = this->get_parameter("window_size").as_int();

        this->declare_parameter<double>("singer_alpha", 5.0);
        this->declare_parameter<double>("singer_a_max", 10.0);
        this->declare_parameter<double>("singer_p_max", 0.5);
        this->declare_parameter<double>("singer_p0", 0.1);
        this->declare_parameter<double>("singer_sigma", 0.1);
        this->declare_parameter<double>("singer_dt", 5.0);
        this->declare_parameter<double>("singer_p", 1.0);
        this->declare_parameter<double>("singer_r", 1.0);
        singer_model_param_.alpha = this->get_parameter("singer_alpha").as_double();
        singer_model_param_.a_max = this->get_parameter("singer_a_max").as_double();
        singer_model_param_.p_max = this->get_parameter("singer_p_max").as_double();
        singer_model_param_.p0 = this->get_parameter("singer_p0").as_double();
        singer_model_param_.sigma = this->get_parameter("singer_sigma").as_double();
        singer_model_param_.dt = this->get_parameter("singer_dt").as_double();
        singer_model_param_.p = this->get_parameter("singer_p").as_double();
        singer_model_param_.r = this->get_parameter("singer_r").as_double();

        this->declare_parameter("disable_filter", false);
        this->declare_parameter("disable_fitting", true);
        this->declare_parameter("draw_predict", false);
        this->declare_parameter("using_imu", false);
        this->declare_parameter("show_predict", true);
        this->declare_parameter("show_transformed_info", false);
        debug_param_.disable_filter = this->get_parameter("disable_filter").as_bool();
        debug_param_.disable_fitting = this->get_parameter("disable_fitting").as_bool();
        debug_param_.draw_predict = this->get_parameter("draw_predict").as_bool();
        debug_param_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
        debug_param_.show_transformed_info = this->get_parameter("show_transformed_info").as_bool();

        this->declare_parameter<std::string>("filter_param_path", "src/global_user/config/filter_param.yaml");
        filter_param_path_ = this->get_parameter("filter_param_path").as_string();

        this->declare_parameter<std::string>("coord_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter<std::string>("coord_param_name", "00J90630561");
        coord_param_path_ = this->get_parameter("coord_param_path").as_string();
        coord_param_name_ = this->get_parameter("coord_param_name").as_string();

        return std::make_unique<Processor>(predict_param_, singer_model_param_, debug_param_, filter_param_path_, coord_param_path_, coord_param_name_);
    }


    void ArmorProcessorNode::msg_callback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr)
    {
        geometry_msgs::msg::PointStamped point_out;

        try
        {
            tf2_buffer_->transform(*point_ptr, point_out, target_frame_); //输出相对于target_frame_坐标系的坐标
            if(processor_->armor_predictor_.debug_param_.show_transformed_info)
            {
                RCLCPP_INFO(this->get_logger(), "Point of target in frame of %s: x:%f y:%f z:%f\n",
                    this->target_frame_,
                    point_out.point.x,
                    point_out.point.y,
                    point_out.point.z
                );
            }
        }
        catch(const std::exception& e)
        {   //print warning info
            RCLCPP_WARN(
                this->get_logger(),
                "Transforme failed : %s\n",
                e.what()
            );
        } 
    }

    void ArmorProcessorNode::target_info_callback(const global_interface::msg::Target& target_info)
    {
        //得到目标装甲板的四个角点
        for(int i = 0; i < 4; ++i)
        {
            apex2d[i].x = target_info.point2d[i].x;
            apex2d[i].y = target_info.point2d[i].y;
        }

        if(target_info.target_switched)
        {
            // RCLCPP_INFO(this->get_logger(), "Target switched...");
            Eigen::Vector3d aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
            auto angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);

            global_interface::msg::Gimbal gimbal_info;
            gimbal_info.pitch = angle[0];
            gimbal_info.yaw = angle[1]; 
            // std::cout << "pitch:" << angle[0] << " " << "yaw:" << angle[1] << std::endl;
            // std::cout << std::endl;
            
            //
            processor_->armor_predictor_.is_ekf_init = false;
            gimbal_info_pub_->publish(gimbal_info);
        }
        else
        {
            Eigen::Vector3d aiming_point;
            aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
            last_predict_point_ = predict_point_;
            auto aiming_point_world = processor_->armor_predictor_.predict(aiming_point, target_info.timestamp);
            predict_point_ = aiming_point_world;

            // Eigen::Vector3d aiming_point_cam = processor_->coordsolver_.worldToCam(aiming_point_world, processor_->rmat_imu);
            
            // global_interface::msg::Target final_point;
            // final_point.aiming_point.x = aiming_point[0];
            // final_point.aiming_point.y = aiming_point[1];
            // final_point.aiming_point.z = aiming_point[2];

            auto angle = processor_->coordsolver_.getAngle(aiming_point_world, processor_->rmat_imu);
            // //若预测出错直接陀螺仪坐标系下坐标作为打击点
            // if(isnan(angle[0]) || isnan(angle[1]))
            // {
            //     angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);
            // }

            global_interface::msg::Gimbal gimbal_info;
            gimbal_info.pitch = angle[0];
            gimbal_info.yaw = angle[1];
            gimbal_info.distance = aiming_point_world.norm();
            gimbal_info.is_switched = target_info.target_switched;
            gimbal_info.is_spinning = target_info.is_spinning;
            // std::cout << "pitch:" << angle[0] << " " << "yaw:" << angle[1] << std::endl;

            gimbal_info_pub_->publish(gimbal_info);
        }

        return;
    }

    // void ArmorProcessorNode::spin_info_callback(const global_interface::msg::SpinInfo::SharedPtr msg) const
    // {
    //     return;
    // }
    rcl_interfaces::msg::SetParametersResult ArmorProcessorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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

    bool ArmorProcessorNode::setParam(rclcpp::Parameter param)
    {   //动态调参
        for(int ii = 0; ii < PARAM_NUM; ++ii)
        {
            if(param.get_name() == (char*)param_names_[ii].data())
            {
                switch (ii)
                {
                case 0:
                    this->predict_param_.bullet_speed = param.as_double();
                    this->processor_->coordsolver_.setBulletSpeed(this->predict_param_.bullet_speed);
                    return true;
                case 1:
                    this->predict_param_.max_time_delta = param.as_double();
                    this->processor_->setMaxTimeDelta(this->predict_param_.max_time_delta);
                    return true;
                case 2:
                    this->predict_param_.max_cost = param.as_int();
                    this->processor_->setMaxCost(this->predict_param_.max_cost);
                    return true;
                case 3:
                    this->predict_param_.min_fitting_lens = param.as_int();
                    this->processor_->setMinFittingLens(this->predict_param_.min_fitting_lens);
                    return true;
                case 4:
                    this->predict_param_.max_v = param.as_int();
                    this->processor_->setMaxVelocity(this->predict_param_.max_v);
                    return true;
                case 5:
                    this->predict_param_.shoot_delay = param.as_int();
                    this->processor_->setShootDelay(this->predict_param_.shoot_delay);
                    return true;
                case 6:
                    this->singer_model_param_.alpha = param.as_double();
                    this->processor_->set_alpha(this->singer_model_param_.alpha);
                    return true;
                case 7:
                    this->singer_model_param_.a_max = param.as_double();
                    this->processor_->set_a_max(this->singer_model_param_.a_max);
                    return true;
                case 8:
                    this->singer_model_param_.p_max = param.as_double();
                    this->processor_->set_p_max(this->singer_model_param_.p_max);
                    return true;
                case 9:
                    this->singer_model_param_.p0 = param.as_double();
                    this->processor_->set_p0(this->singer_model_param_.p0);
                    return true;
                case 10:
                    this->singer_model_param_.sigma = param.as_double();
                    this->processor_->set_sigma(this->singer_model_param_.sigma);
                    return true;
                case 11:
                    this->singer_model_param_.dt = param.as_double();
                    this->processor_->set_dt(this->singer_model_param_.dt);
                    return true;
                case 12:
                    this->singer_model_param_.p = param.as_double();
                    this->processor_->set_p(this->singer_model_param_.p);
                    return true;
                case 13:
                    this->singer_model_param_.r = param.as_double();
                    this->processor_->set_r(this->singer_model_param_.r);
                    return true;
                case 14:
                    this->debug_param_.disable_fitting = param.as_bool();
                    this->processor_->disabledFitting(this->debug_param_.disable_fitting);
                    return true;
                case 15:
                    this->debug_param_.draw_predict = param.as_bool();
                    this->processor_->drawPredict(this->debug_param_.draw_predict);
                    return true;
                case 16:
                    this->debug_param_.using_imu = param.as_bool();
                    this->processor_->armor_predictor_.debug_param_.using_imu = this->debug_param_.using_imu;
                    return true;
                case 17:
                    this->debug_param_.show_predict = param.as_bool();
                    this->processor_->showPredict(this->debug_param_.show_predict);
                    return true;
                case 18:
                    this->debug_param_.show_transformed_info = param.as_bool();
                    this->processor_->showTransformedInfo(this->debug_param_.show_transformed_info);
                    return true;
                default:
                    return false;
                }
            }
        }
    }
} // armor_processor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_processor::ArmorProcessorNode>());
    rclcpp::shutdown();
    
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(armor_processor::ArmorProcessorNode)