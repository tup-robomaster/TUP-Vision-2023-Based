/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2023-01-06 21:42:36
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor_node.cpp
 */
#include "../include/armor_processor_node.hpp"

using namespace std::placeholders;
namespace armor_processor
{
    ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions& options)
    : Node("armor_processor", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting processor node...");
        
        processor_ = init_armor_processor();
        if(!processor_->is_init)
        {
            RCLCPP_INFO(this->get_logger(), "Loading param...");
            processor_->loadParam(path_param_.filter_path);
            processor_->init(path_param_.coord_path, path_param_.coord_name);
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
        gimbal_info_pub_ = this->create_publisher<GimbalMsg>("/armor_processor/gimbal_info", qos);

        // 订阅目标装甲板信息
        target_info_sub_ = this->create_subscription<AutoaimMsg>("/armor_info", qos,
            std::bind(&ArmorProcessorNode::target_info_callback, this, _1));

        // 是否使用共享内存
        this->declare_parameter<bool>("using_shared_memory", false);
        using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();
        
        // 相机类型
        this->declare_parameter<int>("camera_type", global_user::DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();
        
        // 图像的传输方式
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

        //     // Register a callback func.
        //     tf2_filter_->registerCallback(&ArmorProcessorNode::target_info_callback, this); //获取tf关系后进入回调
        // }
        // else
        // {
        //     spin_info_sub = this->create_subscription<global_interface::msg::SpinInfo>("/spin_info", 10,
        //         std::bind(&ArmorProcessorNode::spin_info_callback, this, std::placeholders::_1));
        //     target_info_sub_ = this->create_subscription<global_interface::msg::Target>("/armor_info", rclcpp::SensorDataQoS(),
        //         std::bind(&ArmorProcessorNode::target_info_callback, this, std::placeholders::_1));
        // }
        
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            
            // Prediction info pub.
            predict_info_pub_ = this->create_publisher<AutoaimMsg>("/autoaim/predict_info", qos);

            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ArmorProcessorNode::paramsCallback, this, _1));
            
            if(using_shared_memory_)
            {
                RCLCPP_INFO(this->get_logger(), "Using shared memory...");
                sleep(5);
                // 共享内存配置
                if(!getSharedMemory(shared_memory_param_, 5))
                    RCLCPP_ERROR(this->get_logger(), "Shared memory init failed...");

                // 图像读取线程
                this->read_memory_thread_ = std::thread(&ArmorProcessorNode::img_callback, this);
                // this->read_memory_thread_.join();
            }
            else
            {
                if(camera_type == DaHeng)
                {
                    image_width = DAHENG_IMAGE_WIDTH;
                    image_height = DAHENG_IMAGE_HEIGHT;
                    
                    // daheng image sub.
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/daheng_img",
                        std::bind(&ArmorProcessorNode::image_callback, this, _1), transport_));
                }
                else if(camera_type == HikRobot)
                {
                    image_width = HIK_IMAGE_WIDTH;
                    image_height = HIK_IMAGE_HEIGHT;

                    // hik image sub.
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
                        std::bind(&ArmorProcessorNode::image_callback, this, _1), transport_));
                }
                else if(camera_type == MVSCam)
                {
                    image_width = MVS_IMAGE_WIDTH;
                    image_height = MVS_IMAGE_HEIGHT;

                    // mvs image sub.
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/mvs_img",
                        std::bind(&ArmorProcessorNode::image_callback, this, _1), transport_));
                }
                else if(camera_type == USBCam)
                {
                    image_width = USB_IMAGE_WIDTH;
                    image_height = USB_IMAGE_HEIGHT;

                    // usb image sub.
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/usb_img", 
                        std::bind(&ArmorProcessorNode::image_callback, this, _1), transport_));
                }
            }
        }
    }

    ArmorProcessorNode::~ArmorProcessorNode()
    {
        if(using_shared_memory_)
        {
            if(!destorySharedMemory(shared_memory_param_))
                RCLCPP_ERROR(this->get_logger(), "Destory shared memory failed...");
        }
    }

    void ArmorProcessorNode::img_callback()
    {
        cv::Mat img = cv::Mat(this->image_height, this->image_width, CV_8UC3);

        while(1)
        {
            // 读取共享内存图像数据
            memcpy(img.data, shared_memory_param_.shared_memory_ptr, this->image_height * this->image_width * 3);
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
            return;

        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        // img.copyTo(src.img);
        if(this->debug_param_.show_predict)
        {
            if(!img.empty())
            {
                // RCLCPP_INFO(this->get_logger(), "Show prediction...");
                if(!flag_)
                {}
                else
                {
                    flag_ = false;
                    // last_predict_point_ = predict_point_;
                    cv::Point2f point_2d = processor_->coordsolver_.reproject(predict_point_);
                    for(int i = 0; i < 4; i++)
                    {
                        cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {0, 255, 255}, 5);
                    }
                    // std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
                    // cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
                    // cv::Rect rect = points_pic_rrect.boundingRect();
                    // cv::rectangle(img, rect, {255, 0, 255}, 5);
                    cv::circle(img, point_2d, 10, {255, 255, 0}, -1);
                }
                cv::namedWindow("ekf_predict", cv::WINDOW_AUTOSIZE);
                cv::imshow("ekf_predict", img);
                cv::waitKey(1);
            }
        }
    }

    // void ArmorProcessorNode::msg_callback(const geometry_msgs::msg::PointStamped::SharedPtr point_ptr)
    // {
    //     geometry_msgs::msg::PointStamped point_out;

    //     try
    //     {
    //         tf2_buffer_->transform(*point_ptr, point_out, target_frame_); //输出相对于target_frame_坐标系的坐标
    //         if(processor_->armor_predictor_.debug_param_.show_transformed_info)
    //         {
    //             RCLCPP_INFO(this->get_logger(), "Point of target in frame of %s: x:%f y:%f z:%f\n",
    //                 this->target_frame_,
    //                 point_out.point.x,
    //                 point_out.point.y,
    //                 point_out.point.z
    //             );
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {   // Print warning info.
    //         RCLCPP_WARN(this->get_logger(), "Transforme failed : %s\n", e.what());
    //     } 
    // }

    void ArmorProcessorNode::target_info_callback(const AutoaimMsg& target_info)
    {
        flag_ = true;
        last_predict_point_ = predict_point_;

        if(this->debug_param_.show_predict)
        {
            // Get target 2d cornor points.
            for(int i = 0; i < 4; ++i)
            {
                apex2d[i].x = target_info.point2d[i].x;
                apex2d[i].y = target_info.point2d[i].y;
            }
        }

        double sleep_time = 0.0;
        AutoaimMsg target = std::move(target_info);
        
        auto aiming_point_world = std::move(processor_->predictor(target, sleep_time));

        Eigen::Matrix3d rmat_imu;
        if(!debug_param_.using_imu)
        {
            rmat_imu = Eigen::Matrix3d::Identity();
        }
        else
        {
            Eigen::Quaternion quat_imu = std::move(Eigen::Quaternion{target.quat_imu.w, target.quat_imu.x, target.quat_imu.y, target.quat_imu.z});
            rmat_imu = quat_imu.toRotationMatrix();
        }
        Eigen::Vector3d aiming_point_cam = processor_->coordsolver_.worldToCam(*aiming_point_world, rmat_imu);
        // std::cout << "predict_cam: x:" << aiming_point_cam[0] << " y:" << aiming_point_cam[1] << " z:" << aiming_point_cam[2] << std::endl;
        
        Eigen::Vector2d angle = processor_->coordsolver_.getAngle(aiming_point_cam, rmat_imu);

        // if(target_info.target_switched)
        // {
        //     aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
        //     // angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);
        //     processor_->is_ekf_initialized_ = false;
        // }
        // else
        // {
        //     aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
        //     last_predict_point_ = predict_point_;
        //     Eigen::Vector3d aiming_point_world = processor_->armor_predictor_.predict(aiming_point, target_info.timestamp);
        //     Eigen::Vector3d aiming_point_cam = processor_->coordsolver_.worldToCam(aiming_point_world, processor_->rmat_imu);
        //     predict_point_ = aiming_point_cam;
        //     angle = processor_->coordsolver_.getAngle(aiming_point_cam, processor_->rmat_imu);
        //     // 若预测出错直接陀螺仪坐标系下坐标作为打击点
        //     if(isnan(angle[0]) || isnan(angle[1]))
        //     {
        //         angle = processor_->coordsolver_.getAngle(aiming_point_world, processor_->rmat_imu);
        //     }
        // }

        // Gimbal info pub.
        GimbalMsg gimbal_info;
        gimbal_info.header.frame_id = "barrel_link";
        // auto now = this->get_clock()->now();
        gimbal_info.header.stamp = target_info.header.stamp;
        gimbal_info.pitch = angle[0];
        gimbal_info.yaw = angle[1];
        gimbal_info.distance = aiming_point_cam.norm();
        gimbal_info.is_switched = target_info.target_switched;
        gimbal_info.is_spinning = target_info.is_spinning;
        gimbal_info_pub_->publish(std::move(gimbal_info));

        if(this->debug_)
        {
            AutoaimMsg predict_info;
            predict_info.header.frame_id = "camera_link";
            predict_info.header.stamp = target_info.header.stamp;
            predict_info.aiming_point_cam.x = aiming_point_cam[0];
            predict_info.aiming_point_cam.y = aiming_point_cam[1];
            predict_info.aiming_point_cam.z = aiming_point_cam[2];
            predict_info.period = target_info.period;
            predict_info_pub_->publish(std::move(predict_info));
        }

        predict_point_ = aiming_point_cam;
        if(!(&target_info.image))
            return;

        std::shared_ptr<sensor_msgs::msg::Image> img_info = std::make_shared<sensor_msgs::msg::Image>(target_info.image);
        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        if(this->debug_param_.show_predict)
        {
            if(!img.empty())
            {
                // RCLCPP_INFO(this->get_logger(), "Show prediction...");
                // last_predict_point_ = predict_point_;
                cv::Point2f point_2d = processor_->coordsolver_.reproject(predict_point_);
                for(int i = 0; i < 4; i++)
                {
                    cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {255, 0, 255}, 5);
                }
                // std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
                // cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
                // cv::Rect rect = points_pic_rrect.boundingRect();
                // cv::rectangle(img, rect, {255, 0, 255}, 5);
                cv::circle(img, point_2d, 10, {255, 255, 0}, -1);
                cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
                cv::imshow("dst", img);
                cv::waitKey(1);
            }
        }
        return;
    }

    // void ArmorProcessorNode::spin_info_callback(const global_interface::msg::SpinInfo::SharedPtr msg) const
    // {
    //     return;
    // }

    std::unique_ptr<Processor> ArmorProcessorNode::init_armor_processor()
    {
        params_map_ = 
        {
            {"bullet_speed", 0},
            {"max_delta_time", 1},
            {"max_cost", 2},
            {"min_fitting_lens", 3},
            {"max_v", 4},
            {"shoot_delay", 5},
            {"singer_alpha", 6},
            {"singer_a_max", 7},
            {"singer_p_max", 8},
            {"singer_p0", 9},
            {"singer_sigma", 10},
            {"singer_dt", 11},
            {"singer_p", 12},
            {"singer_r", 13},
            {"disable_fitting", 14},
            {"draw_predict", 15},
            {"using_imu", 16},
            {"show_predict", 17},
            {"show_transformed_info", 18}
        };

        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<int>("max_time_delta", 1000);
        this->declare_parameter<int>("max_cost", 509);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("min_fitting_lens", 10);
        this->declare_parameter<int>("shoot_delay", 100);
        this->declare_parameter<int>("window_size", 3);
        predict_param_.bullet_speed = this->get_parameter("bullet_speed").as_double();
        predict_param_.max_delta_time = this->get_parameter("max_time_delta").as_int();
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
        singer_model_param_.singer_alpha = this->get_parameter("singer_alpha").as_double();
        singer_model_param_.singer_a_max = this->get_parameter("singer_a_max").as_double();
        singer_model_param_.singer_p_max = this->get_parameter("singer_p_max").as_double();
        singer_model_param_.singer_p0 = this->get_parameter("singer_p0").as_double();
        singer_model_param_.singer_sigma = this->get_parameter("singer_sigma").as_double();
        singer_model_param_.singer_dt = this->get_parameter("singer_dt").as_double();
        singer_model_param_.singer_p = this->get_parameter("singer_p").as_double();
        singer_model_param_.singer_r = this->get_parameter("singer_r").as_double();

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
        this->declare_parameter<std::string>("coord_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter<std::string>("coord_param_name", "00J90630561");

        path_param_.coord_name = this->get_parameter("coord_param_name").as_string();
        path_param_.coord_path = this->get_parameter("coord_param_path").as_string();
        path_param_.filter_path = this->get_parameter("filter_param_path").as_string();

        return std::make_unique<Processor>(predict_param_, singer_model_param_, path_param_, debug_param_);
    }

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

    /**
     * @brief 动态调参
     * @param 参数服务器参数
     * @return 是否修改参数成功
    */
    bool ArmorProcessorNode::setParam(rclcpp::Parameter param)
    {   // 动态调参(与rqt_reconfigure一块使用)
        auto param_idx = params_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->predict_param_.bullet_speed = param.as_double();
            // this->processor_->coordsolver_.setBulletSpeed(this->predict_param_.bullet_speed);
            break;
        case 1:
            this->predict_param_.max_delta_time = param.as_double();
            this->processor_->setPredictParam(this->predict_param_.max_delta_time, 1);
            break;
        case 2:
            this->predict_param_.max_cost = param.as_int();
            this->processor_->setPredictParam(this->predict_param_.max_cost, 2);
            break;
        case 3:
            this->predict_param_.min_fitting_lens = param.as_int();
            this->processor_->setPredictParam(this->predict_param_.min_fitting_lens, 3);
            break;
        case 4:
            this->predict_param_.max_v = param.as_int();
            this->processor_->setPredictParam(this->predict_param_.max_v, 4);
            break;
        case 5:
            this->predict_param_.shoot_delay = param.as_int();
            this->processor_->setPredictParam(this->predict_param_.shoot_delay, 5);
            break;
        case 6:
            this->singer_model_param_.singer_alpha = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_alpha, 1);
            break;
        case 7:
            this->singer_model_param_.singer_a_max = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_a_max, 2);
            break;
        case 8:
            this->singer_model_param_.singer_p_max = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_p_max, 3);
            break;
        case 9:
            this->singer_model_param_.singer_p0 = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_p0, 4);
            break;
        case 10:
            this->singer_model_param_.singer_sigma = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_sigma, 5);
            break;
        case 11:
            this->singer_model_param_.singer_dt = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_dt, 6);
            break;
        case 12:
            this->singer_model_param_.singer_p = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_p, 7);
            break;
        case 13:
            this->singer_model_param_.singer_r = param.as_double();
            this->processor_->setSingerParam(this->singer_model_param_.singer_r, 8);
            break;
        case 14:
            this->debug_param_.disable_fitting = param.as_bool();
            this->processor_->setDebugParam(this->debug_param_.disable_fitting, 1);
            break;
        case 15:
            this->debug_param_.draw_predict = param.as_bool();
            this->processor_->setDebugParam(this->debug_param_.draw_predict, 2);
            break;
        case 16:
            this->debug_param_.using_imu = param.as_bool();
            this->processor_->setDebugParam(this->debug_param_.using_imu, 3);
            break;
        case 17:
            this->debug_param_.show_predict = param.as_bool();
            this->processor_->setDebugParam(this->debug_param_.show_predict, 4);
            break;
        case 18:
            this->debug_param_.show_transformed_info = param.as_bool();
            this->processor_->setDebugParam(this->debug_param_.show_transformed_info, 5);
            break;
        default:
            break;
        }
        return true;
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