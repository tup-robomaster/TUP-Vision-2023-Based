/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 17:11:03
 * @LastEditTime: 2022-12-02 14:59:53
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/armor_detector/detector_node.cpp
 */
#include "../../include/armor_detector/detector_node.hpp"

using std::placeholders::_1;

namespace armor_detector
{
    detector_node::detector_node(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
    {
        RCLCPP_WARN(this->get_logger(), "Starting detector node...");

        try
        {
            //detector类初始化
            this->detector_ = init_detector();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        target_ptr = new TargetInfo();
        target_ptr->system_model = IMMMODEL;
        
        //
        processor_ = init_armor_processor();
        if(!processor_->is_initialized)
        {
            processor_->coordsolver_.loadParam(processor_->coord_param_path_, processor_->coord_param_name_);
            processor_->is_initialized = true;
        }

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();
        
        // armors pub
        armors_pub = this->create_publisher<TargetMsg>("/armor_info", qos);
        predict_info_pub = this->create_publisher<TargetMsg>("/predict_info", qos);

        //
        gimbal_info_pub_ = this->create_publisher<global_interface::msg::Gimbal>("/gimbal_info", qos);

        time_start = std::chrono::steady_clock::now();
        
        // global_user::CameraType camera_type;
        this->declare_parameter<int>("camera_type", global_user::DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();

        transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        
        //
        this->declare_parameter("using_shared_memory", false);
        using_shared_memory = this->get_parameter("using_shared_memory").as_bool();
        if(using_shared_memory)
        {
            /**
             * @brief 共享内存配置
             * 
             */
            sleep(5);
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
            this->read_memory_thread_ = std::thread(&detector_node::run, this);
            // this->read_memory_thread_.join();
        }
        else
        {
            // Subscriptions transport type

            //image sub
            // rclcpp::QoS qos(0);
            // qos.keep_last(1);
            // qos.best_effort();
            // qos.reliable();
            // qos.durability();
            // // qos.transient_local();
            // qos.durability_volatile();

            if(camera_type == global_user::DaHeng)
            {
                img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/daheng_img",
                    std::bind(&detector_node::image_callback, this, _1), transport_));
            }
            else
            {
                img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
                    std::bind(&detector_node::image_callback, this, _1), transport_));
            }
        }
        // std::cout << 1 << std::endl;

        // param callback
        // param_timer_ = this->create_wall_timer(1000ms, std::bind(&detector_node::param_callback, this));
        
        // std::cout << 2 << std::endl;
    }

    detector_node::~detector_node()
    {
        //解除映射
        if(shared_memory_ptr_)
        {
            if(shmdt(shared_memory_ptr_) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Dissolution shared memory failed...");
            }
        }
    }

    void detector_node::run()
    {
        global_user::TaskData src;
        std::vector<Armor> armors;

        Mat img = Mat(DAHENG_IMAGE_HEIGHT, DAHENG_IMAGE_WIDTH, CV_8UC3);
        // std::cout << 3 << std::endl;

        while(1)
        {
            // std::cout << 4 << std::endl;
            //读取共享内存图像数据
            memcpy(img.data, shared_memory_ptr_, DAHENG_IMAGE_HEIGHT * DAHENG_IMAGE_WIDTH * 3);
            img.copyTo(src.img);

            // RCLCPP_INFO(this->get_logger(), "...");

            // auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
            // img.copyTo(src.img);
            // cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
            // cv::imshow("image", src.img);
            // cv::waitKey(0);

            TimePoint time_img_sub = std::chrono::steady_clock::now();
            src.timestamp = (int)(std::chrono::duration<double, std::milli>(time_img_sub - time_start).count());
            
            if(detector_->armor_detect(src))
            {   //find armors
                // RCLCPP_INFO(this->get_logger(), "armors detector...");
                TargetMsg target_info;
                TargetMsg predict_info;
                Eigen::Vector3d aiming_point;
                //target's spinning status detect 
                if(detector_->gyro_detector(src, target_info, target_ptr))
                {
                    if(target_info.target_switched)
                    {
                        // RCLCPP_INFO(this->get_logger(), "Target switched...");
                        aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
                        
                        // std::cout << "aiming_point: " << aiming_point[0] << " " << aiming_point[1] << " " << aiming_point[2] << std::endl;
                        
                        auto angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);

                        global_interface::msg::Gimbal gimbal_info;
                        gimbal_info.pitch = angle[0];
                        gimbal_info.yaw = angle[1]; 

                        predict_info = target_info;
                        // std::cout << "pitch:" << angle[0] << " " << "yaw:" << angle[1] << std::endl;
                        // std::cout << std::endl;

                        //
                        processor_->armor_predictor_.is_ekf_init = false;
                        gimbal_info_pub_->publish(gimbal_info);
                    }
                    else
                    {
                        // Eigen::Vector3d aiming_point;
                        aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};

                        // std::cout << "aiming_point: " << aiming_point[0] << " " << aiming_point[1] << " " << aiming_point[2] << std::endl;
                        // last_predict_point_ = predict_point_;
                        // TargetInfoPtr target_ptr;
                        target_ptr->xyz = aiming_point;
                        // std::cout << 24 << std::endl;
                        aiming_point = processor_->armor_predictor_.predict(src.img, target_ptr, target_info.timestamp);
                        // predict_point_ = aiming_point_world;
                        // std::cout << 2 << std::endl;
                        // std::cout << "aiming_point_world: " << aiming_point_world[0] << " " << aiming_point_world[1] << " " << aiming_point_world[2] << std::endl;

                        // Eigen::Vector3d aiming_point_cam = processor_->coordsolver_.worldToCam(aiming_point_world, processor_->rmat_imu);
                        
                        // global_interface::msg::Target final_point;
                        // final_point.aiming_point.x = aiming_point[0];
                        // final_point.aiming_point.y = aiming_point[1];
                        // final_point.aiming_point.z = aiming_point[2];

                        auto angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);
                        // //若预测出错直接陀螺仪坐标系下坐标作为打击点
                        // if(isnan(angle[0]) || isnan(angle[1]))
                        // {
                        //     angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);
                        // }

                        global_interface::msg::Gimbal gimbal_info;
                        gimbal_info.pitch = angle[0];
                        gimbal_info.yaw = angle[1];
                        gimbal_info.distance = aiming_point.norm();
                        gimbal_info.is_switched = target_info.target_switched;
                        gimbal_info.is_spinning = target_info.is_spinning;
                        
                        predict_info.aiming_point.x = aiming_point[0];
                        predict_info.aiming_point.y = aiming_point[1];
                        predict_info.aiming_point.z = aiming_point[2];
                        predict_info.target_switched = target_info.target_switched;
                        predict_info.is_spinning = target_info.is_spinning;
                        // std::cout << "pitch:" << angle[0] << " " << "yaw:" << angle[1] << std::endl;

                        gimbal_info_pub_->publish(gimbal_info);
                    }

                    cv::Point2f point_2d = processor_->coordsolver_.reproject(aiming_point);
                    // for(int i = 0; i < 4; i++)
                    // {
                    //     cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {0, 255, 0}, 4);
                    // }
                    // std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
                    // cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
                    // cv::Rect rect = points_pic_rrect.boundingRect();
                    // cv::rectangle(img, rect, {255, 0, 255}, 5);

                    cv::circle(src.img, point_2d, 10, {255, 255, 0}, -1);
                    
                    // global_interface::msg::Target target_info;
                    // target_info.aiming_point.x = aiming_point_cam[0];
                    // target_info.aiming_point.y = aiming_point_cam[1];
                    // target_info.aiming_point.z = aiming_point_cam[2];
                    target_info.timestamp = src.timestamp;

                    //publish target's information containing 3d point and timestamp.
                    armors_pub->publish(target_info);
                    predict_info_pub->publish(predict_info);
                }
                
                auto time_predict = std::chrono::steady_clock::now();
                double dr_full_ms = std::chrono::duration<double,std::milli>(time_predict - detector_->time_start).count();
                putText(src.img, fmt::format("FPS: {}", int(1000 / dr_full_ms)), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
                // std::cout << 5 << std::endl;
                cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
                cv::imshow("dst", src.img);
                cv::waitKey(1);
            }
        }
    }
    
    void detector_node::param_callback()
    {
        //
        getParameters();
        this->detector_->debugParams(detector_params_, debug_, gyro_params_);
    }

    void detector_node::getParameters()
    {
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
    }

    std::unique_ptr<detector> detector_node::init_detector()
    {
        //detector params
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
        
        //TODO:set by own path
        this->declare_parameter("camera_name", "00J90630561"); //相机型号
        this->declare_parameter("camera_param_path", "src/global_user/config/camera.yaml");
        this->declare_parameter("network_path", "src/vehicle_system/autoaim/armor_detector/model/yolox.xml");
        
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
        // this->declare_parameter("hero_danger_zone", 99);
        this->declare_parameter("max_dead_buffer", 2) ;
        this->declare_parameter("max_delta_dist", 0.3);
        // this->declare_parameter("max_delta_t", 50);
        
        //
        getParameters();

        std::string camera_name = this->get_parameter("camera_name").as_string();
        std::string camera_param_path = this->get_parameter("camera_param_path").as_string();
        std::string network_path = this->get_parameter("network_path").as_string();

        return std::make_unique<detector>(camera_name, camera_param_path, network_path, detector_params_, debug_, gyro_params_);
    }

    std::unique_ptr<Processor> detector_node::init_armor_processor()
    {
        // std::cout << 1 << std::endl;

        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<int>("max_time_delta", 1000);
        this->declare_parameter<int>("max_cost", 509);
        // this->declare_parameter<int>("max_v", 8);
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
        // std::cout << 2 << std::endl;

        this->declare_parameter("disable_filter", false);
        this->declare_parameter("disable_fitting", true);
        this->declare_parameter("draw_predict", false);
        // this->declare_parameter("using_imu", false);
        this->declare_parameter("show_predict", true);
        this->declare_parameter("show_transformed_info", false);
        debug_param_.disable_filter = this->get_parameter("disable_filter").as_bool();
        debug_param_.disable_fitting = this->get_parameter("disable_fitting").as_bool();
        debug_param_.draw_predict = this->get_parameter("draw_predict").as_bool();
        debug_param_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
        debug_param_.show_transformed_info = this->get_parameter("show_transformed_info").as_bool();

        // std::cout << 3 << std::endl;
               
        this->declare_parameter<std::string>("filter_param_path", "src/global_user/config/filter_param.yaml");
        filter_param_path_ = this->get_parameter("filter_param_path").as_string();

        this->declare_parameter<std::string>("coord_param_name", "KE0200110075");
        this->declare_parameter<std::string>("coord_param_path", "src/global_user/config/camera.yaml");
        coord_param_path_ = this->get_parameter("coord_param_path").as_string();
        coord_param_name_ = this->get_parameter("coord_param_name").as_string();

        return std::make_unique<Processor>(predict_param_, singer_model_param_, debug_param_, filter_param_path_, coord_param_path_, coord_param_name_);
    }

    void detector_node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        global_user::TaskData src;
        std::vector<Armor> armors;

        if(!img_info)
        {
            return;
        }
        
        // RCLCPP_INFO(this->get_logger(), "...");

        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);
        // cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
        // cv::imshow("image", src.img);
        // cv::waitKey(0);

        TimePoint time_img_sub = std::chrono::steady_clock::now();
        src.timestamp = (int)(std::chrono::duration<double, std::milli>(time_img_sub - time_start).count());

        if(detector_->armor_detect(src))
        {   //find armors
            // RCLCPP_INFO(this->get_logger(), "armors detector...");
            TargetMsg target_info;
            TargetMsg predict_info;
            Eigen::Vector3d aiming_point;
            //target's spinning status detect 
            if(detector_->gyro_detector(src, target_info, target_ptr))
            {
                if(target_info.target_switched)
                {
                    // std::cout << 26 << std::endl;

                    // RCLCPP_INFO(this->get_logger(), "Target switched...");
                    aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
                    
                    // std::cout << "aiming_point: " << aiming_point[0] << " " << aiming_point[1] << " " << aiming_point[2] << std::endl;
                    
                    auto angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);

                    global_interface::msg::Gimbal gimbal_info;
                    gimbal_info.pitch = angle[0];
                    gimbal_info.yaw = angle[1]; 
                    // std::cout << "pitch:" << angle[0] << " " << "yaw:" << angle[1] << std::endl;
                    // std::cout << std::endl;
                    // std::cout << "1" << std::endl;
                    predict_info = target_info;

                    //
                    processor_->armor_predictor_.is_ekf_init = false;
                    processor_->armor_predictor_.is_imm_init = false;
                    // std::cout << 28 << std::endl;
                    // gimbal_info_pub_->publish(gimbal_info);
                }
                else
                {
                    // std::cout << 27 << std::endl;

                    // Eigen::Vector3d aiming_point;
                    aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};

                    // std::cout << "aiming_point: " << aiming_point[0] << " " << aiming_point[1] << " " << aiming_point[2] << std::endl;
                    // last_predict_point_ = predict_point_;
                    // TargetInfoPtr target_ptr = new TargetInfo();
                    target_ptr->xyz = aiming_point;
                    // target_ptr->

                    // std::cout << 25 << std::endl;
                    aiming_point = processor_->armor_predictor_.predict(src.img, target_ptr, src.timestamp);
                    // predict_point_ = aiming_point_world;

                    // std::cout << 3 << std::endl;

                    // std::cout << "aiming_point_world: " << aiming_point_world[0] << " " << aiming_point_world[1] << " " << aiming_point_world[2] << std::endl;

                    // Eigen::Vector3d aiming_point_cam = processor_->coordsolver_.worldToCam(aiming_point_world, processor_->rmat_imu);
                    
                    // global_interface::msg::Target final_point;
                    // final_point.aiming_point.x = aiming_point[0];
                    // final_point.aiming_point.y = aiming_point[1];
                    // final_point.aiming_point.z = aiming_point[2];

                    auto angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);
                    // //若预测出错直接陀螺仪坐标系下坐标作为打击点
                    // if(isnan(angle[0]) || isnan(angle[1]))
                    // {
                    //     angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);
                    // }

                    global_interface::msg::Gimbal gimbal_info;
                    gimbal_info.pitch = angle[0];
                    gimbal_info.yaw = angle[1];
                    gimbal_info.distance = aiming_point.norm();
                    gimbal_info.is_switched = target_info.target_switched;
                    gimbal_info.is_spinning = target_info.is_spinning;
                    
                    predict_info.aiming_point.x = aiming_point[0];
                    predict_info.aiming_point.y = aiming_point[1];
                    predict_info.aiming_point.z = aiming_point[2];
                    predict_info.target_switched = target_info.target_switched;
                    predict_info.is_spinning = target_info.is_spinning;
                    // std::cout << "2" << std::endl;

                    // std::cout << "pitch:" << angle[0] << " " << "yaw:" << angle[1] << std::endl;

                    gimbal_info_pub_->publish(gimbal_info);
                }

                cv::Point2f point_2d = processor_->coordsolver_.reproject(aiming_point);
                // for(int i = 0; i < 4; i++)
                // {
                //     cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {0, 255, 0}, 4);
                // }
                // std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
                // cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
                // cv::Rect rect = points_pic_rrect.boundingRect();
                // cv::rectangle(img, rect, {255, 0, 255}, 5);

                auto time_predict = std::chrono::steady_clock::now();
                double dr_full_ms = std::chrono::duration<double,std::milli>(time_predict - detector_->time_start).count();
                putText(src.img, fmt::format("FPS: {}", int(1000 / dr_full_ms)), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
                cv::circle(src.img, point_2d, 10, {255, 255, 0}, -1);
                
                // global_interface::msg::Target target_info;
                // target_info.aiming_point.x = aiming_point_cam[0];
                // target_info.aiming_point.y = aiming_point_cam[1];
                // target_info.aiming_point.z = aiming_point_cam[2];
                target_info.timestamp = src.timestamp;

                //publish target's information containing 3d point and timestamp.
                armors_pub->publish(target_info);
                predict_info_pub->publish(predict_info);
            }
            else
            {
                // std::cout << 4 << std::endl;
            }
            // std::cout << 5 << std::endl;
        }
        else
        {
            // std::cout << 6 << std::endl;
        }
        
        cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
        cv::imshow("dst", src.img);
        cv::waitKey(1);
    }
} //namespace detector

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_detector::detector_node>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::detector_node)