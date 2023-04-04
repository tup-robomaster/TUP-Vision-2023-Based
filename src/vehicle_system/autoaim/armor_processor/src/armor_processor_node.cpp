/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2023-04-04 00:09:59
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor_node.cpp
 */
#include "../include/armor_processor_node.hpp"

using namespace std::placeholders;
namespace armor_processor
{
    ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions& options)
    : Node("armor_processor", options), my_sync_policy_(MySyncPolicy(5))
    {
        RCLCPP_INFO(this->get_logger(), "Starting processor node...");
        
        flag_ = false;
        processor_ = initArmorProcessor();
        if (!processor_->is_init_)
        {
            RCLCPP_INFO(this->get_logger(), "Loading param...");

            // Eigen::Vector2d angle_offset = {0.0, 0.0};
            // angle_offset[0] = this->get_parameter("yaw_angle_offset").as_double();
            // angle_offset[1] = this->get_parameter("pitch_angle_offset").as_double();
            
            processor_->loadParam(path_param_.filter_path);
            processor_->init(path_param_.coord_path, path_param_.coord_name);
            // processor_->is_init_ = true;
            // if (!updateAngleOffset())
            // {
            //     RCLCPP_WARN(this->get_logger(), "Warning while update angle offset param...");
            // }
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.durability();
        qos.reliable();
        // qos.best_effort();
        // qos.transient_local();
        // qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // 发布云台转动信息（pitch、yaw角度）
        gimbal_info_pub_ = this->create_publisher<GimbalMsg>("/armor_processor/gimbal_msg", qos);
        tracking_info_pub_ = this->create_publisher<GimbalMsg>("/armor_processor/tracking_msg", qos);

        this->declare_parameter<bool>("sync_transport", false);
        sync_transport_ = this->get_parameter("sync_transport").as_bool();
        if(!sync_transport_)
        {
            // 订阅目标装甲板信息
            target_info_sub_ = this->create_subscription<AutoaimMsg>(
                "/armor_detector/armor_msg",
                qos,
                std::bind(&ArmorProcessorNode::targetMsgCallback, this, _1));
        }
        // 相机类型
        this->declare_parameter<int>("camera_type", DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();
        
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            // Prediction info pub.
            predict_info_pub_ = this->create_publisher<AutoaimMsg>("/armor_processor/predict_msg", qos);
            if (debug_param_.show_img)
            {
                image_size_ = image_info_.image_size_map[camera_type];
                std::string camera_topic = image_info_.camera_topic_map[camera_type];
                if (sync_transport_)
                {
                    RCLCPP_WARN(this->get_logger(), "Synchronously...");
                    my_sync_policy_.setInterMessageLowerBound(0, rclcpp::Duration(0, 3e5));
                    my_sync_policy_.setInterMessageLowerBound(1, rclcpp::Duration(0, 3e5));
                    my_sync_policy_.setMaxIntervalDuration(rclcpp::Duration(0, 3e7));
                    img_msg_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, camera_topic, rmw_qos);
                    target_msg_sync_sub_ = std::make_shared<message_filters::Subscriber<AutoaimMsg>>(this, "/armor_detector/armor_msg", rmw_qos);
                    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(my_sync_policy_), *img_msg_sync_sub_, *target_msg_sync_sub_);
                    sync_->registerCallback(std::bind(&ArmorProcessorNode::syncCallback, this, _1, _2));
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Img subscribing...");
                    // 图像的传输方式
                    std::string transport = "raw";
                    // image sub.
                    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, camera_topic,
                        std::bind(&ArmorProcessorNode::imageCallback, this, _1), transport, rmw_qos));
                }
                // 动态调参回调
                callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ArmorProcessorNode::paramsCallback, this, _1));
            }
        }
    }

    ArmorProcessorNode::~ArmorProcessorNode()
    {
    }

    void ArmorProcessorNode::syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, const AutoaimMsg::ConstSharedPtr& target_msg)
    {
        rclcpp::Time last = img_msg->header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        RCLCPP_WARN(this->get_logger(), "Delay:%.2fms", (now.nanoseconds() - last.nanoseconds()) / 1e6);

        cv::Mat src = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        if (!processTargetMsg(*target_msg, &src))
            RCLCPP_ERROR(this->get_logger(), "Armor processes error synchronously...");
        
        if (debug_param_.show_img)
        {
            cv::namedWindow("pred", cv::WINDOW_AUTOSIZE);
            cv::imshow("pred", src);
            cv::waitKey(1);
        }
        return;
    }

    /**
     * @brief 目标回调函数
     * 
     * @param target_info 装甲板信息
     */
    void ArmorProcessorNode::targetMsgCallback(const AutoaimMsg& target_info)
    {
        if (!processTargetMsg(target_info))
            RCLCPP_WARN(this->get_logger(), "Armor processes error...");            
        return;
    }

    bool ArmorProcessorNode::processTargetMsg(const AutoaimMsg& target_info, cv::Mat* src)
    {
        double sleep_time = 0.0;
        AutoaimMsg target = std::move(target_info);
        target.timestamp = target.header.stamp.nanosec;
        Eigen::Vector2d angle = {0.0, 0.0};
        std::unique_ptr<Eigen::Vector3d> aiming_point_world;
        Eigen::Vector3d aiming_point_cam = {0.0, 0.0, 0.0};
        Eigen::Vector3d tracking_point_cam = {0.0, 0.0, 0.0};
        Eigen::Vector2d tracking_angle = {0.0, 0.0};
        PostProcessInfo post_process_info;
        Eigen::Matrix3d rmat_imu;
        Eigen::Quaterniond quat_imu;

        cv::Mat dst = cv::Mat(image_size_.width, image_size_.height, CV_8UC3);
        if (debug_param_.show_img)
        {
            image_mutex_.lock();
            if(!src_.empty() && flag_)
            {
                src_.copyTo(dst);
                flag_ = false;
            }
            image_mutex_.unlock();
        }

        if (target.spinning_switched)
        {
            processor_->error_cnt_ = 0;
            processor_->is_singer_init_[0] = false;
            processor_->is_singer_init_[1] = false;
            processor_->is_singer_init_[2] = false;
            processor_->is_imm_init_ = false;
            is_aimed_ = false;
            pred_angle_[0][0] = 0.0;
            pred_angle_[0][1] = 0.0;
            pred_angle_[1][0] = 0.0;
            pred_angle_[1][1] = 0.0;
            is_pred_failed_ = false;
            count_ = 0;
            is_pred_ = false;
        }

        if (target.is_target_lost)
        {
            processor_->error_cnt_ = 0;
            processor_->is_singer_init_[0] = false;
            processor_->is_singer_init_[1] = false;
            processor_->is_singer_init_[2] = false;
            processor_->is_imm_init_ = false;
            is_aimed_ = false;
            pred_angle_[0][0] = 0.0;
            pred_angle_[0][1] = 0.0;
            pred_angle_[1][0] = 0.0;
            pred_angle_[1][1] = 0.0;
            is_pred_failed_ = false;
            count_ = 0;
            is_pred_ = false;

            // is_aimed_[0] = false;
            // is_aimed_[1] = false;
            // tracking_point_cam = {target_info.aiming_point_cam.x, target_info.aiming_point_cam.y, target_info.aiming_point_cam.z};
            // angle = processor_->coordsolver_.getAngle(tracking_point_cam, rmat_imu);
        }
        else
        {
            if(!debug_param_.using_imu)
            {
                rmat_imu = Eigen::Matrix3d::Identity();
            }
            else
            {
                quat_imu = std::move(Eigen::Quaterniond{target.quat_imu.w, target.quat_imu.x, target.quat_imu.y, target.quat_imu.z});
                rmat_imu = quat_imu.toRotationMatrix();
            }
            
            param_mutex_.lock();
            if (target_info.mode == SENTRY_NORMAL)
            {
                RCLCPP_INFO(this->get_logger(), "Sentry mode...");
                if(processor_->autoShootingLogic(target, post_process_info))
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Auto shooting...");
                    aiming_point_world = std::make_unique<Eigen::Vector3d>(post_process_info.pred_3d_pos);
                    aiming_point_cam = processor_->coordsolver_.worldToCam(*aiming_point_world, rmat_imu);
                }
            }
            else
            {
                // RCLCPP_WARN(this->get_logger(), "Predict...");
                if(debug_param_.show_img && !dst.empty())
                {
                    aiming_point_world = std::move(processor_->predictor(dst, target, sleep_time));
                }
                else
                {
                    aiming_point_world = std::move(processor_->predictor(target, sleep_time));
                }
                aiming_point_cam = processor_->coordsolver_.worldToCam(*aiming_point_world, rmat_imu);
            }
            angle = processor_->coordsolver_.getAngle(aiming_point_cam, rmat_imu);
            tracking_point_cam = {target_info.aiming_point_cam.x, target_info.aiming_point_cam.y, target_info.aiming_point_cam.z};
            tracking_angle = processor_->coordsolver_.getAngle(tracking_point_cam, rmat_imu);
            
            // is_aimed_[0] = abs(tracking_angle[0] < 0.05 && abs(angle[0]) < 20.0) ? true : false;
            // is_aimed_[1] = abs(tracking_angle[1] < 0.05 && abs(angle[1]) < 20.0) ? true : false;
            // angle[0] = (abs(angle[0]) < 0.25) ? tracking_angle[0] : angle[0];
            // angle[1] = (abs(angle[1]) < 0.25) ? tracking_angle[1] : angle[1];
            // if (is_aimed_)
            // {
            //     if (abs(tracking_angle[0]) < 0.5 && abs(tracking_angle[1]) < 0.5)
            //     {
            //         angle = tracking_angle;
            //     }
            // }
            if (abs(tracking_angle[0]) < 0.25 && abs(tracking_angle[1]) < 0.25)
            {
                is_pred_ = true;
                is_aimed_ = true;
            }
            
            if (abs(angle[0]) > 45.0 || abs(angle[1]) > 45.0)
            {
                is_pred_ = false;
                is_aimed_ = false;
            } 

            // else if (abs(angle[0]) < 0.50 && abs(angle[1] < 0.50))
            // {
            //     is_aimed_ = false;
            // }
            // if (is_aimed_)
            // {
            //     if (abs(angle[0]) < 0.50 && abs(angle[1]) < 0.50)
            //     {
            //         angle = tracking_angle;
            //     }
            // }

            // std::cout << "tracking_point_cam: x:" << tracking_point_cam[0] << " y:" << tracking_point_cam[1] << " z:" << tracking_point_cam[2] << std::endl;
            // std::cout << "predict_cam: x:" << aiming_point_cam[0] << " y:" << aiming_point_cam[1] << " z:" << aiming_point_cam[2] << std::endl;
            // cout << "angle:" << angle[0] << " " << angle[1] << " tracking_angle:" << tracking_angle[0] << " " << tracking_angle[1] << endl; 
        }
        param_mutex_.unlock();

        // angle[0] = (is_aimed_[0]) ? angle[0] : tracking_angle[0];
        // angle[1] = (is_aimed_[1]) ? angle[1] : tracking_angle[1];
        if (!is_aimed_)
        {
            angle = tracking_angle;
            is_pred_ = false;
        }
        else
        {
            pred_angle_[0][0] = pred_angle_[0][1];
            pred_angle_[1][0] = pred_angle_[1][1];
            pred_angle_[0][1] = angle[0];
            pred_angle_[1][1] = angle[1];

            // if (pred_angle_[0][1] / pred_angle_[0][0] < 0)
            // {
            //     count_++;
            //     is_pred_ = true;
            // }

            // if (count_ > 2)
            // {
            //     if (abs(angle[0]) < 1.0 || abs(angle[1]) < 1.0)
            //     {
            //         angle = tracking_angle;
            //         is_pred_ = false;
            //     }
            //     else
            //     {
            //         count_ = 0;
                    is_pred_ = true;
            //     }
            // }
            // if (is_pred_failed_ && count_ > 2)
            // {
            //     if ((abs(tracking_angle[0]) > 0.25 || abs(tracking_angle[1]) > 0.25))
            //     {
            //         angle = tracking_angle;
            //         is_pred_failed_ = true;
            //         is_pred_ = false;
            //         if (abs(angle[0]) > 2.0 || abs(angle[1]) > 2.0)
            //         {
            //             count_ = 0;
            //             is_pred_failed_ = false;
            //             is_pred_ = true;
            //         }
            //     }
            //     // count_++;
            // }
            // else
            // {
            //     is_pred_ = true;
            //     is_pred_failed_ = false;
            //     // count_ = 0;
            //     if (pred_angle_[0][1] / pred_angle_[0][0] < 0)
            //     {
            //         count_++;
            //         angle = tracking_angle;
            //         is_pred_failed_ = true;
            //         is_pred_ = false;
            //         // is_aimed_ = false;
            //     }
            // }
            // if (pred_angle_[1][1] / pred_angle_[1][0] < 0)
            // {
            //     angle[1] = tracking_angle[1];
            // }
        }
        
        // Gimbal info pub.
        GimbalMsg gimbal_info;
        gimbal_info.header.frame_id = "barrel_link";
        gimbal_info.header.stamp = target_info.header.stamp;
        gimbal_info.pitch = abs(angle[1]) >= 45.0 ? 0.0 : angle[1];
        gimbal_info.yaw = abs(angle[0]) >= 45.0 ? 0.0 : angle[0];
        gimbal_info.pred_point_cam.x = aiming_point_cam[0];
        gimbal_info.pred_point_cam.y = aiming_point_cam[1];
        gimbal_info.pred_point_cam.z = aiming_point_cam[2];
        gimbal_info.meas_point_cam.x = tracking_point_cam[0];
        gimbal_info.meas_point_cam.y = tracking_point_cam[1];
        gimbal_info.meas_point_cam.z = tracking_point_cam[2];
        gimbal_info.distance = aiming_point_cam.norm();
        gimbal_info.is_target = target_info.mode == SENTRY_NORMAL ? post_process_info.find_target : !target_info.is_target_lost;
        gimbal_info.is_switched = target_info.target_switched;
        gimbal_info.is_spinning = target_info.is_spinning;
        gimbal_info.is_spinning_switched = target_info.spinning_switched;
        gimbal_info.is_prediction = is_pred_; 
        gimbal_info_pub_->publish(std::move(gimbal_info));

        if (this->debug_)
        {
            // RCLCPP_INFO(this->get_logger(), "Tracking msgs pub!!!");
            GimbalMsg tracking_info;
            tracking_info.header.frame_id = "barrel_link1";
            tracking_info.header.stamp = target_info.header.stamp;
            tracking_info.pitch = abs(tracking_angle[1]) >= 45.0 ? 0.0 : tracking_angle[1];
            tracking_info.yaw = abs(tracking_angle[0]) >= 45.0 ? 0.0 : tracking_angle[0];
            tracking_info.meas_point_cam.x = tracking_point_cam[0];
            tracking_info.meas_point_cam.y = tracking_point_cam[1];
            tracking_info.meas_point_cam.z = tracking_point_cam[2];
            tracking_info.pred_point_cam.x = aiming_point_cam[0];
            tracking_info.pred_point_cam.y = aiming_point_cam[1];
            tracking_info.pred_point_cam.z = aiming_point_cam[2];
            tracking_info.distance = tracking_point_cam.norm();
            tracking_info.is_target = target_info.mode == SENTRY_NORMAL ? post_process_info.find_target : !target_info.is_target_lost;
            tracking_info.is_switched = target_info.target_switched;
            tracking_info.is_spinning = target_info.is_spinning;
            tracking_info.is_spinning_switched = target_info.spinning_switched;
            tracking_info.is_prediction = is_pred_;
            tracking_info_pub_->publish(std::move(tracking_info));
            if (!target.is_target_lost)
            {
                AutoaimMsg predict_info;
                predict_info.header.frame_id = "camera_link";

                predict_info.header.stamp = target_info.header.stamp;
                // predict_info.header.stamp.nanosec += sleep_time;
                predict_info.aiming_point_world.x = (*aiming_point_world)[0];
                predict_info.aiming_point_world.y = (*aiming_point_world)[1];
                predict_info.aiming_point_world.z = (*aiming_point_world)[2];
                predict_info.aiming_point_cam.x = aiming_point_cam[0];
                predict_info.aiming_point_cam.y = aiming_point_cam[1];
                predict_info.aiming_point_cam.z = aiming_point_cam[2];
                predict_info.period = target_info.period;
                predict_info_pub_->publish(std::move(predict_info));
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 20, "target info: %lf %lf %lf", (*aiming_point_world)[0], (*aiming_point_world)[1], (*aiming_point_world)[2]);
            }
        }
        // cout << "is_pred:" << is_pred_ << endl;

        if (debug_param_.show_img && !dst.empty()) 
        {
            if (!target.is_target_lost)
            {
                if (this->debug_param_.show_predict)
                {
                    // Draw target 2d rectangle.
                    for(int i = 0; i < 4; i++)
                        cv::line(dst, cv::Point2f(target_info.point2d[i % 4].x, target_info.point2d[i % 4].y),
                            cv::Point2f(target_info.point2d[(i + 1) % 4].x, target_info.point2d[(i + 1) % 4].y), {255, 0, 125}, 2);
                    cv::Point2f point_2d = processor_->coordsolver_.reproject(aiming_point_cam);
                    cv::Point2f armor_center = processor_->coordsolver_.reproject(tracking_point_cam);
                    cv::circle(dst, point_2d, 8, {255, 255, 0}, -1);
                    cv::line(dst, cv::Point2f(point_2d.x - 30, point_2d.y), cv::Point2f(point_2d.x + 30, point_2d.y), {0, 0, 255}, 1);
                    cv::line(dst, cv::Point2f(point_2d.x, point_2d.y - 35), cv::Point2f(point_2d.x, point_2d.y + 35), {0, 0, 255}, 1);
                    cv::line(dst, cv::Point2f(armor_center.x - 30, armor_center.y), cv::Point2f(armor_center.x + 30, armor_center.y), {0, 0, 255}, 1);
                    cv::line(dst, cv::Point2f(armor_center.x, armor_center.y - 35), cv::Point2f(armor_center.x, armor_center.y + 35), {0, 0, 255}, 1);
                    cv::line(dst, cv::Point2f(point_2d.x, point_2d.y), cv::Point2f(armor_center.x, armor_center.y), {255, 0, 125}, 1);
                }
            }
            if (debug_param_.show_aim_cross)
            {
                line(dst, cv::Point2f(dst.size().width / 2, 0), cv::Point2f(dst.size().width / 2, dst.size().height), {0, 255, 0}, 1);
                line(dst, cv::Point2f(0, dst.size().height / 2), cv::Point2f(dst.size().width, dst.size().height / 2), {0, 255, 0}, 1);
            }

            char ch[40];
            char ch1[40];
            sprintf(ch, "Track:pitch:%.2f yaw:%.2f", tracking_angle[1], tracking_angle[0]);
            sprintf(ch1, "Pred:pitch:%.2f yaw:%.2f", angle[1], angle[0]);
            std::string angle_str = ch;
            std::string angle_str1 = ch1;
            putText(dst, angle_str, {dst.size().width / 2 + 5, 30}, cv::FONT_HERSHEY_TRIPLEX, 1, {0, 255, 255});
            putText(dst, angle_str1, {dst.size().width / 2 + 5, 65}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            // putText(dst, (is_aimed_[0] ? "pitchState:Predicting" : "pitchState:Tracking"), {5, 80}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            // putText(dst, (is_aimed_[1] ? "yawState:Predicting" : "yawState:Tracking"), {5, 130}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            putText(dst, (is_pred_ ? "State:Predicting" : "State:Tracking"), {5, 80}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            
            cv::namedWindow("pred", cv::WINDOW_AUTOSIZE);
            cv::imshow("pred", dst);
            cv::waitKey(1);
        }
        return true;
    }

    /**
     * @brief 图像回调函数
     * 
     * @param img_info 图像数据信息
     */
    void ArmorProcessorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_msg)
    {
        if (!img_msg)
            return;
        // rclcpp::Time last = img_msg->header.stamp;
        // rclcpp::Time now = this->get_clock()->now();
        // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Delay:%.2fms", (now.nanoseconds() - last.nanoseconds()) / 1e6);
        image_mutex_.lock();
        src_ = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        flag_ = true;
        image_mutex_.unlock();
    }

    /**
     * @brief 初始化processor类
     * 
     * @return std::unique_ptr<Processor> 
     */
    std::unique_ptr<Processor> ArmorProcessorNode::initArmorProcessor()
    {
        // Declare prediction params.
        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<int>("max_time_delta", 1000);
        this->declare_parameter<int>("max_cost", 509);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("min_fitting_lens", 10);
        this->declare_parameter<int>("shoot_delay", 100);
        this->declare_parameter<int>("window_size", 3);
        this->declare_parameter<double>("yaw_angle_offset", 0.0);
        this->declare_parameter<double>("pitch_angle_offset", 0.0);

        // Declare debug params.
        this->declare_parameter("show_img", false);
        this->declare_parameter("using_imu", false);
        this->declare_parameter("draw_predict", false);
        this->declare_parameter("show_predict", true);
        this->declare_parameter("print_delay", false);
        this->declare_parameter("x_axis_filter", true);
        this->declare_parameter("y_axis_filter", false);
        this->declare_parameter("z_axis_filter", false);
        this->declare_parameter("disable_filter", false);
        this->declare_parameter("disable_fitting", true);
        this->declare_parameter("show_transformed_info", false);
        this->declare_parameter("show_aim_cross", true);

        // Declare path params.
        this->declare_parameter<std::string>("filter_param_path", "/config/filter_param.yaml");
        this->declare_parameter<std::string>("coord_param_path", "/config/camera.yaml");
        this->declare_parameter<std::string>("coord_param_name", "00J90630561");
        
        // Get path param.
        string pkg_share_path = get_package_share_directory("global_user");
        path_param_.coord_name = this->get_parameter("coord_param_name").as_string();
        path_param_.coord_path = pkg_share_path + this->get_parameter("coord_param_path").as_string();
        path_param_.filter_path = pkg_share_path + this->get_parameter("filter_param_path").as_string();

        // Get param from param server.
        bool success = updateParam();
        if(success)
            RCLCPP_INFO(this->get_logger(), "Update param success!");

        vector<double> imm_model_trans_prob_params = {0.6, 0.3, 0.05, 0.05, 0.5, 0.4, 0.05, 0.05, 0.1, 0.1, 0.75, 0.05, 0.1, 0.1, 0.05, 0.75};
        this->declare_parameter("trans_prob_matrix", imm_model_trans_prob_params);
        imm_model_trans_prob_params = this->get_parameter("trans_prob_matrix").as_double_array();

        vector<double> imm_model_prob_params = {0.4, 0.3, 0.15, 0.15};
        this->declare_parameter("model_prob_vector", imm_model_prob_params);
        imm_model_prob_params = this->get_parameter("model_prob_vector").as_double_array();

        vector<double> process_noise_params = {0.4, 0.4, 0.3, 0.3, 0.2, 0.2};
        this->declare_parameter("process_noise", process_noise_params);
        process_noise_params = this->get_parameter("process_noise").as_double_array();

        vector<double> measure_noise_params = {60.0, 60.0, 30.0, 30.0};
        this->declare_parameter("measure_noise", measure_noise_params);
        measure_noise_params = this->get_parameter("measure_noise").as_double_array();

        vector<double> singer_model_params[3] = 
        {
            {0.80, 5.0, 0.10, 0.80, 0.80, 0.20, 1.0, 1.0, 5.0}, 
            {0.80, 5.0, 0.10, 0.80, 0.80, 0.20, 1.0, 1.0, 5.0},
            {0.80, 5.0, 0.10, 0.80, 0.80, 0.20, 1.0, 1.0, 5.0}
        };
        this->declare_parameter("singer_model_x_axis", singer_model_params[0]);
        this->declare_parameter("singer_model_y_axis", singer_model_params[1]);
        this->declare_parameter("singer_model_z_axis", singer_model_params[2]);
        
        singer_model_params[0] = this->get_parameter("singer_model_x_axis").as_double_array();
        singer_model_params[1] = this->get_parameter("singer_model_y_axis").as_double_array();
        singer_model_params[2] = this->get_parameter("singer_model_z_axis").as_double_array();

        // cout << "singer_param:" <<  singer_model_params[0][0] << " " << singer_model_params[0][1] << " " << singer_model_params[0][2] << " " <<  singer_model_params[0][3]
        //     << " " <<  singer_model_params[0][4] << " " <<  singer_model_params[0][5] << " " <<  singer_model_params[0][6] << " " <<  singer_model_params[0][7] << endl;
        predict_param_.filter_model_param.imm_model_trans_prob_params = imm_model_trans_prob_params;
        predict_param_.filter_model_param.imm_model_prob_params = imm_model_prob_params;
        predict_param_.filter_model_param.process_noise_params = process_noise_params;
        predict_param_.filter_model_param.measure_noise_params = measure_noise_params;
        return std::make_unique<Processor>(predict_param_, singer_model_params, path_param_, debug_param_);
    }

    /**
     * @brief 参数回调
     * 
     * @param params 参数服务器变动的参数值
     * @return rcl_interfaces::msg::SetParametersResult 
     */
    rcl_interfaces::msg::SetParametersResult ArmorProcessorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    { 
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        result.successful = updateParam();
        
        param_mutex_.lock();
        processor_->predict_param_ = this->predict_param_;
        processor_->debug_param_ = this->debug_param_;
        param_mutex_.unlock();
        return result;
    }

    bool ArmorProcessorNode::updateAngleOffset()
    {
        if (processor_->is_init_)
        {
            Eigen::Vector2d angle_offset = {0.0, 0.0};
            angle_offset[0] = this->get_parameter("yaw_angle_offset").as_double();
            angle_offset[1] = this->get_parameter("pitch_angle_offset").as_double();
            processor_->coordsolver_.setStaticAngleOffset(angle_offset);
        }
        else
        {
            return false;
        }
        return true;
    }

    /**
     * @brief 动态调参
     * 
     * @return 是否修改参数成功
    */
    bool ArmorProcessorNode::updateParam()
    {   // 动态调参(与rqt_reconfigure一块使用)
        //Prediction param.
        predict_param_.bullet_speed = this->get_parameter("bullet_speed").as_double();
        predict_param_.max_delta_time = this->get_parameter("max_time_delta").as_int();
        predict_param_.max_cost = this->get_parameter("max_cost").as_int();
        predict_param_.max_v = this->get_parameter("max_v").as_int();
        predict_param_.min_fitting_lens = this->get_parameter("min_fitting_lens").as_int();
        predict_param_.shoot_delay = this->get_parameter("shoot_delay").as_int();
        predict_param_.window_size = this->get_parameter("window_size").as_int();
        predict_param_.max_offset_value = this->get_parameter("max_offset_value").as_double();
        predict_param_.reserve_factor = this->get_parameter("reserve_factor").as_double();
        
        //Debug param.
        debug_param_.show_img = this->get_parameter("show_img").as_bool();
        debug_param_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_param_.draw_predict = this->get_parameter("draw_predict").as_bool();
        debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
        debug_param_.x_axis_filter = this->get_parameter("x_axis_filter").as_bool();
        debug_param_.y_axis_filter = this->get_parameter("y_axis_filter").as_bool();
        debug_param_.z_axis_filter = this->get_parameter("z_axis_filter").as_bool();
        debug_param_.print_delay = this->get_parameter("print_delay").as_bool();
        debug_param_.disable_filter = this->get_parameter("disable_filter").as_bool();
        debug_param_.disable_fitting = this->get_parameter("disable_fitting").as_bool();
        debug_param_.show_transformed_info = this->get_parameter("show_transformed_info").as_bool();
        debug_param_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();

        // is_aimed_[0] = false;
        // is_aimed_[1] = false;
        is_aimed_ = false;
        pred_angle_[0][0] = 0.0;
        pred_angle_[0][1] = 0.0;
        pred_angle_[1][0] = 0.0;
        pred_angle_[1][1] = 0.0;
        is_pred_failed_ = false;
        count_ = 0;
        is_pred_ = false;

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