/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2023-05-20 03:51:33
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
        try
        {
            processor_ = initArmorProcessor();
            if (!processor_->is_param_initialized_)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Loading param...");
                processor_->init(path_param_.coord_path, path_param_.coord_name);
                processor_->coordsolver_.setBulletSpeed(predict_param_.bullet_speed);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(this->get_logger(), "Fatal while initializing armor processor: %s", e.what());
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.durability();
        qos.reliable();
        // qos.best_effort();
        // qos.transient_local();
        // qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 3;

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
        
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            
            // Prediction info pub.
            // predict_info_pub_ = this->create_publisher<AutoaimMsg>("/armor_processor/predict_msg", qos);
            
            // marker pub
            marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 1);
            shape_ = visualization_msgs::msg::Marker::SPHERE;

            if (debug_param_.show_img)
            {
                std::string camera_topic = "/image";
                std::string transport = "raw";

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
                    RCLCPP_WARN_ONCE(this->get_logger(), "Img subscribing...");
                    // 图像的传输方式
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
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Delay:%.2fms", (now.nanoseconds() - last.nanoseconds()) / 1e6);

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
        // rclcpp::Time sp = target_info.header.stamp;
        // cout << "sp:" << sp.nanoseconds() / 1e9 << endl;

        double sleep_time = 0.0;
        AutoaimMsg target = std::move(target_info);
        Eigen::Vector2d angle = {0.0, 0.0};
        Eigen::Vector2d tracking_angle = {0.0, 0.0};
        Eigen::Vector3d aiming_point_world = {0.0, 0.0, 0.0};
        Eigen::Vector3d aiming_point_cam = {0.0, 0.0, 0.0};
        Eigen::Vector3d tracking_point_cam = {0.0, 0.0, 0.0};
        Eigen::Matrix3d rmat_imu;
        Eigen::Quaterniond quat_imu;
        vector<Eigen::Vector4d> armor3d_vec;
        Eigen::Vector3d vehicle_center3d_cam = {0.0, 0.0, 0.0};
        Eigen::Vector4d vehicle_center3d_world = {0.0, 0.0, 0.0, 0.0};
        // PostProcessInfo post_process_info;
        cv::Point2f point_2d = {0, 0};
        double min_dist = 1e2;
        int idx = 0, flag = -1;
        bool is_shooting = false;

        if (target.bullet_speed >= 10.0)
        {   //更新弹速
            processor_->coordsolver_.setBulletSpeed(target.bullet_speed);
        }

        if (target.shoot_delay >= 50)
        {
            processor_->predict_param_.shoot_delay = (processor_->predict_param_.shoot_delay + target.shoot_delay) / 2.0;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            100, 
            "rec_bullet_speed:%.3f cur_bullet_speed:%.3f cur_shoot_delay:%.3f", 
            target_info.bullet_speed, processor_->coordsolver_.getBulletSpeed(), processor_->predict_param_.shoot_delay
        );
        
        if(debug_param_.use_serial)
        {
            if (debug_param_.use_imu)
            {
                quat_imu = std::move(Eigen::Quaterniond{target.quat_imu.w, target.quat_imu.x, target.quat_imu.y, target.quat_imu.z});
                rmat_imu = quat_imu.toRotationMatrix();
            }
            else
            {
                rmat_imu = Eigen::Matrix3d::Identity();
            }
        }
        else
        {
            rmat_imu = Eigen::Matrix3d::Identity();
        }
                                     
        cv::Mat dst;
        if (debug_param_.show_img)
        {
            image_mutex_.lock();
            if(!src_.empty() && image_flag_)
            {
                src_.copyTo(dst);
                image_flag_ = false;
            }         
            image_mutex_.unlock();
        }

        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "is target lost: %d", (int)target.is_target_lost);
        if (target.is_target_lost && processor_->is_last_exists_)
        {   //目标丢失且上帧存在，预测器进入丢失预测状态
            processor_->armor_predictor_.predictor_state_ = LOSTING;
            is_shooting = false;
            processor_->is_last_exists_ = false;
        }

        if (target.is_target_lost && processor_->armor_predictor_.predictor_state_ == LOST)
        {   //目标丢失且预测器处于丢失状态则退出预测状态
            is_aimed_ = false;
            is_pred_ = false;
            is_shooting = false;
        }
        else
        {
            param_mutex_.lock();
            if (processor_->predictor(target, aiming_point_world, armor3d_vec, sleep_time))
            {
                if (!target.is_target_lost)
                {
                    if (target.is_spinning)
                    {   //小陀螺下自动开火判据
                        for (auto armor_point3d_world : armor3d_vec)
                        {
                            double armor3d_dist = armor_point3d_world.norm();
                            int scale = armor_point3d_world(3) / (2 * CV_PI);
                            double rangle = armor_point3d_world(3) - scale * (2 * CV_PI);
                            if (armor3d_dist < min_dist && rangle >= 1.45 && rangle <= 1.55)
                            {
                                min_dist = armor3d_dist;
                                flag = idx;
                            }
                        
                            Eigen::Vector3d armor_point3d_cam = processor_->coordsolver_.worldToCam({armor_point3d_world(0), armor_point3d_world(1), armor_point3d_world(2)}, rmat_imu);
                            point_2d = processor_->coordsolver_.reproject(armor_point3d_cam);
                            cv::circle(dst, point_2d, 13, {255, 255, 0}, -1);
                            ++idx;
                        }
                        if (flag != -1)
                        {
                            aiming_point_world = {armor3d_vec.at(flag)(0), armor3d_vec.at(flag)(1), armor3d_vec.at(flag)(2)};
                            is_shooting = true;
                        }
                        else
                        {
                            is_shooting = false;
                        }
                    }
                    else
                    {   //机动目标下自动开火判据(TODO)

                    }
                    aiming_point_cam = processor_->coordsolver_.worldToCam(aiming_point_world, rmat_imu);
                    angle = processor_->coordsolver_.getAngle(aiming_point_cam, rmat_imu);
                }

                if (abs(tracking_angle[0]) < 6.50 && abs(tracking_angle[1]) < 6.50)
                {
                    is_pred_ = true;
                    is_aimed_ = true;
                }
                if (abs(angle[0]) > 45.0 || abs(angle[1]) > 45.0)
                {
                    is_pred_ = false;
                    is_aimed_ = false;
                    is_shooting = false;
                } 
            }
            else
            {
                is_pred_ = false;
                is_shooting = false;
            }
            param_mutex_.unlock();
        }

        if (!target_info.is_target_lost)
        {
            tracking_point_cam = {target_info.armors[0].point3d_cam.x, target_info.armors[0].point3d_cam.y, target_info.armors[0].point3d_cam.z};
            tracking_angle = processor_->coordsolver_.getAngle(tracking_point_cam, rmat_imu);
        }

        // 此处为机动目标的自动开火判据
        if (!target.is_spinning && !target.is_target_lost)
        {
            // 将开火范围限制在6m以内
            if (aiming_point_world.norm() <= 6.0)
            {
                if (!iszero(angle(0)) && !iszero(angle(1)))
                {   
                    // 针对纵向机动目标
                    if ((tracking_angle(0) / angle(0) > 0) && abs(tracking_angle(1) - angle(1) <= 1.0))
                    {
                        is_shooting = true;
                    }
                    // 针对横向机动目标
                    if ((tracking_angle(1) / angle(1) > 0) && abs(tracking_angle(0) - angle(0) <= 1.0))
                    {
                        is_shooting = true;
                    }
                }

                // 针对静止目标
                if (abs(tracking_angle(0) - angle(0)) <= 1.0 && abs(tracking_angle(1) - angle(1)) <= 1.0)
                {
                    is_shooting = true;
                }
            }
        }
        
        if (!is_aimed_)
        {
            angle = tracking_angle;
            is_pred_ = false;
            is_shooting = false;
        }
        else
        {
            is_pred_ = true;
        }

        if (processor_->armor_predictor_.predictor_state_ != PREDICTING)
        {
            is_shooting = false;
        }

        // 云台单发限制
        // if (shoot_flag_)
        // {
        //     if (count_ <= 40)
        //     {
        //         is_shooting = false;
        //         count_++;
        //     }
        //     else
        //     {
        //         shoot_flag_ = false;
        //         count_ = 0;
        //     }
        // }
        // if (is_shooting)
        // {
        //     shoot_flag_ = true;
        // }

        RCLCPP_WARN_EXPRESSION(this->get_logger(), is_shooting, "Shooting...");
        
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
        gimbal_info.is_target = !target_info.is_target_lost;
        gimbal_info.is_switched = target_info.target_switched;
        gimbal_info.is_spinning = target_info.is_spinning;
        gimbal_info.is_spinning_switched = target_info.spinning_switched;
        gimbal_info.is_shooting = is_shooting;
        gimbal_info.is_prediction = is_pred_; 
        gimbal_info_pub_->publish(std::move(gimbal_info));

        if (this->debug_)
        {
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
            tracking_info.is_target = !target_info.is_target_lost;
            tracking_info.is_switched = target_info.target_switched;
            tracking_info.is_spinning = target_info.is_spinning;
            tracking_info.is_spinning_switched = target_info.spinning_switched;
            tracking_info.is_shooting = is_shooting;
            tracking_info.is_prediction = is_pred_;
            tracking_info_pub_->publish(std::move(tracking_info));
            
            if (!target.is_target_lost && show_marker_)
            {
                pubMarkerArray(armor3d_vec, target.clockwise, flag);
            }
        }

        if (!target.is_target_lost)
        {
            processor_->is_last_exists_ = true;
        }
        else
        {
            processor_->is_last_exists_ = false;
        }
        
        if (debug_param_.show_img && !dst.empty()) 
        {
            if (!target.is_target_lost)
            {
                // Draw target 2d rectangle.
                for(int i = 0; i < 4; i++)
                    cv::line(dst, cv::Point2f(target_info.armors.front().point2d[i % 4].x, target_info.armors.front().point2d[i % 4].y),
                        cv::Point2f(target_info.armors.front().point2d[(i + 1) % 4].x, target_info.armors.front().point2d[(i + 1) % 4].y), {125, 0, 255}, 1);
                cv::Point2f point_2d = processor_->coordsolver_.reproject(aiming_point_cam);
                cv::Point2f armor_center = processor_->coordsolver_.reproject(tracking_point_cam);
                cv::circle(dst, point_2d, 18, {255, 0, 125}, 3);
                // cv::line(dst, cv::Point2f(point_2d.x - 30, point_2d.y), cv::Point2f(point_2d.x + 30, point_2d.y), {0, 0, 255}, 1);
                // cv::line(dst, cv::Point2f(point_2d.x, point_2d.y - 35), cv::Point2f(point_2d.x, point_2d.y + 35), {0, 0, 255}, 1);
                // cv::line(dst, cv::Point2f(armor_center.x - 30, armor_center.y), cv::Point2f(armor_center.x + 30, armor_center.y), {0, 0, 255}, 1);
                // cv::line(dst, cv::Point2f(armor_center.x, armor_center.y - 35), cv::Point2f(armor_center.x, armor_center.y + 35), {0, 0, 255}, 1);
                // cv::line(dst, cv::Point2f(point_2d.x, point_2d.y), cv::Point2f(armor_center.x, armor_center.y), {255, 0, 125}, 1);
            }
            if (debug_param_.show_aim_cross)
            {
                line(dst, cv::Point2f(dst.size().width / 2, 0), cv::Point2f(dst.size().width / 2, dst.size().height), {0, 255, 0}, 1);
                line(dst, cv::Point2f(0, dst.size().height / 2), cv::Point2f(dst.size().width, dst.size().height / 2), {0, 255, 0}, 1);
            }

            if (debug_param_.draw_predict)
            {
                // Draw vel and acc curve.
                processor_->curveDrawer(0, dst, processor_->armor_predictor_.history_vel_[0], cv::Point2i(260, 120));
                processor_->curveDrawer(1, dst, processor_->armor_predictor_.history_vel_[1], cv::Point2i(260, 200));
                processor_->curveDrawer(2, dst, processor_->armor_predictor_.history_vel_[2], cv::Point2i(260, 280));
            }

            char ch[40];
            char ch1[40];
            sprintf(ch, "Track:pitch:%.2f yaw:%.2f", tracking_angle[1], tracking_angle[0]);
            sprintf(ch1, "Pred:pitch:%.2f yaw:%.2f", angle[1], angle[0]);
            std::string angle_str = ch;
            std::string angle_str1 = ch1;
            putText(dst, angle_str, {dst.size().width / 2 + 5, 30}, cv::FONT_HERSHEY_TRIPLEX, 1, {0, 255, 255});
            putText(dst, angle_str1, {dst.size().width / 2 + 5, 65}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            putText(dst, state_map_[(int)(processor_->armor_predictor_.predictor_state_)], {5, 80}, cv::FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            
            cv::namedWindow("pred", cv::WINDOW_AUTOSIZE);
            cv::imshow("pred", dst);
            cv::waitKey(1);
        }
        return true;
    }

    void ArmorProcessorNode::pubMarkerArray(vector<Eigen::Vector4d> armor3d_vec, bool is_clockwise, int flag)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        int marker_id = 0;
        
        rclcpp::Time now = this->get_clock()->now();

        // Set the frame ID and timestamp.
        marker.header.frame_id = "base_link";
        marker.header.stamp = now;

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";

        // Set the marker type.  
        // Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape_;

        // Set the marker action.
        // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.lifetime = rclcpp::Duration::from_nanoseconds((rcl_duration_value_t)5e3);
        
        int idx = 0;
        for (auto armor3d : armor3d_vec)
        {
            marker.id = marker_id;
            
            tf2::Quaternion q;
            q.setRPY(CV_PI, -CV_PI / 2, 0);
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = armor3d(0);
            marker.pose.position.y = armor3d(1);
            marker.pose.position.z = armor3d(2);

            marker.type = (marker.id == 0 || flag == idx) ? visualization_msgs::msg::Marker::ARROW : shape_;
            if (flag == idx)
            {
                q.setRPY(0, 0, armor3d(3));
            }
            
            marker.pose.orientation.x = q.x();
            marker.pose.orientation.y = q.y();
            marker.pose.orientation.z = q.z();
            marker.pose.orientation.w = q.w();

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = marker.id == 0 ? armor3d(2) : (flag == idx ? (is_clockwise ? -0.25 : 0.25) : 0.060);
            marker.scale.y = marker.id == 0 ? 0.010 : (flag == idx ? 0.040 : 0.060);
            marker.scale.z = marker.scale.y;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = marker.id == 1 ? 1.0f : 0.5f;
            marker.color.g = marker.id == 0 ? 1.0f : (marker.id == 1 ? 0.0f : 0.5f);
            marker.color.b = marker.id == 0 ? 0.0f : (marker.id == 1 ? 0.0f : 1.0f);
            marker.color.a = 1.0;

            marker_array.markers.emplace_back(marker);                     
            ++marker_id;
            ++idx;
        }
        // Publish the marker_array
        marker_array_pub_->publish(marker_array);
    }

    /**
     * @brief 图像回调函数
     * 
     * @param img_info 图像数据信息
     */
    void ArmorProcessorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
    {
        if (!img_msg)
            return;
        rclcpp::Time last = img_msg->header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Delay:%.2fms", (now.nanoseconds() - last.nanoseconds()) / 1e6);
        
        image_mutex_.lock();
        src_ = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        image_flag_ = true;
        image_mutex_.unlock();
    }

    /**
     * @brief 初始化processor类
     * 
     * @return std::unique_ptr<Processor> 
     */
    std::unique_ptr<Processor> ArmorProcessorNode::initArmorProcessor()
    {
        state_map_ =
        {
            {0, "State:Tracking"},
            {1, "State:Predicting"},
            {2, "State:Losting"},
            {3, "State:Lost"},
        };
        // Declare prediction params.
        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<double>("shoot_delay", 100.0);
        this->declare_parameter<double>("delay_coeff", 2.5);
        this->declare_parameter<int>("max_dt", 1000);
        this->declare_parameter<int>("max_cost", 509);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("min_fitting_lens", 10);
        this->declare_parameter<int>("window_size", 3);
        this->declare_parameter<double>("max_offset_value", 0.25);
        this->declare_parameter<double>("reserve_factor", 15.0);
        // this->declare_parameter<double>("rotation_yaw", 0.0);
        // this->declare_parameter<double>("rotation_pitch", 0.0);
        // this->declare_parameter<double>("rotation_roll", 0.0);

        // Declare debug params.
        this->declare_parameter("use_serial", true);
        this->declare_parameter("use_imu", true);
        this->declare_parameter("show_img", false);
        this->declare_parameter("draw_predict", false);
        this->declare_parameter("show_predict", true);
        this->declare_parameter("print_delay", false);
        this->declare_parameter("show_aim_cross", true);
        this->declare_parameter("show_marker", false);

        // Declare path params.
        this->declare_parameter<std::string>("camera_name", "00J90630561");
        this->declare_parameter<std::string>("camera_param_path", "/config/camera.yaml");
        this->declare_parameter<std::string>("filter_param_path", "/config/filter_param.yaml");
        
        // Get path param.
        string pkg_share_path = get_package_share_directory("global_user");
        path_param_.coord_name = this->get_parameter("camera_name").as_string();
        path_param_.coord_path = pkg_share_path + this->get_parameter("camera_param_path").as_string();
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

        vector<double> uniform_ekf_params[2] = 
        {
            {1.0, 1.0, 1.0, 1.0, 1.0, 1.0},
            {1.0, 1.0, 1.0, 1.0},
        };

        vector<double> singer_model_params[2] = 
        {
            {20.0, 8.0, 8.0, 0.0025, 0.0030, 0.0030},
            {1.0, 1.0, 1.0}
        };
        
        this->declare_parameter("uniform_ekf_process_noise_param", uniform_ekf_params[0]);
        this->declare_parameter("uniform_ekf_measure_noise_param", uniform_ekf_params[1]);
        this->declare_parameter("singer_model_process_param", singer_model_params[0]);
        this->declare_parameter("singer_model_measure_param", singer_model_params[1]);
        uniform_ekf_params[0] = this->get_parameter("uniform_ekf_process_noise_param").as_double_array();
        uniform_ekf_params[1] = this->get_parameter("uniform_ekf_measure_noise_param").as_double_array();
        singer_model_params[0] = this->get_parameter("singer_model_process_param").as_double_array();
        singer_model_params[1] = this->get_parameter("singer_model_measure_param").as_double_array();

        predict_param_.filter_model_param.imm_model_trans_prob_params = imm_model_trans_prob_params;
        predict_param_.filter_model_param.imm_model_prob_params = imm_model_prob_params;
        predict_param_.filter_model_param.process_noise_params = process_noise_params;
        predict_param_.filter_model_param.measure_noise_params = measure_noise_params;
        
        return std::make_unique<Processor>(predict_param_, uniform_ekf_params, singer_model_params, debug_param_);
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

    /**
     * @brief 动态调参
     * 
     * @return 是否修改参数成功
    */
    bool ArmorProcessorNode::updateParam()
    {   // 动态调参(与rqt_reconfigure一块使用)
        //Prediction param.
        predict_param_.bullet_speed = this->get_parameter("bullet_speed").as_double();
        predict_param_.shoot_delay = this->get_parameter("shoot_delay").as_double();
        predict_param_.delay_coeff = this->get_parameter("delay_coeff").as_double();
        predict_param_.max_dt = this->get_parameter("max_dt").as_int();
        predict_param_.max_cost = this->get_parameter("max_cost").as_int();
        predict_param_.max_v = this->get_parameter("max_v").as_int();
        predict_param_.min_fitting_lens = this->get_parameter("min_fitting_lens").as_int();
        predict_param_.window_size = this->get_parameter("window_size").as_int();
        predict_param_.max_offset_value = this->get_parameter("max_offset_value").as_double();
        predict_param_.reserve_factor = this->get_parameter("reserve_factor").as_double();
        // predict_param_.rotation_yaw = this->get_parameter("rotation_yaw").as_double();
        // predict_param_.rotation_pitch = this->get_parameter("rotation_pitch").as_double();
        // predict_param_.rotation_roll = this->get_parameter("rotation_roll").as_double();

        //Debug param.
        debug_param_.use_serial = this->get_parameter("use_serial").as_bool();
        debug_param_.use_imu = this->get_parameter("use_imu").as_bool();
        debug_param_.show_img = this->get_parameter("show_img").as_bool();
        debug_param_.draw_predict = this->get_parameter("draw_predict").as_bool();
        debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
        debug_param_.print_delay = this->get_parameter("print_delay").as_bool();
        debug_param_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        show_marker_ = this->get_parameter("show_marker").as_bool();

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