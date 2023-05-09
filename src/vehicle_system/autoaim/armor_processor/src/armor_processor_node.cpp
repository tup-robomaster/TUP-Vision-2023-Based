/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2023-04-27 21:24:23
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
        
        // cout << 3 << endl;
        try
        {
            processor_ = initArmorProcessor();
            if (!processor_->is_param_initialized_)
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Loading param...");
                // processor_->loadParam(path_param_.filter_path);
                processor_->init(path_param_.coord_path, path_param_.coord_name);
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL(this->get_logger(), "Fatal while initializing armor processor: %s", e.what());
        }
        // cout << 4 << endl;

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
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);

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
        double sleep_time = 0.0;
        AutoaimMsg target = std::move(target_info);
        Eigen::Vector2d angle = {0.0, 0.0};
        Eigen::Vector2d tracking_angle = {0.0, 0.0};
        Eigen::Vector3d aiming_point_world = {0.0, 0.0, 0.0};
        Eigen::Vector3d aiming_point_cam = {0.0, 0.0, 0.0};
        Eigen::Vector3d tracking_point_cam = {0.0, 0.0, 0.0};
        Eigen::Matrix3d rmat_imu;
        Eigen::Quaterniond quat_imu;
        bool is_shooting = false;
        vector<Eigen::Vector4d> armor3d_vec;
        Eigen::Vector3d vehicle_center3d_cam = {0.0, 0.0, 0.0};
        Eigen::Vector4d vehicle_center3d_world = {0.0, 0.0, 0.0, 0.0};
        // PostProcessInfo post_process_info;

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

        if (target.is_target_lost && processor_->is_last_exists_)
        {   //目标丢失且上帧存在，预测器进入丢失预测状态
            processor_->armor_predictor_.predictor_state_ = LOSTING;
            is_shooting = false;
        }

        if (target.is_target_lost && processor_->armor_predictor_.predictor_state_ == LOST)
        {   //目标丢失且预测器处于丢失状态则退出预测状态
            is_aimed_ = false;
            is_pred_ = false;
            is_shooting = false;
        }
        else
        {
            if (target_info.bullet_speed > 10.0)
            {   //更新弹速
                processor_->coordsolver_.setBulletSpeed(target_info.bullet_speed);
            }
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
            // cout << 1 << endl;
            if (processor_->predictor(target, aiming_point_world, armor3d_vec, sleep_time))
            {
                // cout << 2 << endl;
                aiming_point_cam = processor_->coordsolver_.worldToCam(aiming_point_world, rmat_imu);
                angle = processor_->coordsolver_.getAngle(aiming_point_cam, rmat_imu);
                tracking_point_cam = {target_info.armors[0].point3d_cam.x, target_info.armors[0].point3d_cam.y, target_info.armors[0].point3d_cam.z};
                tracking_angle = processor_->coordsolver_.getAngle(tracking_point_cam, rmat_imu);

                Eigen::VectorXd state = processor_->armor_predictor_.uniform_ekf_.x();
                vehicle_center3d_world = {state(0), state(1), state(2), 0.0};
                vehicle_center3d_cam = processor_->coordsolver_.worldToCam({vehicle_center3d_world(0), vehicle_center3d_world(1), vehicle_center3d_world(2)}, rmat_imu);
                armor3d_vec.emplace_back(vehicle_center3d_world);
                // cout << "radius:" << state(3) << endl;

                if (abs(tracking_angle[0]) < 3.50 && abs(tracking_angle[1]) < 3.50)
                {
                    is_pred_ = true;
                    is_aimed_ = true;
                    is_shooting = true;
                }
                if (abs(angle[0]) > 45.0 || abs(angle[1]) > 45.0)
                {
                    is_pred_ = false;
                    is_aimed_ = false;
                    is_shooting = false;
                } 
            }
            param_mutex_.unlock();
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
            is_shooting = true;
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
        gimbal_info.is_target = !target_info.is_target_lost;
        gimbal_info.is_switched = target_info.target_switched;
        gimbal_info.is_spinning = target_info.is_spinning;
        gimbal_info.is_spinning_switched = target_info.spinning_switched;
        gimbal_info.is_shooting = is_shooting;
        gimbal_info.is_prediction = is_pred_; 
        gimbal_info_pub_->publish(std::move(gimbal_info));

        // publish gimbal joint states.
        sensor_msgs::msg::JointState gimbal_joint_states;
        gimbal_joint_states.header.frame_id = "base_link";
        gimbal_joint_states.name.emplace_back("base_to_camera_yaw_joint");
        gimbal_joint_states.position.emplace_back(gimbal_info.yaw * CV_PI / 180);
        gimbal_joint_states.name.emplace_back("base_to_camera_pitch_joint");
        gimbal_joint_states.position.emplace_back(gimbal_info.pitch * CV_PI / 180);
        joint_state_pub_->publish(gimbal_joint_states);
        
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
            
            if (!target.is_target_lost)
            {
                if (show_marker_)
                {
                    rclcpp::Time now = this->get_clock()->now();

                    visualization_msgs::msg::MarkerArray marker_array;
                    visualization_msgs::msg::Marker marker;
                    int marker_id = 0;
                    
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

                    marker.lifetime = rclcpp::Duration::from_nanoseconds((rcl_duration_value_t)5e6);
                    
                    for (auto armor3d : armor3d_vec)
                    {
                        marker.id = marker_id;
                        
                        tf2::Quaternion q;
                        q.setRPY(CV_PI / 2, -CV_PI / 2, 0);
                        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                        marker.pose.position.x = armor3d(0);
                        marker.pose.position.y = armor3d(1);
                        marker.pose.position.z = 0;
                        if (marker.id == 4)
                        {
                            marker.type = visualization_msgs::msg::Marker::ARROW;
                            marker.pose.position.z = 0;
                        }
                        else
                        {
                            marker.type = shape_;
                        }
                        marker.pose.orientation.x = q.x();
                        marker.pose.orientation.y = q.y();
                        marker.pose.orientation.z = q.z();
                        marker.pose.orientation.w = q.w();

                        // Set the scale of the marker -- 1x1x1 here means 1m on a side
                        if (marker.id == 4)
                        {
                            marker.scale.x = armor3d(2);
                            marker.scale.y = 0.025;
                            marker.scale.z = 0.025;
                        }
                        else
                        {
                            marker.scale.x = 0.040;
                            marker.scale.y = 0.040;
                            marker.scale.z = 0.040;
                        }

                        // Set the color -- be sure to set alpha to something non-zero!
                        if (marker.id == 4)
                        {
                            marker.color.r = 125.0f;
                            marker.color.g = 255.0f;
                            marker.color.b = 0.0f;
                            marker.color.a = 1.0;
                        }
                        else
                        {
                            marker.color.r = 255.0f;
                            marker.color.g = 0.5f;
                            marker.color.b = 0.5f;
                            marker.color.a = 1.0;
                        }

                        // while ((int)marker_array_pub_->get_subscription_count() < 1)
                        // {
                        //     if (!rclcpp::ok())
                        //     {
                        //         return 0;
                        //     }
                        //     RCLCPP_WARN(this->get_logger(), "Please create a subscriber to the marker");
                        //     sleep(1);
                        // }
                        marker_array.markers.emplace_back(marker);                     
                        ++marker_id;
                    }
                    // Publish the marker_array
                    marker_array_pub_->publish(marker_array);
                }
                // AutoaimMsg predict_info;
                // predict_info.header.frame_id = "camera_link";
                // predict_info.header.stamp = target_info.header.stamp;
                // // predict_info.header.stamp.nanosec += sleep_time;
                // // predict_info.aiming_point_world.x = (aiming_point_world)[0];
                // // predict_info.aiming_point_world.y = (aiming_point_world)[1];
                // // predict_info.aiming_point_world.z = (aiming_point_world)[2];
                // // predict_info.aiming_point_cam.x = aiming_point_cam[0];
                // // predict_info.aiming_point_cam.y = aiming_point_cam[1];
                // // predict_info.aiming_point_cam.z = aiming_point_cam[2];
                // // predict_info.period = target_info.period;
                // predict_info_pub_->publish(std::move(predict_info));
                // RCLCPP_INFO_EXPRESSION(
                //     this->get_logger(), 
                //     debug_param_.show_predict && debug_param_.print_delay, 
                //     "tracking_point_world:[%.3f %.3f %.3f] aiming_point_world:[%.3f %.3f %.3f]",
                //     tracking_point_cam[0], tracking_point_cam[1], tracking_point_cam[2],
                //     aiming_point_cam[0], aiming_point_cam[1], aiming_point_cam[2]
                // );
            }
            
            if (abs(angle[0]) > 45.0 || abs(angle[1]) > 45.0)
            {
                is_pred_ = false;
                is_aimed_ = false;
                is_shooting = false;
            } 

            RCLCPP_INFO_EXPRESSION(this->get_logger(), debug_param_.show_predict && debug_param_.print_delay, "tracking_point_cam:[%.3f %.3f %.3f]", 
                tracking_point_cam[0], tracking_point_cam[1], tracking_point_cam[2]);
            RCLCPP_INFO_EXPRESSION(this->get_logger(), debug_param_.show_predict && debug_param_.print_delay, "predict_point_cam:[%.3f %.3f %.3f]", 
                aiming_point_cam[0], aiming_point_cam[1], aiming_point_cam[2]);
        }

        if (!(target.is_target_lost))
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
                if (this->debug_param_.show_predict)
                {
                    // Draw target 2d rectangle.
                    for (auto armor : target_info.armors)
                    {
                        for(int i = 0; i < 4; i++)
                            cv::line(dst, cv::Point2f(armor.point2d[i % 4].x, armor.point2d[i % 4].y),
                                cv::Point2f(armor.point2d[(i + 1) % 4].x, armor.point2d[(i + 1) % 4].y), {125, 0, 255}, 1);
                    }

                    cv::Point2f point_2d = {0, 0};
                    for (auto armor_point3d_world : armor3d_vec)
                    {
                        Eigen::Vector3d armor_point3d_cam = processor_->coordsolver_.worldToCam({armor_point3d_world(0), armor_point3d_world(1), armor_point3d_world(2)}, rmat_imu);
                        point_2d = processor_->coordsolver_.reproject(armor_point3d_cam);
                        cv::circle(dst, point_2d, 10, {255, 255, 0}, -1);
                    }
                    point_2d = processor_->coordsolver_.reproject(aiming_point_cam);
                    cv::circle(dst, point_2d, 14, {255, 0, 125}, 2);

                    // cv::Point2f vehicle_center2d = processor_->coordsolver_.reproject(vehicle_center3d_cam);
                    // cv::circle(dst, vehicle_center2d, 11, {255, 255, 0}, -1);
                    // cv::Point2f armor_center = processor_->coordsolver_.reproject(tracking_point_cam);
                    // putText(dst, state_map_[(int)(processor_->armor_predictor_.predictor_state_)], point_2d, cv::FONT_HERSHEY_SIMPLEX, 1, {125, 0, 255}, 1);
                    // cv::line(dst, cv::Point2f(point_2d.x - 30, point_2d.y), cv::Point2f(point_2d.x + 30, point_2d.y), {0, 0, 255}, 1);
                    // cv::line(dst, cv::Point2f(point_2d.x, point_2d.y - 35), cv::Point2f(point_2d.x, point_2d.y + 35), {0, 0, 255}, 1);
                    // cv::line(dst, cv::Point2f(armor_center.x - 30, armor_center.y), cv::Point2f(armor_center.x + 30, armor_center.y), {0, 0, 255}, 1);
                    // cv::line(dst, cv::Point2f(armor_center.x, armor_center.y - 35), cv::Point2f(armor_center.x, armor_center.y + 35), {0, 0, 255}, 1);
                    // cv::line(dst, cv::Point2f(point_2d.x, point_2d.y), cv::Point2f(armor_center.x, armor_center.y), {255, 0, 125}, 1);
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
            putText(dst, state_map_[(int)(processor_->armor_predictor_.predictor_state_)], {30, 80}, cv::FONT_HERSHEY_TRIPLEX, 1, {0, 125, 255}, 1);
            
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
        this->declare_parameter<int>("max_time_delta", 1000);
        this->declare_parameter<int>("max_cost", 509);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("min_fitting_lens", 10);
        this->declare_parameter<int>("shoot_delay", 100);
        this->declare_parameter<int>("window_size", 3);
        this->declare_parameter<double>("yaw_angle_offset", 0.0);
        this->declare_parameter<double>("pitch_angle_offset", 0.0);
        this->declare_parameter<double>("max_offset_value", 0.25);
        this->declare_parameter<double>("reserve_factor", 15.0);
        this->declare_parameter<double>("rotation_yaw", 0.0);
        this->declare_parameter<double>("rotation_pitch", 0.0);
        this->declare_parameter<double>("rotation_roll", 0.0);

        // Declare debug params.
        this->declare_parameter("show_img", false);
        this->declare_parameter("using_imu", false);
        this->declare_parameter("show_predict", true);
        this->declare_parameter("print_delay", false);
        this->declare_parameter("x_axis_filter", true);
        this->declare_parameter("y_axis_filter", false);
        this->declare_parameter("z_axis_filter", false);
        this->declare_parameter("disable_filter", false);
        this->declare_parameter("disable_fitting", true);
        this->declare_parameter("show_transformed_info", false);
        this->declare_parameter("show_aim_cross", true);
        this->declare_parameter("show_marker", false);

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

        vector<double> uniform_ekf_params[3] = 
        {
            {1.0, 1.0},
            {1.0, 1.0, 1.0, 1.0},
            {8.00, 10.0, 0.1, 0.8, 0.0030}
        };
        
        this->declare_parameter("uniform_ekf_process_noise_param", uniform_ekf_params[0]);
        this->declare_parameter("uniform_ekf_measure_noise_param", uniform_ekf_params[1]);
        this->declare_parameter("uniform_ekf_singer_param", uniform_ekf_params[2]);
        uniform_ekf_params[0] = this->get_parameter("uniform_ekf_process_noise_param").as_double_array();
        uniform_ekf_params[1] = this->get_parameter("uniform_ekf_measure_noise_param").as_double_array();
        uniform_ekf_params[2] = this->get_parameter("uniform_ekf_singer_param").as_double_array();

        predict_param_.filter_model_param.imm_model_trans_prob_params = imm_model_trans_prob_params;
        predict_param_.filter_model_param.imm_model_prob_params = imm_model_prob_params;
        predict_param_.filter_model_param.process_noise_params = process_noise_params;
        predict_param_.filter_model_param.measure_noise_params = measure_noise_params;
        return std::make_unique<Processor>(predict_param_, uniform_ekf_params, debug_param_);
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
        result.successful = processor_->coordsolver_.setStaticAngleOffset(predict_param_.angle_offset);
        
        param_mutex_.lock();
        // processor_->predict_param_ = this->predict_param_;
        // processor_->debug_param_ = this->debug_param_;
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
        predict_param_.max_delta_time = this->get_parameter("max_time_delta").as_int();
        predict_param_.max_cost = this->get_parameter("max_cost").as_int();
        predict_param_.max_v = this->get_parameter("max_v").as_int();
        predict_param_.min_fitting_lens = this->get_parameter("min_fitting_lens").as_int();
        predict_param_.shoot_delay = this->get_parameter("shoot_delay").as_int();
        predict_param_.window_size = this->get_parameter("window_size").as_int();
        predict_param_.max_offset_value = this->get_parameter("max_offset_value").as_double();
        predict_param_.reserve_factor = this->get_parameter("reserve_factor").as_double();
        predict_param_.rotation_yaw = this->get_parameter("rotation_yaw").as_double();
        predict_param_.rotation_pitch = this->get_parameter("rotation_pitch").as_double();
        predict_param_.rotation_roll = this->get_parameter("rotation_roll").as_double();
        predict_param_.angle_offset[0] = this->get_parameter("yaw_angle_offset").as_double();
        predict_param_.angle_offset[1] = this->get_parameter("pitch_angle_offset").as_double();

        //Debug param.
        debug_param_.show_img = this->get_parameter("show_img").as_bool();
        debug_param_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
        debug_param_.x_axis_filter = this->get_parameter("x_axis_filter").as_bool();
        debug_param_.y_axis_filter = this->get_parameter("y_axis_filter").as_bool();
        debug_param_.z_axis_filter = this->get_parameter("z_axis_filter").as_bool();
        debug_param_.print_delay = this->get_parameter("print_delay").as_bool();
        debug_param_.disable_filter = this->get_parameter("disable_filter").as_bool();
        debug_param_.disable_fitting = this->get_parameter("disable_fitting").as_bool();
        debug_param_.show_transformed_info = this->get_parameter("show_transformed_info").as_bool();
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