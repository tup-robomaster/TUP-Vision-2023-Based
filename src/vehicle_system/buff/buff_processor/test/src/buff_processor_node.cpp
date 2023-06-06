/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:11:19
 * @LastEditTime: 2023-06-04 00:22:15
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/src/buff_processor_node.cpp
 */
#include "../include/buff_processor_node.hpp"

using namespace std::placeholders;
namespace buff_processor
{
    BuffProcessorNode::BuffProcessorNode(const rclcpp::NodeOptions& options)
    : Node("buff_processor", options)
    {
        RCLCPP_INFO(this->get_logger(), "Buff processor node...");

        try
        {
            buff_processor_ = initBuffProcessor();
        }
        catch(const std::exception& e)
        {
            RCLCPP_FATAL_ONCE(this->get_logger(), "Fatal while initializing buff processor node: %s", e.what());
        }
        
        if(!buff_processor_->is_initialized_)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Init coord params...");
            buff_processor_->coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            buff_processor_->is_initialized_ = true;
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.reliable();
        qos.durability();
        // qos.best_effort();
        // qos.transient_local();
        // qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 3;

        // 发布云台转动信息（pitch、yaw角度）
        gimbal_msg_pub_ = this->create_publisher<GimbalMsg>("/buff_processor/gimbal_msg", qos);

        // 发布预测点信息
        predict_msg_pub_ = this->create_publisher<BuffMsg>("/buff_processor/predict_msg", qos);

        // 订阅待打击目标信息
        buff_msg_sub_ = this->create_subscription<BuffMsg>("/buff_detector/buff_msg", qos,
            std::bind(&BuffProcessorNode::predictorCallback, this, _1));

        bool debug = false;
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug);
        if(debug)
        {
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BuffProcessorNode::paramsCallback, this, _1));
            if(debug_param_.show_img)
            {
                // 图像的传输方式
                std::string transport_type = "raw";
                std::string camera_topic = "/image";
                // image sub.
                img_msg_sub_ = std::make_shared<image_transport::Subscriber>(
                    image_transport::create_subscription(
                        this, 
                        camera_topic, 
                        std::bind(&BuffProcessorNode::imageCallback, this, _1), 
                        transport_type, 
                        rmw_qos
                    )
                );
            }
        }
    }

    BuffProcessorNode::~BuffProcessorNode()
    {}

    void BuffProcessorNode::predictorCallback(const BuffMsg& buff_msg)
    {
        cv::Mat dst;
        BuffInfo predict_info;
        GimbalMsg gimbal_msg;
        bool is_shooting = false;

        if (buff_msg.bullet_speed >= 10.0)
        {   //更新弹速
            double cur_bullet_speed = buff_processor_->coordsolver_.getBulletSpeed();
            if (abs(buff_msg.bullet_speed - cur_bullet_speed) <= 0.5)
            {
                cur_bullet_speed = (buff_msg.bullet_speed + cur_bullet_speed) / 2.0;
            }
            else
            {
                cur_bullet_speed = buff_msg.bullet_speed;
            }
            buff_processor_->coordsolver_.setBulletSpeed(cur_bullet_speed);
        }

        if (buff_msg.shoot_delay >= 50 && buff_msg.shoot_delay <= 300)
        {
            buff_processor_->predictor_param_.shoot_delay = (buff_processor_->predictor_param_.shoot_delay + buff_msg.shoot_delay) / 2.0;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            100, 
            "rec_bullet_speed:%.3f cur_bullet_speed:%.3f cur_shoot_delay:%.3f", 
            buff_msg.bullet_speed, buff_processor_->coordsolver_.getBulletSpeed(), buff_processor_->predictor_param_.shoot_delay
        );

        if (debug_param_.show_img)
        {
            image_mutex_.lock();
            src_.copyTo(dst);
            image_mutex_.unlock();
        }

        bool is_predicted = false;        
        debug_param_.show_img = this->get_parameter("show_img").as_bool();
        if (!buff_msg.is_target_lost)
        {
            if (buff_processor_->predict(buff_msg, predict_info))
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "predict...");
                gimbal_msg.is_target = true;
                gimbal_msg.pitch = predict_info.angle[1];
                gimbal_msg.yaw = predict_info.angle[0];
                gimbal_msg.distance = predict_info.hit_point_cam.norm();
                gimbal_msg.is_switched = predict_info.target_switched;
                gimbal_msg.is_target = true;
                is_predicted = true;
            }
        }
        else
        {
            gimbal_msg.pitch = 0.0;
            gimbal_msg.yaw = 0.0;
            gimbal_msg.is_shooting = false;
            gimbal_msg.is_target = false;
        }
        gimbal_msg.header.frame_id = "barrel_link2";
        gimbal_msg.header.stamp = buff_msg.header.stamp;
        gimbal_msg_pub_->publish(std::move(gimbal_msg));

        if (is_predicted)
        {
            if (debug_param_.show_marker)
            {
                visualization_msgs::msg::MarkerArray marker_array;
                visualization_msgs::msg::Marker marker;
                
                // Set the frame ID and timestamp.
                marker.header.frame_id = "base_link";
                marker.header.stamp = buff_msg.header.stamp;
                marker.header.stamp.nanosec += (500 * 1e6);
                marker.id = 7;

                // Set the namespace and id for this marker.  This serves to create a unique ID
                // Any marker sent with the same namespace and id will overwrite the old one
                marker.ns = "basic_shapes";

                // Set the marker type.  
                // Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                marker.type = shape_;

                // Set the marker action.
                // Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                marker.action = visualization_msgs::msg::Marker::ADD;

                marker.lifetime = rclcpp::Duration::from_nanoseconds((rcl_duration_value_t)1e4);

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                
                marker.pose.position.x = predict_info.hit_point_world(0);
                marker.pose.position.y = predict_info.hit_point_world(1);
                marker.pose.position.z = predict_info.hit_point_world(2);

                marker.pose.orientation.w = q.w();
                marker.pose.orientation.x = q.x();
                marker.pose.orientation.y = q.y();
                marker.pose.orientation.z = q.z();

                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                // Set the color -- be sure to set alpha to something non-zero!
                marker.color.r = 0.5f;
                marker.color.g = 0.5f;
                marker.color.b = 1.0f;
                marker.color.a = 1.0;

                marker_array.markers.emplace_back(marker);                     
                // Publish the marker_array
                marker_array_pub_->publish(marker_array);
            }
        }

        if (debug_param_.show_fitting_curve)
        {
            BuffMsg predict_msg;
            predict_msg.header.frame_id = "camera_link1";
            predict_msg.header.stamp = buff_msg.header.stamp;
            predict_msg.header.stamp.nanosec += (500 * 1e6);

            if (is_predicted)
            {
                // Visual info.
                predict_msg.armor3d_cam = buff_msg.armor3d_cam;
                predict_msg.armor3d_world = buff_msg.armor3d_world;

                predict_msg.predict_point3d_cam.x = predict_info.hit_point_cam[0];
                predict_msg.predict_point3d_cam.y = predict_info.hit_point_cam[1];
                predict_msg.predict_point3d_cam.z = predict_info.hit_point_cam[2];
                
                predict_msg.predict_point3d_world.x = predict_info.hit_point_world[0];
                predict_msg.predict_point3d_world.y = predict_info.hit_point_world[1];
                predict_msg.predict_point3d_world.z = predict_info.hit_point_world[2];

                predict_msg.abs_meas_angle = predict_info.abs_meas_angle;
                predict_msg.abs_fitting_angle = predict_info.abs_fitting_angle;
                predict_msg.abs_pred_angle = predict_info.abs_pred_angle;
                
                last_armor3d_cam(0) = buff_msg.armor3d_cam.x;
                last_armor3d_cam(1) = buff_msg.armor3d_cam.y;
                last_armor3d_cam(2) = buff_msg.armor3d_cam.z;
                last_armor3d_world(0) = buff_msg.armor3d_world.x;
                last_armor3d_world(1) = buff_msg.armor3d_world.y;
                last_armor3d_world(2) = buff_msg.armor3d_world.z;
                last_hit3d_cam = predict_info.hit_point_cam;
                last_hit3d_world = predict_info.hit_point_world;
                last_abs_meas_angle = predict_info.abs_meas_angle;
                last_abs_fitting_angle = predict_info.abs_fitting_angle;
                last_abs_pred_angle = predict_info.abs_pred_angle;
            }
            else
            {
                predict_msg.armor3d_cam.x = last_armor3d_cam(0);
                predict_msg.armor3d_cam.y = last_armor3d_cam(1);
                predict_msg.armor3d_cam.z = last_armor3d_cam(2);

                predict_msg.armor3d_world.x = last_armor3d_world(0);
                predict_msg.armor3d_world.y = last_armor3d_world(1);
                predict_msg.armor3d_world.z = last_armor3d_world(2);

                predict_msg.predict_point3d_cam.x = last_hit3d_cam(0);
                predict_msg.predict_point3d_cam.y = last_hit3d_cam(1);
                predict_msg.predict_point3d_cam.z = last_hit3d_cam(2);
                
                predict_msg.predict_point3d_world.x = last_hit3d_world(0);
                predict_msg.predict_point3d_world.y = last_hit3d_world(1);
                predict_msg.predict_point3d_world.z = last_hit3d_world(2);
                
                predict_msg.abs_meas_angle = last_abs_meas_angle;
                predict_msg.abs_fitting_angle = last_abs_fitting_angle;
                predict_msg.abs_pred_angle = last_abs_pred_angle;
            }
            predict_msg_pub_->publish(std::move(predict_msg));
        }

        if (debug_param_.show_img && !dst.empty())
        {
            cv::Point2f r_center;
            cv::Point2f vertex_sum;
            cv::Point2f armor_center;
            for (int ii = 0; ii < 5; ii++)
            {
                if(ii != 0)
                {
                    vertex_sum.x += buff_msg.points2d[ii].x;
                    vertex_sum.y += buff_msg.points2d[ii].y;
                }
                else
                {
                    r_center.x = buff_msg.points2d[ii].x;
                    r_center.y = buff_msg.points2d[ii].y;
                }

                cv::line(
                    dst, 
                    cv::Point2i(
                        buff_msg.points2d[ii % 5].x, 
                        buff_msg.points2d[ii % 5].y), 
                        cv::Point2i(buff_msg.points2d[(ii + 1) % 5].x, 
                        buff_msg.points2d[(ii + 1) % 5].y
                    ), 
                    {0, 0, 255}, 
                    1
                );
            }
            armor_center = (vertex_sum / 4.0);
            cv::Point2f point_2d = buff_processor_->coordsolver_.reproject(predict_info.hit_point_cam);
            
            cv::line(dst, r_center, armor_center, {125, 0, 125}, 1);
            cv::line(dst, armor_center, point_2d, {125, 125, 0}, 1);
            cv::line(dst, r_center, point_2d, {0, 125, 125}, 1);
            cv::circle(dst, point_2d, 8, {255, 0, 125}, 2);

            cv::namedWindow("pred_img", cv::WINDOW_NORMAL);
            cv::imshow("pred_img", dst);
            cv::waitKey(1);
        }
    }

    void BuffProcessorNode::imageCallback(const ImageMsg::ConstSharedPtr &img_msg)
    {
        if(!img_msg)
            return;

        cv::Mat img;
        img = cv_bridge::toCvShare(std::move(img_msg), "bgr8")->image;

        if (!img.empty())
        {
            image_mutex_.lock();
            img.copyTo(src_);
            image_mutex_.unlock();
        }
    }

    rcl_interfaces::msg::SetParametersResult BuffProcessorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        result.successful = updateParam();
        param_mutex_.lock();
        buff_processor_->predictor_param_ = this->predict_param_;
        buff_processor_->debug_param_ = this->debug_param_;
        param_mutex_.unlock();

        return result;
    }

    bool BuffProcessorNode::updateParam()
    {
        //Prediction param.
        this->get_parameter("bullet_speed", predict_param_.bullet_speed);
        this->get_parameter("shoot_delay", predict_param_.shoot_delay);
        this->get_parameter("delay_coeff", predict_param_.delay_coeff);
        this->get_parameter("delay_big", predict_param_.delay_big);
        this->get_parameter("delay_small", predict_param_.delay_small);

        this->get_parameter("history_deque_len_cos", predict_param_.history_deque_len_cos);
        this->get_parameter("history_deque_len_phase", predict_param_.history_deque_len_phase);
        this->get_parameter("history_deque_len_uniform", predict_param_.history_deque_len_uniform);
        this->get_parameter("max_a", predict_param_.max_a);
        this->get_parameter("max_rmse", predict_param_.max_rmse);
        this->get_parameter("max_timespan", predict_param_.max_timespan);
        this->get_parameter("max_v", predict_param_.max_v);
        this->get_parameter("pf_path", predict_param_.pf_path);
        this->get_parameter("window_size", predict_param_.window_size);
        
        //Debug param.
        this->get_parameter("show_img", this->debug_param_.show_img);
        this->get_parameter("show_marker", this->debug_param_.show_marker);
        this->get_parameter("show_fitting_curve", this->debug_param_.show_fitting_curve);

        return true;
    }

    std::unique_ptr<Processor> BuffProcessorNode::initBuffProcessor()
    {
        //Prediction param.
        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<double>("shoot_delay", 100.0);
        this->declare_parameter<double>("delay_coeff", 1.0);

        this->declare_parameter<double>("delay_big", 200.0);
        this->declare_parameter<double>("delay_small", 150.0);

        this->declare_parameter<int>("history_deque_len_cos", 250);
        this->declare_parameter<int>("history_deque_len_phase", 100);
        this->declare_parameter<int>("history_deque_len_uniform", 100);
        this->declare_parameter<double>("max_a", 8.0);
        this->declare_parameter<double>("max_rmse", 0.5);
        this->declare_parameter<double>("max_timespan", 20000.0);
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<int>("window_size", 2);

        // this->declare_parameter<double>("delay_big", 175.0);
        // this->declare_parameter<double>("delay_small", 100.0);

        //Path param.
        this->declare_parameter<std::string>("camera_name", "KE0200110075");
        this->declare_parameter<std::string>("camera_param_path", "/config/camera.yaml");
        this->declare_parameter<std::string>("pf_path", "/config/filter_param.yaml");
        
        string pkg_share_pth = get_package_share_directory("global_user");
        this->path_param_.camera_name = this->get_parameter("camera_name").as_string();
        this->predict_param_.pf_path = pkg_share_pth + this->get_parameter("pf_path").as_string();
        this->path_param_.camera_param_path = pkg_share_pth + this->get_parameter("camera_param_path").as_string();

        //Debug param.
        this->declare_parameter<bool>("show_img", false);
        this->declare_parameter<bool>("show_marker", false);
        this->declare_parameter<bool>("show_fitting_curve", false);

        auto success = updateParam();
        if(success)
            RCLCPP_INFO(this->get_logger(), "Update param!");

        return std::make_unique<Processor>(predict_param_, path_param_, debug_param_);
    }
} //namespace buff_processor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<buff_processor::BuffProcessorNode>());
    rclcpp::shutdown();
    
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(buff_processor::BuffProcessorNode)