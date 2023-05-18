/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:11:19
 * @LastEditTime: 2023-05-18 19:05:59
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
        rmw_qos.depth = 1;

        // 发布云台转动信息（pitch、yaw角度）
        gimbal_msg_pub_ = this->create_publisher<GimbalMsg>("/buff_processor/gimbal_msg", qos);

        // 发布预测点信息
        predict_msg_pub_ = this->create_publisher<BuffMsg>("/buff_processor/predict_msg", qos);

        // 订阅待打击目标信息
        buff_msg_sub_ = this->create_subscription<BuffMsg>(
            "/buff_detector/buff_msg", 
            qos,
            std::bind(&BuffProcessorNode::buffMsgCallback, this, _1)
        );

        // 图像的传输方式
        std::string transport_type = "raw";
        std::string camera_topic = "/image";
        
        bool debug = false;
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug);
        if(debug)
        {
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BuffProcessorNode::paramsCallback, this, _1));
            if(debug_param_.show_predict)
            {
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

    void BuffProcessorNode::buffMsgCallback(const BuffMsg& buff_msg)
    {
        TargetInfo predict_info;
        cv::Point2f point_2d = cv::Point2f(0, 0);
        Eigen::Quaterniond quat_gimbal;
        geometry_msgs::msg::TransformStamped t = buff_msg.transform_gimbal;

        quat_gimbal.x() = t.transform.rotation.x;
        quat_gimbal.y() = t.transform.rotation.y;
        quat_gimbal.z() = t.transform.rotation.z;
        quat_gimbal.w() = t.transform.rotation.w;
        Eigen::Matrix3d rmat_gimbal = quat_gimbal.toRotationMatrix();
        Eigen::Vector3d translation = {t.transform.translation.x, t.transform.translation.y, t.transform.translation.z};

        if (buff_msg.bullet_speed >= 10.0)
        {   //更新弹速
            buff_processor_->coordsolver_.setBulletSpeed(buff_msg.bullet_speed);
        }

        if (buff_msg.shoot_delay >= 50)
        {   //更新发弹延迟
            buff_processor_->predictor_param_.shoot_delay = (buff_processor_->predictor_param_.shoot_delay + buff_msg.shoot_delay) / 2.0;
        }

        cv::Mat dst;
        if (debug_param_.show_predict)
        {
            image_mutex_.lock();
            src_.copyTo(dst);
            image_mutex_.unlock();
        }

        if (buff_processor_->predict(buff_msg, rmat_gimbal, translation, predict_info))
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "predict...");
            GimbalMsg gimbal_msg;
            gimbal_msg.header.frame_id = "barrel_link2";
            gimbal_msg.header.stamp = buff_msg.header.stamp;
            gimbal_msg.pitch = predict_info.angle[1];
            gimbal_msg.yaw = predict_info.angle[0];
            gimbal_msg.distance = predict_info.hit_point_cam.norm();
            gimbal_msg.is_switched = predict_info.target_switched;
            
            gimbal_msg_pub_->publish(std::move(gimbal_msg));

            debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
            if (debug_param_.show_predict)
            {
                BuffMsg predict_msg;
                predict_msg.header.frame_id = "camera_link";
                predict_msg.header.stamp = buff_msg.header.stamp;
                predict_msg.predict_point.x = predict_info.hit_point_world[0];
                predict_msg.predict_point.y = predict_info.hit_point_world[1];
                predict_msg.predict_point.z = predict_info.hit_point_world[2];
                
                // predict_msg.header.stamp.nanosec += (500 * 1e6);
                // predict_msg.predict_point.x = predict_info.hit_point_cam[0];
                // predict_msg.predict_point.y = predict_info.hit_point_cam[1];
                // predict_msg.predict_point.z = predict_info.hit_point_cam[2];
            
                predict_msg_pub_->publish(std::move(predict_msg));
            }

            if (debug_param_.show_predict && !dst.empty())
            {
                cv::Point2f r_center;
                cv::Point2f vertex_sum;
                cv::Point2f armor_center;
                for (int i = 0; i < 5; i++)
                {
                    // if(i != 0)
                    //     vertex_sum += apex2d[i];
                    // else
                    //     r_center = apex2d[i];
                    cv::line(dst, cv::Point2i(buff_msg.points2d[i % 5].x, buff_msg.points2d[i % 5].y),
                        cv::Point2i(buff_msg.points2d[(i+1)%5].x, buff_msg.points2d[(i+1)%5].y), {125, 0, 255}, 2);
                }
                point_2d = buff_processor_->coordsolver_.reproject(predict_info.hit_point_cam);
            
                // armor_center = (vertex_sum / 4.0);
                // cv::line(dst, r_center, armor_center, {125,125, 0}, 4);
                // cv::line(dst, armor_center, point_2d, {125, 125, 0}, 4);
                // cv::line(dst, r_center, point_2d, {125, 125, 0}, 4);
                cv::circle(dst, point_2d, 6, {0, 0, 255}, 2);
                // for(int i = 0; i < 5; i++)
                //     cv::line(dst, apex2d[i % 5], apex2d[(i + 1) % 5], {0, 255, 255}, 5);
            }
        }

        if (debug_param_.show_predict && !dst.empty())
        {
            cv::namedWindow("pred_img", cv::WINDOW_NORMAL);
            cv::imshow("pred_img", dst);
            cv::waitKey(1);
        }
    }

    void BuffProcessorNode::imageCallback(const ImageMsg::ConstSharedPtr &img_msg)
    {
        if(!img_msg)
            return;
        cv::Mat img = cv_bridge::toCvShare(std::move(img_msg), "bgr8")->image;

        if (!img.empty())
        {
            image_mutex_.lock();
            src_ = cv::Mat(image_size_.width, image_size_.height, CV_8UC3);
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
        this->get_parameter("show_predict", this->debug_param_.show_predict);
        this->get_parameter("use_serial", this->debug_param_.use_serial);

        return true;
    }

    std::unique_ptr<Processor> BuffProcessorNode::initBuffProcessor()
    {
        //Prediction param.
        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<double>("shoot_delay", 100.0);
        this->declare_parameter<double>("delay_big", 175.0);
        this->declare_parameter<double>("delay_small", 100.0);
        this->declare_parameter<int>("history_deque_len_cos", 250);
        this->declare_parameter<int>("history_deque_len_phase", 100);
        this->declare_parameter<int>("history_deque_len_uniform", 100);
        this->declare_parameter<double>("max_a", 8.0);
        this->declare_parameter<double>("max_rmse", 0.5);
        this->declare_parameter<double>("max_timespan", 20000.0);
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<int>("window_size", 2);

        //Path param.
        this->declare_parameter<std::string>("camera_name", "KE0200110075");
        this->declare_parameter<std::string>("camera_param_path", "/config/camera.yaml");
        this->declare_parameter<std::string>("pf_path", "/config/filter_param.yaml");
        
        string pkg_share_pth = get_package_share_directory("global_user");
        this->path_param_.camera_name = this->get_parameter("camera_name").as_string();
        this->predict_param_.pf_path = pkg_share_pth + this->get_parameter("pf_path").as_string();
        this->path_param_.camera_param_path = pkg_share_pth + this->get_parameter("camera_param_path").as_string();

        //Debug param.
        this->declare_parameter<bool>("show_predict", true);
        this->declare_parameter<bool>("use_serial", false);

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