/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:11:19
 * @LastEditTime: 2023-06-06 21:27:01
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

        buff_processor_->is_initialized_ = false;
        if(!buff_processor_->is_initialized_)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Init coord params...");
            buff_processor_->coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            buff_processor_->is_initialized_ = true;
        }

        start_time_ = this->get_clock()->now();

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
        if (debug)
        {
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&BuffProcessorNode::paramsCallback, this, _1));
            if (debug_param_.show_img)
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

        if (debug_param_.show_fitting_curve)
        {
            draw_curve_callback_timer_ = this->create_wall_timer(5ms, std::bind(&BuffProcessorNode::drawCurve, this));
        }        
    }

    BuffProcessorNode::~BuffProcessorNode()
    {}

    void BuffProcessorNode::drawCurve()
    {
        if (buff_processor_->buff_predictor_.is_params_confirmed)
        {
            std::vector<double> plt_time;
            std::vector<double> plt_speed;
            std::vector<double> plt_fitted;
            
            plot_mutex_.lock();
            double params[4] = {
                buff_processor_->buff_predictor_.params[0], 
                buff_processor_->buff_predictor_.params[1], 
                buff_processor_->buff_predictor_.params[2], 
                buff_processor_->buff_predictor_.params[3]
            };
            deque<TargetInfo> his_info(buff_processor_->buff_predictor_.history_info.cbegin(), buff_processor_->buff_predictor_.history_info.cend());
            plot_mutex_.unlock();

            // uint64_t st = start_time_.nanoseconds();
            for (int ii = 0; ii < (int)his_info.size(); ii++)
            {
                auto t = his_info[ii].timestamp / 1e9;
                double pred = params[0] * sin (params[1] * t + params[2]) + params[3];
                double measure = his_info[ii].speed;

                plt_time.push_back(t);
                plt_speed.push_back(measure);
                plt_fitted.push_back(pred);

                // cout << "dt:"<< t << " pre:" << pred << " measure:" << measure << endl;
            }
            plt::clf();
            plt::plot(plt_time, plt_speed, "bx");
            plt::plot(plt_time, plt_fitted, "r-");
            plt::pause(0.001);
        }
    }

    void BuffProcessorNode::predictorCallback(const BuffMsg& buff_msg)
    {
        BuffMsg target_msg = move(buff_msg);
        cv::Mat dst;
        BuffInfo predict_info;
        GimbalMsg gimbal_msg;
        bool is_shooting = false;

        rclcpp::Time stamp = buff_msg.header.stamp;
        rclcpp::Time now = this->get_clock()->now();
        target_msg.timestamp = now.nanoseconds() - start_time_.nanoseconds();
        // target_msg.timestamp = stamp.nanoseconds() - start_time_.nanoseconds();

        if (target_msg.bullet_speed >= 10.0)
        {   //更新弹速
            double cur_bullet_speed = buff_processor_->coordsolver_.getBulletSpeed();
            if (abs(target_msg.bullet_speed - cur_bullet_speed) <= 0.5)
            {
                cur_bullet_speed = (target_msg.bullet_speed + cur_bullet_speed) / 2.0;
            }
            else
            {
                cur_bullet_speed = target_msg.bullet_speed;
            }
            buff_processor_->coordsolver_.setBulletSpeed(cur_bullet_speed);
        }

        if (target_msg.shoot_delay >= 50 && target_msg.shoot_delay <= 300)
        {
            buff_processor_->predictor_param_.shoot_delay = (buff_processor_->predictor_param_.shoot_delay + target_msg.shoot_delay) / 2.0;
        }

        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(), 
            100, 
            "rec_bullet_speed:%.3f cur_bullet_speed:%.3f cur_shoot_delay:%.3f", 
            target_msg.bullet_speed, buff_processor_->coordsolver_.getBulletSpeed(), buff_processor_->predictor_param_.shoot_delay
        );

        if (debug_param_.show_img)
        {
            image_mutex_.lock();
            src_.copyTo(dst);
            image_mutex_.unlock();
        }
        
        debug_param_.show_img = this->get_parameter("show_img").as_bool();
        bool is_predicted = false;
        if (!target_msg.is_target_lost)
        {
            plot_mutex_.lock();
            if (buff_processor_->predictor(target_msg, predict_info))
            {
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(),
                    100, 
                    "is predicting..."
                );

                gimbal_msg.pitch = predict_info.angle[1];
                gimbal_msg.yaw = predict_info.angle[0];
                gimbal_msg.distance = predict_info.hit_point_cam.norm();
                gimbal_msg.is_switched = predict_info.target_switched;
                gimbal_msg.meas_point_cam.x = target_msg.armor3d_cam.x;
                gimbal_msg.meas_point_cam.y = target_msg.armor3d_cam.y;
                gimbal_msg.meas_point_cam.z = target_msg.armor3d_cam.z;
                gimbal_msg.pred_point_cam.x = predict_info.hit_point_cam(0);
                gimbal_msg.pred_point_cam.y = predict_info.hit_point_cam(1);
                gimbal_msg.pred_point_cam.z = predict_info.hit_point_cam(2);
                gimbal_msg.is_target = true;
                is_predicted = true;
            }
            else
            {
                gimbal_msg.pitch = predict_info.angle[1];
                gimbal_msg.yaw = predict_info.angle[0];

                Eigen::Vector3d armor3d_world = {target_msg.armor3d_world.x, target_msg.armor3d_world.y, target_msg.armor3d_world.z};
                gimbal_msg.distance = armor3d_world.norm();
                gimbal_msg.is_switched = target_msg.target_switched;
                gimbal_msg.meas_point_cam.x = target_msg.armor3d_cam.x;
                gimbal_msg.meas_point_cam.y = target_msg.armor3d_cam.y;
                gimbal_msg.meas_point_cam.z = target_msg.armor3d_cam.z;
                gimbal_msg.pred_point_cam.x = target_msg.armor3d_cam.x;
                gimbal_msg.pred_point_cam.y = target_msg.armor3d_cam.y;
                gimbal_msg.pred_point_cam.z = target_msg.armor3d_cam.z;
                gimbal_msg.is_target = true;
                is_predicted = false;

            }
            plot_mutex_.unlock();

            last_gimabl_angle_ = predict_info.angle;
            last_meas_point3d_cam_ = predict_info.armor3d_cam;
            last_meas_point3d_world_ = predict_info.armor3d_world;
            last_pred_point3d_cam_ = predict_info.hit_point_cam;
            last_pred_point3d_world_ = predict_info.hit_point_world;
        }
        else
        {
            gimbal_msg.pitch = 0.0;
            gimbal_msg.yaw = 0.0;
            gimbal_msg.meas_point_cam.x = 0.0;
            gimbal_msg.meas_point_cam.y = 0.0;
            gimbal_msg.meas_point_cam.z = 0.0;
            gimbal_msg.is_shooting = false;
            gimbal_msg.is_target = false;
            gimbal_msg.is_switched = buff_msg.target_switched;
            gimbal_msg.meas_point_cam.x = last_meas_point3d_cam_(0);
            gimbal_msg.meas_point_cam.y = last_meas_point3d_cam_(1);
            gimbal_msg.meas_point_cam.z = last_meas_point3d_cam_(2);
            gimbal_msg.pred_point_cam.x = last_pred_point3d_cam_(0);
            gimbal_msg.pred_point_cam.y = last_pred_point3d_cam_(1);
            gimbal_msg.pred_point_cam.z = last_pred_point3d_cam_(2);

            is_predicted = false;
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            100, 
            "Yaw: %.3f Pitch: %.3f", 
            gimbal_msg.yaw, gimbal_msg.pitch
        );

        gimbal_msg.header.frame_id = "barrel_link";
        gimbal_msg.header.stamp = target_msg.header.stamp;
        gimbal_msg_pub_->publish(std::move(gimbal_msg));
        
        if (debug_param_.show_fitting_curve)
        {
            BuffMsg predict_msg;
            predict_msg.header.frame_id = "camera_link1";
            predict_msg.header.stamp = target_msg.header.stamp;
            // predict_msg.header.stamp.nanosec += (500 * 1e6);

            if (is_predicted)
            {
                predict_msg.armor3d_cam = target_msg.armor3d_cam;
                predict_msg.armor3d_world = target_msg.armor3d_world;

                predict_msg.predict_point3d_cam.x = predict_info.hit_point_cam(0);
                predict_msg.predict_point3d_cam.y = predict_info.hit_point_cam(1);
                predict_msg.predict_point3d_cam.z = predict_info.hit_point_cam(2);
                
                predict_msg.predict_point3d_world.x = predict_info.hit_point_world(0);
                predict_msg.predict_point3d_world.y = predict_info.hit_point_world(1);
                predict_msg.predict_point3d_world.z = predict_info.hit_point_world(2);

                predict_msg.abs_pred_angle = predict_info.abs_pred_angle;
                predict_msg.abs_meas_angle = predict_info.abs_meas_angle;
                last_pred_angle_ = predict_msg.abs_pred_angle;
                last_meas_angle_ = predict_msg.abs_meas_angle;

                last_pred_point3d_cam_(0) = predict_msg.predict_point3d_cam.x;
                last_pred_point3d_cam_(1) = predict_msg.predict_point3d_cam.y;
                last_pred_point3d_cam_(2) = predict_msg.predict_point3d_cam.z;

                last_pred_point3d_world_(0) = predict_msg.predict_point3d_world.x;
                last_pred_point3d_world_(1) = predict_msg.predict_point3d_world.y;
                last_pred_point3d_world_(2) = predict_msg.predict_point3d_world.z;

                last_meas_point3d_cam_(0) = target_msg.armor3d_cam.x;
                last_meas_point3d_cam_(1) = target_msg.armor3d_cam.y;
                last_meas_point3d_cam_(2) = target_msg.armor3d_cam.z;

                last_meas_point3d_world_(0) = target_msg.armor3d_world.x;
                last_meas_point3d_world_(1) = target_msg.armor3d_world.y;
                last_meas_point3d_world_(2) = target_msg.armor3d_world.z;
            }
            else
            {
                predict_msg.predict_point3d_cam.x = last_pred_point3d_cam_(0);
                predict_msg.predict_point3d_cam.y = last_pred_point3d_cam_(1);
                predict_msg.predict_point3d_cam.z = last_pred_point3d_cam_(2);

                predict_msg.predict_point3d_world.x = last_pred_point3d_world_(0);
                predict_msg.predict_point3d_world.y = last_pred_point3d_world_(1);
                predict_msg.predict_point3d_world.z = last_pred_point3d_world_(2);

                predict_msg.armor3d_cam.x = last_meas_point3d_cam_(0);
                predict_msg.armor3d_cam.y = last_meas_point3d_cam_(1);
                predict_msg.armor3d_cam.z = last_meas_point3d_cam_(2);

                predict_msg.armor3d_world.x = last_meas_point3d_world_(0);
                predict_msg.armor3d_world.y = last_meas_point3d_world_(1);
                predict_msg.armor3d_world.z = last_meas_point3d_world_(2);

                predict_msg.abs_pred_angle = last_pred_angle_;
                predict_msg.abs_meas_angle = last_meas_angle_;
            }
            predict_msg_pub_->publish(std::move(predict_msg));
        }

        if (debug_param_.show_img && !dst.empty())
        {
            if (!target_msg.is_target_lost)
            {
                cv::Point2f r_center;
                cv::Point2f vertex_sum;
                cv::Point2f armor_center;
                for (int ii = 0; ii < 5; ii++)
                {
                    if(ii != 0)
                    {
                        vertex_sum.x += target_msg.points2d[ii].x;
                        vertex_sum.y += target_msg.points2d[ii].y;
                    }
                    else
                    {
                        r_center.x = target_msg.points2d[ii].x;
                        r_center.y = target_msg.points2d[ii].y;
                    }
                    cv::line(
                        dst, 
                        cv::Point2i(
                            target_msg.points2d[ii % 5].x, 
                            target_msg.points2d[ii % 5].y), 
                            cv::Point2i(target_msg.points2d[(ii + 1) % 5].x, 
                            target_msg.points2d[(ii + 1) % 5].y
                        ), 
                        {0, 0, 255}, 
                        2
                    );
                }

                armor_center = (vertex_sum / 4.0);
                cv::line(dst, r_center, armor_center, {125, 0, 125}, 2);

                if (is_predicted)
                {
                    cv::Point2f point_2d = buff_processor_->coordsolver_.reproject(predict_info.hit_point_cam);
                    cv::line(dst, armor_center, point_2d, {125, 125, 0}, 2);
                    cv::line(dst, r_center, point_2d, {0, 125, 125}, 2);
                    cv::circle(dst, point_2d, 8, {255, 0, 125}, 2);
                }                
            }

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
        this->get_parameter("history_deque_len_cos", predict_param_.history_deque_len_cos);
        this->get_parameter("history_deque_len_phase", predict_param_.history_deque_len_phase);
        this->get_parameter("history_deque_len_uniform", predict_param_.history_deque_len_uniform);
        this->get_parameter("max_a", predict_param_.max_a);
        this->get_parameter("max_rmse", predict_param_.max_rmse);
        this->get_parameter("max_timespan", predict_param_.max_timespan);
        this->get_parameter("max_v", predict_param_.max_v);
        this->get_parameter("pf_path", predict_param_.pf_path);
        this->get_parameter("window_size", predict_param_.window_size);
        
        this->get_parameter("delay_big", predict_param_.delay_big);
        this->get_parameter("delay_small", predict_param_.delay_small);

        cout << "delay_small:" << predict_param_.delay_small << " delay_big:" << predict_param_.delay_big << endl;

        //Debug param.
        this->get_parameter("show_img", this->debug_param_.show_img);
        this->get_parameter("show_fitting_curve", this->debug_param_.show_fitting_curve);
        this->get_parameter("show_marker", this->debug_param_.show_marker);

        return true;
    }

    std::unique_ptr<Processor> BuffProcessorNode::initBuffProcessor()
    {
        //Prediction param.
        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<double>("shoot_delay", 100.0);
        this->declare_parameter<int>("history_deque_len_cos", 250);
        this->declare_parameter<int>("history_deque_len_phase", 100);
        this->declare_parameter<int>("history_deque_len_uniform", 100);
        this->declare_parameter<double>("max_a", 8.0);
        this->declare_parameter<double>("max_rmse", 0.5);
        this->declare_parameter<double>("max_timespan", 20000.0);
        this->declare_parameter<double>("max_v", 3.0);
        this->declare_parameter<int>("window_size", 2);

        this->declare_parameter<double>("delay_big", 175.0);
        this->declare_parameter<double>("delay_small", 100.0);

        vector<double> params_bound = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        this->declare_parameter("params_bound", params_bound);
        params_bound = this->get_parameter("params_bound").as_double_array();
        predict_param_.params_bound = params_bound;
        
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
        this->declare_parameter<bool>("show_fitting_curve", false);
        this->declare_parameter<bool>("show_marker", false);

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