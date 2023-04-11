/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2023-04-10 12:44:51
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
        
        flag_ = false;
        processor_ = initArmorProcessor();
        if(!processor_->is_init)
        {
            RCLCPP_INFO(this->get_logger(), "Loading param...");
            processor_->loadParam(path_param_.filter_path);
            processor_->init(path_param_.coord_path, path_param_.coord_name);
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 1;

        // 发布云台转动信息（pitch、yaw角度）
        gimbal_info_pub_ = this->create_publisher<GimbalMsg>("/armor_processor/gimbal_msg", qos);
        tracking_info_pub_ = this->create_publisher<GimbalMsg>("/armor_processor/tracking_msg", qos);

        // 订阅目标装甲板信息
        target_info_sub_ = this->create_subscription<AutoaimMsg>(
            "/armor_detector/armor_msg",
            qos,
            std::bind(&ArmorProcessorNode::targetMsgCallback, this, _1)
        );   

        // 是否使用共享内存
        this->declare_parameter<bool>("using_shared_memory", false);
        using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();
        
        // 相机类型
        this->declare_parameter<int>("camera_type", global_user::DaHeng);
        int camera_type = this->get_parameter("camera_type").as_int();
        
        // 图像的传输方式
        std::string transport = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
        
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            RCLCPP_INFO(this->get_logger(), "debug...");
            
            // Prediction info pub.
            predict_info_pub_ = this->create_publisher<AutoaimMsg>("/armor_processor/predict_msg", qos);

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
                this->read_memory_thread_ = std::thread(&ArmorProcessorNode::imgCallbackThread, this);
            }
            else
            {
                image_size_ = image_info_.image_size_map[camera_type];
                // image sub.
                std::string camera_topic = image_info_.camera_topic_map[camera_type];
                img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, camera_topic,
                    std::bind(&ArmorProcessorNode::imageCallback, this, _1), transport, rmw_qos));
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
        if(this->read_memory_thread_.joinable())
            this->read_memory_thread_.join();
    }

    /**
     * @brief 目标回调函数
     * 
     * @param target_info 装甲板信息
     */
    void ArmorProcessorNode::targetMsgCallback(const AutoaimMsg& target_info)
    {
        double sleep_time = 0.0;
        AutoaimMsg target = target_info;
        Eigen::Vector2d angle = {0.0, 0.0};
        Eigen::Vector3d aiming_point_cam = {0.0, 0.0, 0.0};
        Eigen::Vector3d pred_point_world = {0.0, 0.0, 0.0};
        Eigen::Vector3d tracking_point_cam = {0.0, 0.0, 0.0};
        Eigen::Vector2d tracking_angle = {0.0, 0.0};
        // cout << "target_info.period:" << target_info.period << endl;

        cv::Mat dst;
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

        if(target.is_target_lost)
        {
            processor_->error_cnt_ = 0;
            processor_->is_ekf_init = false;
            processor_->is_imm_init = false;
        }
        else
        {
            param_mutex_.lock();
            auto aiming_point_world = std::move(processor_->predictor(target, sleep_time));
            // RCLCPP_INFO(this->get_logger(), "Predict...");
            param_mutex_.unlock();
            pred_point_world = *aiming_point_world;
            
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
            aiming_point_cam = processor_->coordsolver_.worldToCam(*aiming_point_world, rmat_imu);
            // std::cout << "predict_cam: x:" << aiming_point_cam[0] << " y:" << aiming_point_cam[1] << " z:" << aiming_point_cam[2] << std::endl;
            
            angle = processor_->coordsolver_.getAngle(aiming_point_cam, rmat_imu);
            tracking_point_cam = {target_info.aiming_point_cam.x, target_info.aiming_point_cam.y, target_info.aiming_point_cam.z};
            tracking_angle = processor_->coordsolver_.getAngle(tracking_point_cam, rmat_imu);
        }

        // Gimbal info pub.
        GimbalMsg gimbal_info;
        gimbal_info.header.frame_id = "barrel_link";
        gimbal_info.header.stamp = target_info.header.stamp;
        gimbal_info.pitch = abs(angle[1]) >= 45.0 ? 0.0 : angle[1];
        gimbal_info.yaw = abs(angle[0]) >= 45.0 ? 0.0 : angle[0];
        gimbal_info.distance = aiming_point_cam.norm();
        gimbal_info.is_target = !target.is_target_lost;
        gimbal_info.is_switched = target_info.target_switched;
        gimbal_info.is_spinning = target_info.is_spinning;
        gimbal_info_pub_->publish(std::move(gimbal_info));

        if(this->debug_)
        {
            if(!target.is_target_lost)
            {
                GimbalMsg tracking_info;
                tracking_info.header.frame_id = "barrel_link1";
                tracking_info.header.stamp = target_info.header.stamp;
                tracking_info.pitch =  abs(tracking_angle[1]) >= 45.0 ? 0.0 : tracking_angle[1];
                tracking_info.yaw = abs(tracking_angle[0]) >= 45.0 ? 0.0 : tracking_angle[0];
                tracking_info.distance = tracking_point_cam.norm();
                tracking_info.is_target = !target.is_target_lost;
                tracking_info.is_switched = target_info.target_switched;
                tracking_info.is_spinning = target_info.is_spinning;
                gimbal_info_pub_->publish(std::move(tracking_info));

                AutoaimMsg predict_info;
                predict_info.header.frame_id = "camera_link";
                predict_info.header.stamp = target_info.header.stamp;
                predict_info.header.stamp.nanosec += sleep_time;
                predict_info.aiming_point_cam.x = aiming_point_cam[0];
                predict_info.aiming_point_cam.y = aiming_point_cam[1];
                predict_info.aiming_point_cam.z = aiming_point_cam[2];
                predict_info.aiming_point_world.x = pred_point_world[0];
                predict_info.aiming_point_world.y = pred_point_world[1];
                predict_info.aiming_point_world.z = pred_point_world[2];
                predict_info.period = target_info.period;
                predict_info_pub_->publish(std::move(predict_info));
            }
        }

        if (debug_param_.show_img && !dst.empty()) 
        {
            // debug_mutex_.lock();
            if (!target.is_target_lost && this->debug_param_.show_predict)
            {
                // Get target 2d cornor points.
                for (int i = 0; i < 4; ++i)
                {
                    apex2d[i].x = target_info.point2d[i].x;
                    apex2d[i].y = target_info.point2d[i].y;
                }
                for(int i = 0; i < 4; i++)
                    cv::line(dst, apex2d[i % 4], apex2d[(i + 1) % 4], {0, 125, 255}, 1);
                // auto point_pred = predict_point_;
                cv::Point2f point_2d = processor_->coordsolver_.reproject(aiming_point_cam);
                cv::circle(dst, point_2d, 14, {255, 0, 125}, 2);
            }

            // predict_point_ = aiming_point_cam;
            // flag_ = true;
            // debug_mutex_.unlock();
            if(debug_param_.show_aim_cross)
            {
                line(dst, cv::Point2f(dst.size().width / 2, 0), cv::Point2f(dst.size().width / 2, dst.size().height), {0, 255, 0}, 1);
                line(dst, cv::Point2f(0, dst.size().height / 2), cv::Point2f(dst.size().width, dst.size().height / 2), {0, 255, 0}, 1);
            }

            char ch[50];
            sprintf(ch, "pitch_angle:%.2f yaw_angle:%.2f", tracking_angle[1], tracking_angle[0]);
            std::string angle_str = ch;
            putText(dst, angle_str, {dst.size().width / 2 + 50, 30}, cv::FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255});

            cv::namedWindow("pred", cv::WINDOW_AUTOSIZE);
            cv::imshow("pred", dst);
            cv::waitKey(1);
        }
        return;
    }

    /**
     * @brief 目标信息可视化
     * 
     * @param img 
     */
    void ArmorProcessorNode::imageProcessor(cv::Mat& img)
    {
        // if(this->debug_param_.show_predict)
        // {
        //     if(!img.empty())
        //     {
        //         debug_mutex_.lock();
        //         if(flag_)
        //         {
        //             for(int i = 0; i < 4; i++)
        //                 cv::line(img, apex2d[i % 4], apex2d[(i + 1) % 4], {0, 255, 255}, 5);
        //             auto point_pred = predict_point_;
        //             cv::Point2f point_2d = processor_->coordsolver_.reproject(point_pred);
        //             cv::circle(img, point_2d, 10, {255, 255, 0}, -1);
        //             flag_ = false;
        //         }
        //         debug_mutex_.unlock();
        //         // std::vector<cv::Point2f> points_pic(apex2d, apex2d + 4);
        //         // cv::RotatedRect points_pic_rrect = cv::minAreaRect(points_pic);
        //         // cv::Rect rect = points_pic_rrect.boundingRect();
        //         // cv::rectangle(img, rect, {255, 0, 255}, 5);
                
        //         cv::namedWindow("ekf_predict", cv::WINDOW_AUTOSIZE);
        //         cv::imshow("ekf_predict", img);
        //         cv::waitKey(1);
        //     }
        // }
    }

    /**
     * @brief 以共享内存的方式传输图像，方便可视化
     * 
     */
    void ArmorProcessorNode::imgCallbackThread()
    {
        cv::Mat img = cv::Mat(this->image_size_.height, this->image_size_.width, CV_8UC3);

        while(1)
        {
            // 读取共享内存图像数据
            memcpy(img.data, shared_memory_param_.shared_memory_ptr, this->image_size_.height * this->image_size_.width * 3);
            imageProcessor(img);
        }
    }

    /**
     * @brief 图像回调函数
     * 
     * @param img_info 图像数据信息
     */
    void ArmorProcessorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        if(!img_info)
            return;

        image_mutex_.lock();
        src_ = cv_bridge::toCvShare(img_info, "bgr8")->image;
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
        this->declare_parameter<double>("max_offset_value", 0.25);
        this->declare_parameter<double>("reserve_factor", 15.0);
        this->declare_parameter<double>("rotation_yaw", 0.0);
        this->declare_parameter<double>("rotation_pitch", 0.0);
        this->declare_parameter<double>("rotation_roll", 0.0);

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
        // predict_param_.max_offset_value = this->get_parameter("max_offset_value").as_double();
        // predict_param_.reserve_factor = this->get_parameter("reserve_factor").as_double();
        // predict_param_.rotation_yaw = this->get_parameter("rotation_yaw").as_double();
        // predict_param_.rotation_pitch = this->get_parameter("rotation_pitch").as_double();
        // predict_param_.rotation_roll = this->get_parameter("rotation_roll").as_double();
        
        //Debug param.
        debug_param_.show_img = this->get_parameter("show_img").as_bool();
        debug_param_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_param_.draw_predict = this->get_parameter("draw_predict").as_bool();
        debug_param_.show_predict = this->get_parameter("show_predict").as_bool();
        // debug_param_.x_axis_filter = this->get_parameter("x_axis_filter").as_bool();
        // debug_param_.y_axis_filter = this->get_parameter("y_axis_filter").as_bool();
        // debug_param_.z_axis_filter = this->get_parameter("z_axis_filter").as_bool();
        // debug_param_.print_delay = this->get_parameter("print_delay").as_bool();
        debug_param_.disable_filter = this->get_parameter("disable_filter").as_bool();
        debug_param_.disable_fitting = this->get_parameter("disable_fitting").as_bool();
        debug_param_.show_transformed_info = this->get_parameter("show_transformed_info").as_bool();
        debug_param_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();

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