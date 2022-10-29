/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-24 14:57:52
 * @LastEditTime: 2022-10-25 23:29:48
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/src/armor_processor_node.cpp
 */
#include "../include/armor_processor_node.hpp"

namespace armor_processor
{
    ArmorProcessorNode::ArmorProcessorNode(const rclcpp::NodeOptions& options)
    : Node("armor_processor", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting processor node...");
        processor_ = init_armor_processor();

        gimbal_info_pub_ = this->create_publisher<global_interface::msg::Gimbal>("/gimbal_info", 10);

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
            // spin_info_sub = this->create_subscription<global_interface::msg::SpinInfo>("/spin_info", 10,
            //     std::bind(&ArmorProcessorNode::spin_info_callback, this, std::placeholders::_1));
        target_info_sub_ = this->create_subscription<global_interface::msg::Target>("armor_detector/armor_info", 10,
            std::bind(&ArmorProcessorNode::target_info_callback, this, std::placeholders::_1));
        // }

    }

    ArmorProcessorNode::~ArmorProcessorNode()
    {
        
    }

    std::unique_ptr<Processor> ArmorProcessorNode::init_armor_processor()
    {
        PredictParam predict_param;
        // std::cout << 1 << std::endl;

        this->declare_parameter<double>("bullet_speed", 28.0);
        this->declare_parameter<int>("max_time_delta", 1000);
        this->declare_parameter<int>("max_cost", 509);
        this->declare_parameter<int>("max_v", 8);
        this->declare_parameter<int>("min_fitting_lens", 10);
        this->declare_parameter<int>("shoot_delay", 100);
        this->declare_parameter<int>("window_size", 3);
        predict_param.bullet_speed = this->get_parameter("bullet_speed").as_double();
        predict_param.max_time_delta = this->get_parameter("max_time_delta").as_int();
        predict_param.max_cost = this->get_parameter("max_cost").as_int();
        predict_param.max_v = this->get_parameter("max_v").as_int();
        predict_param.min_fitting_lens = this->get_parameter("min_fitting_lens").as_int();
        predict_param.shoot_delay = this->get_parameter("shoot_delay").as_int();
        predict_param.window_size = this->get_parameter("window_size").as_int();

        DebugParam debug_param;
        // std::cout << 2 << std::endl;

        this->declare_parameter("disable_fitting", false);
        this->declare_parameter("draw_predict", false);
        this->declare_parameter("using_imu", false);
        this->declare_parameter("show_predict", true);
        this->declare_parameter("show_transformed_info", true);
        debug_param.disable_fitting = this->get_parameter("disable_fitting").as_bool();
        debug_param.draw_predict = this->get_parameter("draw_predict").as_bool();
        debug_param.using_imu = this->get_parameter("using_imu").as_bool();
        debug_param.show_predict = this->get_parameter("show_predict").as_bool();
        debug_param.show_transformed_info = this->get_parameter("show_transformed_info").as_bool();

        // std::cout << 3 << std::endl;
               
        std::string filter_param_path;
        this->declare_parameter<std::string>("filter_param_path", "src/global_user/config/filter_param.yaml");
        filter_param_path = this->get_parameter("filter_param_path").as_string();

        return std::make_unique<Processor>(predict_param, debug_param, filter_param_path);
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
        if(target_info.target_switched)
        {
            RCLCPP_INFO(this->get_logger(), "Target switched...");

            Eigen::Vector3d aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};
            auto angle = processor_->coordsolver_.getAngle(aiming_point, processor_->rmat_imu);

            global_interface::msg::Gimbal gimbal_info;
            gimbal_info.pitch = angle[0];
            gimbal_info.yaw = angle[1]; 
            gimbal_info_pub_->publish(gimbal_info);
        }
        else
        {
            Eigen::Vector3d aiming_point;
            aiming_point = {target_info.aiming_point.x, target_info.aiming_point.y, target_info.aiming_point.z};

            auto aiming_point_world = processor_->armor_predictor_.predict(aiming_point, target_info.timestamp);
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
            gimbal_info_pub_->publish(gimbal_info);
        }

        return;
    }

    // void ArmorProcessorNode::spin_info_callback(const global_interface::msg::SpinInfo::SharedPtr msg) const
    // {
    //     return;
    // }
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