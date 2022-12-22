/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-19 23:11:19
 * @LastEditTime: 2022-12-22 23:53:54
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
            buff_processor_ = init_buff_processor();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        if(!predict_param_.is_initialized)
        {
            predict_param_.coordsolver_->loadParam(predict_param_.path_param_.camera_param_path, predict_param_.path_param_.camera_name);
            predict_param_.is_initialized = true;
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
        gimbal_info_pub_ = this->create_publisher<GimbalMsg>("/buff_processor/gimbal_info", qos);

        // 发布预测点信息
        predict_info_pub_ = this->create_publisher<BuffMsg>("/buff_predict_info", qos);

        // 订阅待打击目标信息
        target_info_sub_ = this->create_subscription<BuffMsg>("/buff_info", qos,
            std::bind(&BuffProcessorNode::target_info_callback, this, _1));
    }

    BuffProcessorNode::~BuffProcessorNode()
    {}

    void BuffProcessorNode::target_info_callback(const BuffMsg& target_info)
    {
        if(target_info.target_switched)
        {
            RCLCPP_INFO(this->get_logger(), "Target switched...");    
        }
        TargetInfo target;
        if(predict_param_.predictor(target_info, target))
        {
            GimbalMsg gimbal_msg;
            gimbal_msg.header.frame_id = "gimbal";
            gimbal_msg.header.stamp = this->get_clock()->now();
            gimbal_msg.pitch = target.angle[0];
            gimbal_msg.yaw = target.angle[1];
            gimbal_msg.distance = target.hit_point_cam.norm();
            gimbal_msg.is_switched = target.target_switched;
            
            gimbal_info_pub_->publish(gimbal_msg);

            BuffMsg predict_info;
            predict_info.header.frame_id = "buff_predictor";
            predict_info.header.stamp = this->get_clock()->now();
            predict_info.predict_point.x = target.hit_point_cam[0];
            predict_info.predict_point.y = target.hit_point_cam[1];
            predict_info.predict_point.z = target.hit_point_cam[2];

            predict_info_pub_->publish(predict_point);
        }
    }

    rcl_interfaces::msg::SetParametersResult BuffProcessorNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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

    bool BuffProcessorNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            buff_processor_->setPredictorParam(param.as_double(), 1);
            break;
        case 1:
            buff_processor_->setPredictorParam(param.as_double(), 2);
            break;
        case 2:
            buff_processor_->setPredictorParam(param.as_double(), 3);
            break;
        case 3:
            buff_processor_->setPredictorParam(param.as_double(), 4);
            break;
        case 4:
            buff_processor_->setPredictorParam(param.as_double(), 5);
            break;
        case 5:
            buff_processor_->setPredictorParam(param.as_double(), 6);
            break;
        case 6:
            buff_processor_->setPredictorParam(param.as_double(), 7);
            break;
        case 7:
            buff_processor_->setPredictorParam(param.as_double(), 8);
            break;
        case 8:
            buff_processor_->setPredictorParam(param.as_double(), 9);
            break;
        case 9:
            buff_processor_->setPredictorParam(param.as_double(), 10);
            break;
        case 10:
            buff_processor_->setPredictorParam(param.as_double(), 11);
            break;
        case 11:
            buff_processor_->setDebugParam(param.as_bool(), 1);
            break;        
        case 12:
            buff_processor_->setDebugParam(param.as_bool(), 2);
            break;        
        default:
            break;
        }
        return true;
    }


    std::unique_ptr<Processor> BuffProcessorNode::init_buff_processor()
    {
        param_map_ = 
        {
            {"bullet_speed", 0},
            {"delay_big", 1},
            {"delay_small", 2},
            {"history_deque_len_cos", 3},
            {"history_deque_len_phase", 4},
            {"history_deque_len_uniform", 5},
            {"max_a", 6},
            {"max_rmse", 7},
            {"max_timespan", 8},
            {"max_v", 9},
            {"window_size", 10},
            {"show_predict", 11},
            {"using_imu", 12}
        };

        this->declare_parameter<std::string>("pf_path", "src/global_user/config/filter_param.yaml");
        this->declare_parameter<double>("bullet_speed", 28.0);
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
        this->get_parameter("bullet_speed", predict_param_.bullet_speed);
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

        this->declare_parameter<std::string>("camera_name", "KE0200110075");
        this->declare_parameter<std::string>("camera_param_path", "src/global_user/config/camera.yaml");
        this->get_parameter("camera_name", this->path_param_.camera_name);
        this->get_parameter("camera_param_path", this->path_param_.camera_param_path);

        this->declare_parameter<bool>("show_predict", true);
        this->declare_parameter<bool>("using_imu", false);
        this->get_parameter("show_predict", this->debug_param_.show_predict);
        this->get_parameter("using_imu", this->debug_param_.using_imu);

        return std::make_unique<Processor>(predict_param_, path_param_, debug_param_);
    }
} //namespace buff_processor

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_unique<buff_detector::BuffDetectorNode>());
    rclcpp::shutdown();
    
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(buff_processor::BuffProcessorNode)