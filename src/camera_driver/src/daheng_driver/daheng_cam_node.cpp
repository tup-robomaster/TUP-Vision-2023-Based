/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-09 14:25:39
 * @LastEditTime: 2022-12-26 23:15:40
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/daheng_driver/daheng_cam_node.cpp
 */
#include "../../include/daheng_driver/daheng_cam_node.hpp"

using namespace std::placeholders;

namespace camera_driver
{
    DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options)
    : Node("daheng_driver", options)
    {
        RCLCPP_WARN(this->get_logger(), "Camera driver node...");
        
        // camera params initialize 
        daheng_cam = init_daheng_cam();

        this->declare_parameter<bool>("save_video", false);
        save_video_ = this->get_parameter("save_video").as_bool();
        if(save_video_)
        {   // Video save.
            videoRecorder(video_record_param_);
        }

        //QoS    
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();

        // create img publisher
        this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("daheng_img", qos);
        
        this->declare_parameter("frame_id", "daheng_cam");
        this->frame_id = this->get_parameter("frame_id").as_string();
        
        // this->declare_parameter("image_width", 1280);
        // this->image_height = this->declare_parameter("image_height", 1024);
        // this->daheng_cam_id = this->declare_parameter("daheng_cam_id", 0);

        this->image_width = this->get_parameter("image_width").as_int();
        this->image_height = this->get_parameter("image_height").as_int();

        // Acquisition system clock.
        last_frame_ = this->get_clock()->now();

        // Open daheng cam.
        if(!daheng_cam->open())
            RCLCPP_WARN(this->get_logger(), "Camera open failed!");

        // Use shared memory.
        this->declare_parameter("using_shared_memory", false);
        using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();
        if(using_shared_memory_)
        {
            try
            {
                if(!setSharedMemory(shared_memory_param_, 5, this->image_width, this->image_height))
                    RCLCPP_ERROR(this->get_logger(), "Shared memory init failed...");
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }

            // 内存写入线程
            memory_write_thread_ = std::thread(&DahengCamNode::image_callback, this);        
        }
        else
        {
            timer = this->create_wall_timer(1ms, std::bind(&DahengCamNode::image_callback, this));
        }

        bool debug_;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {   //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DahengCamNode::paramsCallback, this, _1));
        }
    }

    DahengCamNode::~DahengCamNode()
    {
        if(using_shared_memory_)
        {
            if(!destorySharedMemory(shared_memory_param_))
                RCLCPP_ERROR(this->get_logger(), "Destory shared memory failed...");
        }
    }

    std::unique_ptr<sensor_msgs::msg::Image> DahengCamNode::convert_frame_to_msg(cv::Mat frame)
    {
        std_msgs::msg::Header header;
        sensor_msgs::msg::Image ros_image;

        if(frame.size().width != image_width || frame.size().height != image_height)
        {
            cv::resize(frame, frame, cv::Size(image_width, image_height));
            RCLCPP_INFO(this->get_logger(), "resize frame...");
        }

        ros_image.header.frame_id = this->frame_id;
        ros_image.header.stamp = this->get_clock()->now();
        ros_image.width = frame.size().width;
        ros_image.height = frame.size().height;
        ros_image.encoding = "bgr8";
        
        ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);  
        ros_image.is_bigendian = false;
        ros_image.data.assign(frame.datastart, frame.dataend);

        auto msg_ptr = std::make_unique<sensor_msgs::msg::Image>(ros_image);      
        return msg_ptr;
    }

    void DahengCamNode::image_callback()
    {
        if(using_shared_memory_)
        {
            while(1)
            {
                if(!daheng_cam->get_frame(frame))
                {
                    RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
                    return;
                }

                if(!frame.empty())
                    memcpy(shared_memory_param_.shared_memory_ptr, frame.data, DAHENG_IMAGE_HEIGHT * DAHENG_IMAGE_WIDTH * 3);

                if(save_video_)
                {   // Video recorder.
                    videoRecorder(video_record_param_, &frame);
                }

                // cv::namedWindow("daheng_cam_frame", cv::WINDOW_AUTOSIZE);
                // cv::imshow("daheng_cam_frame", frame);
                // cv::waitKey(1);
            }
        }
        else
        {
            if(!daheng_cam->get_frame(frame))
            {
                RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
                return;
            }

            auto now = this->get_clock()->now();;
            this->last_frame_ = now;
            // rclcpp::Time timestamp = now;
            sensor_msgs::msg::Image::UniquePtr msg = convert_frame_to_msg(frame);
            image_pub->publish(std::move(msg));

            if(save_video_)
            {   // Video recorder.
                videoRecorder(video_record_param_, &frame);
            }
            
            // cv::namedWindow("daheng_cam_frame", cv::WINDOW_AUTOSIZE);
            // cv::imshow("daheng_cam_frame", frame);
            // cv::waitKey(1);
        }
    }

    /**
     * @brief 动态调参
     * @param 参数服务器参数
     * @return 是否修改参数成功
    */
    bool DahengCamNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            daheng_cam->SetExposureTime(param.as_int());
            break;
        case 1:
            daheng_cam->SetGAIN(3, param.as_int());
            break;
        case 2:
            daheng_cam->Set_BALANCE(0, param.as_double());
            break;
        case 3:
            daheng_cam->Set_BALANCE(1, param.as_double());
            break;
        case 4:
            daheng_cam->Set_BALANCE(2, param.as_double());
            break;
        default:
            break;
        }
    }

    rcl_interfaces::msg::SetParametersResult DahengCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            result.successful = setParam(param);
        }
        return result;

        // for(const auto& param : params)
        // {
        //     if(param.get_name() == "exposure_time")
        //     {
        //         if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        //         {
        //             if(param.as_int() >= 0)
        //             {
        //                 RCLCPP_INFO(this->get_logger(), 
        //                     "Param callback: Receive update to parameter\"%s\" of type %s: \"%ld\"",
        //                     param.get_name().c_str(),
        //                     param.get_type_name().c_str(),
        //                     param.as_int()
        //                 );

        //                 this->daheng_cam_param_.exposure_time = param.as_int();
        //                 this->daheng_cam->SetExposureTime(this->daheng_cam_param_.exposure_time);
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "exposure_gain")
        //     {
        //         if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        //         {
        //             if(param.as_int() >= 0)
        //             {
        //                 RCLCPP_INFO(this->get_logger(), 
        //                     "Param callback: Receive update to parameter\"%s\" of type %s: \"%ld\"",
        //                     param.get_name().c_str(),
        //                     param.get_type_name().c_str(),
        //                     param.as_int()
        //                 );
        //                 this->daheng_cam_param_.exposure_gain = param.as_int();
        //                 this->daheng_cam->SetGAIN(3, this->daheng_cam_param_.exposure_gain);
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "balance_b")
        //     {
        //         if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        //         {
        //             if(param.as_double() >= 0)
        //             {
        //                 RCLCPP_INFO(this->get_logger(), 
        //                     "Param callback: Receive update to parameter\"%s\" of type %s: \"%lf\"",
        //                     param.get_name().c_str(),
        //                     param.get_type_name().c_str(),
        //                     param.as_double()
        //                 );
        //                 this->daheng_cam_param_.balance_b = param.as_double();
        //                 this->daheng_cam->Set_BALANCE(0, this->daheng_cam_param_.balance_b);
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "balance_g")
        //     {
        //         if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        //         {
        //             if(param.as_double() >= 0)
        //             {
        //                 RCLCPP_INFO(this->get_logger(), 
        //                     "Param callback: Receive update to parameter\"%s\" of type %s: \"%lf\"",
        //                     param.get_name().c_str(),
        //                     param.get_type_name().c_str(),
        //                     param.as_double()
        //                 );
        //                 this->daheng_cam_param_.balance_g = param.as_double();
        //                 this->daheng_cam->Set_BALANCE(1, this->daheng_cam_param_.balance_g);
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "balance_r")
        //     {
        //         if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        //         {
        //             if(param.as_double() >= 0)
        //             {
        //                 RCLCPP_INFO(this->get_logger(), 
        //                     "Param callback: Receive update to parameter\"%s\" of type %s: \"%lf\"",
        //                     param.get_name().c_str(),
        //                     param.get_type_name().c_str(),
        //                     param.as_double()
        //                 );
        //                 this->daheng_cam_param_.balance_r = param.as_double();
        //                 this->daheng_cam->Set_BALANCE(2, this->daheng_cam_param_.balance_r);
        //                 result.successful = true;
        //             }
        //         }
        //     }
        // }
        
        // return result;
    }

    std::unique_ptr<DaHengCam> DahengCamNode::init_daheng_cam()
    {
        param_map_ = 
        {
            {"exposure_time", 0},
            {"exposure_gain", 1},
            {"balance_b", 2},
            {"balance_g", 3},
            {"balance_r", 4}
        };

        this->declare_parameter("daheng_cam_id", 1);
        this->declare_parameter("image_width", 1280);
        this->declare_parameter("image_height", 1024);
        this->declare_parameter("width_scale", 1);
        this->declare_parameter("height_scale", 1);
        this->declare_parameter("exposure_time", 6000);
        this->declare_parameter("exposure_gain", 14);
        this->declare_parameter("exposure_gain_b", 0);
        this->declare_parameter("exposure_gain_g", 0);
        this->declare_parameter("exposure_gain_r", 0);
        this->declare_parameter("auto_balance", false);
        this->declare_parameter("balance_b", 1.56);
        this->declare_parameter("balance_g", 1.0); 
        this->declare_parameter("balance_r", 1.548);

        daheng_cam_param_.daheng_cam_id = this->get_parameter("daheng_cam_id").as_int();
        daheng_cam_param_.image_width = this->get_parameter("image_width").as_int();
        daheng_cam_param_.image_height = this->get_parameter("image_height").as_int();
        daheng_cam_param_.width_scale = this->get_parameter("width_scale").as_int();
        daheng_cam_param_.height_scale = this->get_parameter("height_scale").as_int();
        daheng_cam_param_.exposure_time = this->get_parameter("exposure_time").as_int();
        daheng_cam_param_.exposure_gain = this->get_parameter("exposure_gain").as_int();
        daheng_cam_param_.exposure_gain_b = this->get_parameter("exposure_gain_b").as_int();
        daheng_cam_param_.exposure_gain_g = this->get_parameter("exposure_gain_g").as_int();
        daheng_cam_param_.exposure_gain_r = this->get_parameter("exposure_gain_r").as_int();
        daheng_cam_param_.auto_balance = this->get_parameter("auto_balance").as_bool();
        daheng_cam_param_.balance_b = this->get_parameter("balance_b").as_double();
        daheng_cam_param_.balance_g = this->get_parameter("balance_g").as_double();
        daheng_cam_param_.balance_r = this->get_parameter("balance_r").as_double();

        return std::make_unique<DaHengCam>(daheng_cam_param_);
    }
}

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<camera_driver::DahengCamNode>());
//     rclcpp::shutdown();
    
//     return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::DahengCamNode)

