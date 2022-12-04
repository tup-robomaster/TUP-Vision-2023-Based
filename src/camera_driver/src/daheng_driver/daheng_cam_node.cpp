/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-09 14:25:39
 * @LastEditTime: 2022-12-03 21:21:10
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/daheng_driver/daheng_cam_node.cpp
 */
#include "../../include/daheng_driver/daheng_cam_node.hpp"

using namespace std::chrono_literals;

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
        {
            // Video save
            frame_cnt = 0;
            const std::string &storage_location = "src/camera_driver/video/";
            char now[64];
            std::time_t tt;
            struct tm *ttime;
            int width = IMAGE_WIDTH;
            int height = IMAGE_HEIGHT;

            tt = time(nullptr);
            ttime = localtime(&tt);
            strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  // 以时间为名字
            std::string now_string(now);
            std::string path(std::string(storage_location + now_string).append(".avi"));
            video_writer_ = std::make_shared<cv::VideoWriter>(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(width, height));    // Avi format
            is_first_loop = true;
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
        
        // this->declare_parameter("image_width", 1280);
        // this->image_height = this->declare_parameter("image_height", 1024);
        // this->daheng_cam_id = this->declare_parameter("daheng_cam_id", 0);
        this->frame_id = this->declare_parameter("frame_id", "daheng_cam");

        this->image_width = this->get_parameter("image_width").as_int();
        this->image_height = this->get_parameter("image_height").as_int();
        this->frame_id = this->get_parameter("frame_id").as_string();

        // acquisition system clock
        last_frame = std::chrono::steady_clock::now();

        // open DaHengCam
        if(!daheng_cam->open())
        {
            RCLCPP_WARN(this->get_logger(), "Camera open failed!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Camera open success!");
        }

        //
        this->declare_parameter("using_shared_memory", false);
        using_shared_memory = this->get_parameter("using_shared_memory").as_bool();
        if(using_shared_memory)
        {
            /**
             * @brief 创建图像数据共享内存空间
             * 
             */
            // 生成key
            key_ = ftok("./", 5);
            
            // 返回内存id
            shared_memory_id_ = shmget(key_, IMAGE_HEIGHT * IMAGE_WIDTH * 3, IPC_CREAT | 0666 | IPC_EXCL);
            if(shared_memory_id_ == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Get shared memory failed...");
            }

            //映射到内存地址
            this->shared_memory_ptr = shmat(shared_memory_id_, 0, 0);
            if(shared_memory_ptr == (void*)-1)
            {
                RCLCPP_ERROR(this->get_logger(), "Remapping shared memory failed...");
            }

            //内存写入线程
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
        {
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DahengCamNode::paramsCallback, this, std::placeholders::_1));
        }
    }

    DahengCamNode::~DahengCamNode()
    {
        if(using_shared_memory)
        {
            //解除共享内存映射
            if(this->shared_memory_ptr)
            {
                if(shmdt(this->shared_memory_ptr) == -1)
                {
                    RCLCPP_ERROR(this->get_logger(), "Dissolution remapping failed...");
                }
            }
            
            //销毁共享内存
            if(shmctl(shared_memory_id_, IPC_RMID, NULL) == -1)
            {
                RCLCPP_ERROR(this->get_logger(), "Destroy shared memory failed...");        
            }
        }
    }

    std::unique_ptr<DaHengCam> DahengCamNode::init_daheng_cam()
    {
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
        if(using_shared_memory)
        {
            while(1)
            {
                if(!daheng_cam->get_frame(frame))
                {
                    RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
                    return;
                }

                if(!frame.empty())
                {
                    memcpy(this->shared_memory_ptr, frame.data, IMAGE_HEIGHT * IMAGE_WIDTH * 3);
                }

                if(save_video_)
                {
                    // Video recorder
                    frame_cnt++;
                    if(frame_cnt % 10 == 0)
                    {
                        frame_cnt = 0;
                        //异步读写加速,避免阻塞生产者
                        if (is_first_loop)
                            is_first_loop = false;
                        else
                            writer_video_.wait();
                        writer_video_ = std::async(std::launch::async, [&](){video_writer_->write(frame);});
                    }
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

            auto now = std::chrono::steady_clock::now();
            this->last_frame = now;
            rclcpp::Time timestamp = this->get_clock()->now();
            sensor_msgs::msg::Image::UniquePtr msg = convert_frame_to_msg(frame);
            image_pub->publish(std::move(msg));

            if(save_video_)
            {
                // Video recorder
                frame_cnt++;
                if(frame_cnt % 3 == 0)
                {
                    frame_cnt = 0;
                    //异步读写加速,避免阻塞生产者
                    if (is_first_loop)
                        is_first_loop = false;
                    else
                        writer_video_.wait();
                    writer_video_ = std::async(std::launch::async, [&](){video_writer_->write(frame);});
                }
            }
            
            // cv::namedWindow("daheng_cam_frame", cv::WINDOW_AUTOSIZE);
            // cv::imshow("daheng_cam_frame", frame);
            // cv::waitKey(1);
        }
    }

    rcl_interfaces::msg::SetParametersResult DahengCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            if(param.get_name() == "exposure_time")
            {
                if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    if(param.as_int() >= 0)
                    {
                        RCLCPP_INFO(this->get_logger(), 
                            "Param callback: Receive update to parameter\"%s\" of type %s: \"%ld\"",
                            param.get_name().c_str(),
                            param.get_type_name().c_str(),
                            param.as_int()
                        );

                        this->daheng_cam_param_.exposure_time = param.as_int();
                        this->daheng_cam->SetExposureTime(this->daheng_cam_param_.exposure_time);
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "exposure_gain")
            {
                if(param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    if(param.as_int() >= 0)
                    {
                        RCLCPP_INFO(this->get_logger(), 
                            "Param callback: Receive update to parameter\"%s\" of type %s: \"%ld\"",
                            param.get_name().c_str(),
                            param.get_type_name().c_str(),
                            param.as_int()
                        );
                        this->daheng_cam_param_.exposure_gain = param.as_int();
                        this->daheng_cam->SetGAIN(3, this->daheng_cam_param_.exposure_gain);
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "balance_b")
            {
                if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    if(param.as_double() >= 0)
                    {
                        RCLCPP_INFO(this->get_logger(), 
                            "Param callback: Receive update to parameter\"%s\" of type %s: \"%lf\"",
                            param.get_name().c_str(),
                            param.get_type_name().c_str(),
                            param.as_double()
                        );
                        this->daheng_cam_param_.balance_b = param.as_double();
                        this->daheng_cam->Set_BALANCE(0, this->daheng_cam_param_.balance_b);
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "balance_g")
            {
                if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    if(param.as_double() >= 0)
                    {
                        RCLCPP_INFO(this->get_logger(), 
                            "Param callback: Receive update to parameter\"%s\" of type %s: \"%lf\"",
                            param.get_name().c_str(),
                            param.get_type_name().c_str(),
                            param.as_double()
                        );
                        this->daheng_cam_param_.balance_g = param.as_double();
                        this->daheng_cam->Set_BALANCE(1, this->daheng_cam_param_.balance_g);
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "balance_r")
            {
                if(param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
                {
                    if(param.as_double() >= 0)
                    {
                        RCLCPP_INFO(this->get_logger(), 
                            "Param callback: Receive update to parameter\"%s\" of type %s: \"%lf\"",
                            param.get_name().c_str(),
                            param.get_type_name().c_str(),
                            param.as_double()
                        );
                        this->daheng_cam_param_.balance_r = param.as_double();
                        this->daheng_cam->Set_BALANCE(2, this->daheng_cam_param_.balance_r);
                        result.successful = true;
                    }
                }
            }
        }
        
        return result;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver::DahengCamNode>());
    rclcpp::shutdown();
    
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::DahengCamNode)

