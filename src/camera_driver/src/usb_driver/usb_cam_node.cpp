/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:53
 * @LastEditTime: 2022-11-19 16:41:15
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam_node.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{
    usb_cam_node::usb_cam_node(const rclcpp::NodeOptions& option)
    : Node("usb_driver", option), is_filpped(false)
    {
        RCLCPP_WARN(this->get_logger(), "Camera driver node...");
       
        usb_cam_ = init_usb_cam();
        
        frame_pub = this->create_publisher<sensor_msgs::msg::Image>("usb_img", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
        
        // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
        // camera_info_pub = image_transport::create_camera_publisher(this, "image");

        // auto camera_calibr_file = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
        // cam_info_manager->loadCameraInfo(camera_calibr_file);

        cap.open(usb_cam_->usb_cam_params_.camera_id);
        if(cap.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "open camera success!");
        }

        cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_->usb_cam_params_.image_width);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_->usb_cam_params_.image_height);

        last_frame = std::chrono::steady_clock::now();

        timer = this->create_wall_timer(1ms, std::bind(&usb_cam_node::image_callback, this));

        bool debug_;
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&usb_cam_node::paramsCallback, this, std::placeholders::_1));
            
            //创建参数订阅者监测参数改变情况
            //既可以用于本节点也可以是其他节点
            // param_subscriber_ = std::make_shared<ParamSubscriber>(this);

            // //设置回调用于本节点参数修改
            // ParamCallbackType cb = [this](const rclcpp::Parameter& p)
            // {
            //     RCLCPP_INFO(this->get_logger(), 
            //         "Param callback: Receive update to parameter\"%s\" of type %s: \"%ld\"",
            //         p.get_name().c_str(),
            //         p.get_type_name().c_str(),
            //         p.as_int()
            //     );

            //     for(int ii = 0; ii < this->params_vec_.size(); ii++)
            //     {
            //         if(this->params_vec_[ii] == p.get_name().c_str())
            //         {
            //             index = ii;
            //             break;
            //         }
            //         // std::cout << ii << std::endl;
            //     }
            //     switch (index)
            //     {
            //     case 0:
            //         this->usb_cam_->usb_cam_params_.camera_id = p.as_int();
            //         break;
            //     case 1:
            //         this->usb_cam_->usb_cam_params_.frame_id = p.as_int();
            //         break;
            //     case 2:
            //         this->usb_cam_->usb_cam_params_.image_width = p.as_int();
            //         break;
            //     case 3:
            //         this->usb_cam_->usb_cam_params_.image_height = p.as_int();
            //         break;
            //     case 4:
            //         this->usb_cam_->usb_cam_params_.fps = p.as_int();
            //         break;
            //     default:
            //         break;
            //     }
            // };

            // param_cb_handle_ = param_subscriber_->add_parameter_callback("ParamCallback", cb);
        }
    }

    std::unique_ptr<usb_cam> usb_cam_node::init_usb_cam()
    {
        // index = 0;
        usb_cam_params usb_cam_params_;
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("frame_id", "usb_image");
        this->declare_parameter("image_width", 480);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("fps", 30);
        // this->params_vec_.push_back("camera_id");
        // this->params_vec_.push_back("frame_id");
        // this->params_vec_.push_back("image_width");
        // this->params_vec_.push_back("image_height");
        // this->params_vec_.push_back("fps");

        usb_cam_params_.camera_id = this->get_parameter("camera_id").as_int();
        usb_cam_params_.frame_id = this->get_parameter("frame_id").as_string();
        usb_cam_params_.image_width = this->get_parameter("image_width").as_int();
        usb_cam_params_.image_height = this->get_parameter("image_height").as_int();
        usb_cam_params_.fps = this->get_parameter("fps").as_int();

        printf("camera_id: %d\n", usb_cam_params_.camera_id);

        return std::make_unique<usb_cam>(usb_cam_params_);
    }

    rcl_interfaces::msg::SetParametersResult usb_cam_node::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "debug";
        for(const auto& param : params)
        {
            if(param.get_name() == "camera_id")
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

                        this->usb_cam_->usb_cam_params_.camera_id = param.as_int();
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "frame_id")
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
                        this->usb_cam_->usb_cam_params_.frame_id = param.as_int();
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "image_width")
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
                        this->usb_cam_->usb_cam_params_.image_width = param.as_int();
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "image_height")
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
                        this->usb_cam_->usb_cam_params_.image_height = param.as_int();
                        result.successful = true;
                    }
                }
            }
            if(param.get_name() == "fps")
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
                        this->usb_cam_->usb_cam_params_.fps = param.as_int();
                        result.successful = true;
                    }
                }
            }
        }
        
        return result;
    }

    std::shared_ptr<sensor_msgs::msg::Image> usb_cam_node::convert_frame_to_message(cv::Mat &frame)
    {
        std_msgs::msg::Header header;

        sensor_msgs::msg::Image ros_image;

        // cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);
        // cv::imshow("raw", frame);
        // cv::waitKey(0);

        if(frame.rows != usb_cam_->usb_cam_params_.image_width || frame.cols != usb_cam_->usb_cam_params_.image_height)
        { 
            cv::resize(frame, frame, cv::Size(usb_cam_->usb_cam_params_.image_width, usb_cam_->usb_cam_params_.image_height));
            RCLCPP_INFO(this->get_logger(), "resize frame...");
        }

        ros_image.header = header;
        ros_image.height = frame.rows;
        ros_image.width = frame.cols;
        ros_image.encoding = "bgr8";
        
        ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        ros_image.is_bigendian = false;
        ros_image.data.assign(frame.datastart, frame.dataend);

        RCLCPP_INFO(this->get_logger(), "copy frame...");
        // ros_image.is_bigendian = (std::endian::native == std::endian::big);
        // ros_image.step = frame.cols * frame.elemSize();
        // size_t size = ros_image.step * frame.rows;
        
        // ros_image.data.resize(size);
        // RCLCPP_INFO(this->get_logger(), "resize ros frame...");

        // RCLCPP_INFO(this->get_logger(), "ros_image: %d %d", ros_image.height, ros_image.width);
        // RCLCPP_INFO(this->get_logger(), "raw_image: %d %d", frame.size().height, frame.size().width);
        // RCLCPP_INFO(this->get_logger(), "size: %ld", size / frame.size().width);

        // if(frame.isContinuous())
        // {
        //     RCLCPP_INFO(this->get_logger(), "copy frame...");
        //     memcpy(reinterpret_cast<char*>(&ros_image.data[0]), frame.data, frame.size().height * frame.size().width);
        // }
        // else
        // {
        //     // copy row by row
        //     RCLCPP_INFO(this->get_logger(), "frame is not continuous...");
        //     uchar *ros_data_ptr = reinterpret_cast<uchar*>(&ros_image.data[0]);
        //     uchar *cv_data_ptr = frame.data;
        //     for(int i = 0; i < frame.rows; i++)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "frame copy row by row...");
        //         memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
        //         ros_data_ptr += ros_image.step;
        //         cv_data_ptr += frame.step;
        //     }
        // }

        auto msg_ptr = std::make_shared<sensor_msgs::msg::Image>(ros_image);

        RCLCPP_INFO(this->get_logger(), "msg_ptr...");
  
        cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);
        cv::imshow("raw", frame);
        cv::waitKey(1);
        return msg_ptr;
    }

    void usb_cam_node::image_callback()
    {
        cap >> frame;

        // RCLCPP_INFO(this->get_logger(), "frame stream...");
        auto now = std::chrono::steady_clock::now();
        
        // std::cout << "image_width:" << this->usb_cam_->usb_cam_params_.image_width << std::endl;

        if(!frame.empty() && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame).count() > 1 / usb_cam_->usb_cam_params_.fps * 1000)
        {
            last_frame = now;
            
            // if(!is_filpped)
            // {
            //     RCLCPP_INFO(this->get_logger(), "is_filpped...");
            //     image_msg = convert_frame_to_message(frame);
            //     RCLCPP_INFO(this->get_logger(), "convert success...");
            // }
            // else
            // {
            //     //flip the image
            //     // cv::filp(frame, filpped_frame, 1);
            //     image_msg = convert_frame_to_message(frame);
            // }

            // Put the message into a queue to be processed by the middleware.
            // This call is non-blocking.
            // RCLCPP_INFO(this->get_logger(), "get info...");
            // sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg(
            //     new sensor_msgs::msg::CameraInfo(cam_info_manager->getCameraInfo()));
            

            rclcpp::Time timestamp = this->get_clock()->now();

            // image_msg->header.stamp = timestamp;
            // image_msg->header.frame_id = frame_id;
            
            // camera_info_msg->header.stamp = timestamp;
            // camera_info_msg->header.frame_id = frame_id;
            sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>();

            msg->header.stamp = timestamp;
            msg->header.frame_id = usb_cam_->usb_cam_params_.frame_id;
            msg->encoding = "bgr8";
            msg->width = frame.cols;
            msg->height = frame.rows;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
            msg->is_bigendian = false;
            msg->data.assign(frame.datastart, frame.dataend);

            // camera_info_pub.publish(image_msg, camera_info_msg);
            frame_pub->publish(std::move(msg));

            cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
            cv::imshow("raw_image", frame);
            cv::waitKey(1);
        }
    }
} //namespace usb_cam_node

int main(int argc, char** argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;
    auto usb_cam_node = std::make_shared<camera_driver::usb_cam_node>(options);

    exec.add_node(usb_cam_node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::usb_cam_node)