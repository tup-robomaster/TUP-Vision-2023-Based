/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:53
 * @LastEditTime: 2023-02-02 15:10:09
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam_node.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace camera_driver
{
    UsbCamNode::UsbCamNode(const rclcpp::NodeOptions& option)
    : Node("usb_driver", option), is_filpped(false)
    {
        RCLCPP_INFO(this->get_logger(), "Camera driver node...");
        
        try
        {
            usb_cam_ = init_usb_cam();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing camera: %s", e.what());
        }        
        
        this->declare_parameter<bool>("save_video", false);
        save_video_ = this->get_parameter("save_video").as_bool();
        if(save_video_)
        {   // Video save.
            videoRecorder(video_record_param_);
        }

        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        // qos.transient_local();
        qos.durability_volatile();

        frame_pub = this->create_publisher<sensor_msgs::msg::Image>("usb_img", qos);
        
        // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
        // camera_info_pub = image_transport::create_camera_publisher(this, "image");

        // auto camera_calibr_file = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
        // cam_info_manager->loadCameraInfo(camera_calibr_file);
        this->declare_parameter<bool>("using_video", true);
        using_video_ = this->get_parameter("using_video").as_bool();
        this->declare_parameter<std::string>("video_path", " ");
        video_path_ = this->get_parameter("video_path").as_string();
        
        cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_height);

        // sleep(10);
        if(using_video_)
        {
            cout << video_path_ << endl;
            cap.open(video_path_);
            if(!cap.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Open camera failed!");
            }
        }
        else
        {
            cap.open(usb_cam_params_.camera_id);
            if(cap.isOpened())
            {
                RCLCPP_INFO(this->get_logger(), "Open camera success!");
            }
        }


        last_frame_ = this->get_clock()->now();

        // Using shared memory.
        this->declare_parameter("using_shared_memory", false);
        using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();
        if(using_shared_memory_)
        {
            try
            {
                if(!setSharedMemory(shared_memory_param_, 5, usb_cam_params_.image_width, usb_cam_params_.image_height))
                    RCLCPP_ERROR(this->get_logger(), "Shared memory init failed...");
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error while initializing shared memory: %s", e.what());
            }

            //内存写入线程
            memory_write_thread_ = std::thread(&UsbCamNode::image_callback, this);        
        }
        else
        {
            timer = this->create_wall_timer(1ms, std::bind(&UsbCamNode::image_callback, this));
        }

        bool debug_;
        this->declare_parameter<bool>("debug", false);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&UsbCamNode::paramsCallback, this, _1));
            RCLCPP_INFO(this->get_logger(), "Usb camera debug...");
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

            // param_cb_handle_ = param_subscriber_->add_parameter_callback("ParamCallback", cb);
        }
    }

    UsbCamNode::~UsbCamNode()
    {
        if(using_shared_memory_)
        {
            if(!destorySharedMemory(shared_memory_param_))
                RCLCPP_ERROR(this->get_logger(), "Destory shared memory failed...");
        }
    }

    std::shared_ptr<sensor_msgs::msg::Image> UsbCamNode::convert_frame_to_message(cv::Mat &frame)
    {
        std_msgs::msg::Header header;
        sensor_msgs::msg::Image ros_image;

        if(frame.rows != usb_cam_params_.image_width || frame.cols != usb_cam_params_.image_height)
        { 
            cv::resize(frame, frame, cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height));
            RCLCPP_INFO(this->get_logger(), "Resize frame...");
        }

        ros_image.header.frame_id = "usb_camera_link";
        ros_image.header.stamp = this->get_clock()->now();
        ros_image.height = frame.rows;
        ros_image.width = frame.cols;
        ros_image.encoding = "bgr8";
        
        ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        ros_image.is_bigendian = false;
        ros_image.data.assign(frame.datastart, frame.dataend);

        RCLCPP_INFO(this->get_logger(), "Copy frame...");
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

        // cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);
        // cv::imshow("raw", frame);
        // cv::waitKey(1);
        return msg_ptr;
    }

    void UsbCamNode::image_callback()
    {
        cap >> frame;
        
        // RCLCPP_INFO(this->get_logger(), "frame stream...");
        auto now = this->get_clock()->now();

        auto dt = (now.nanoseconds() - last_frame_.nanoseconds()) / 1e9;
        if(!frame.empty() && 
            dt > (1 / usb_cam_params_.fps))
        {
            last_frame_ = now;
            if(using_shared_memory_)
            {
                memcpy(shared_memory_param_.shared_memory_ptr, frame.data, USB_IMAGE_HEIGHT * USB_IMAGE_WIDTH * 3);
            }
            else
            {
                // std::cout << "Pub img..." << std::endl;

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

                msg->header.frame_id = usb_cam_params_.frame_id;
                msg->header.stamp = timestamp;
                msg->encoding = "bgr8";
                msg->width = frame.cols;
                msg->height = frame.rows;
                msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
                msg->is_bigendian = false;
                msg->data.assign(frame.datastart, frame.dataend);

                // camera_info_pub.publish(image_msg, camera_info_msg);
                frame_pub->publish(std::move(msg));
            }

            save_video_ = this->get_parameter("save_video").as_bool();
            if(save_video_)
            {   // Video recorder.
                videoRecorder(video_record_param_, &frame);
            }

            bool show_img = this->get_parameter("show_img").as_bool();
            if(show_img)
            {
                cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
                cv::imshow("raw_image", frame);
                cv::waitKey(2000);
            }

            if(using_video_)
                usleep(10000);
        }
    }

    bool UsbCamNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = param_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            usb_cam_params_.image_width = param.as_int();
            break;
        case 1:
            usb_cam_params_.image_height = param.as_int();
            break;
        case 2:
            usb_cam_params_.fps = param.as_int();
            break;
        default:
            break;
        }
        return true;
    }

    rcl_interfaces::msg::SetParametersResult UsbCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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
        //     if(param.get_name() == "camera_id")
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

        //                 this->usb_cam_->usb_cam_params_.camera_id = param.as_int();
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "frame_id")
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
        //                 this->usb_cam_->usb_cam_params_.frame_id = param.as_int();
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "image_width")
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
        //                 this->usb_cam_->usb_cam_params_.image_width = param.as_int();
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "image_height")
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
        //                 this->usb_cam_->usb_cam_params_.image_height = param.as_int();
        //                 result.successful = true;
        //             }
        //         }
        //     }
        //     if(param.get_name() == "fps")
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
        //                 this->usb_cam_->usb_cam_params_.fps = param.as_int();
        //                 result.successful = true;
        //             }
        //         }
        //     }
        // }
        // return result;
    }

    std::unique_ptr<UsbCam> UsbCamNode::init_usb_cam()
    {
        param_map_ = 
        {
            {"image_width", 0},
            {"image_height", 1},
            {"fps", 2}
        };

        this->declare_parameter("camera_id", 0);
        this->declare_parameter("frame_id", "usb_camera_link");
        this->declare_parameter("image_width", 480);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("fps", 30);

        this->declare_parameter<bool>("show_img", false);

        usb_cam_params_.camera_id = this->get_parameter("camera_id").as_int();
        usb_cam_params_.frame_id = this->get_parameter("frame_id").as_string();
        usb_cam_params_.image_width = this->get_parameter("image_width").as_int();
        usb_cam_params_.image_height = this->get_parameter("image_height").as_int();
        usb_cam_params_.fps = this->get_parameter("fps").as_int();

        return std::make_unique<UsbCam>(usb_cam_params_);
    }
} //namespace camera_driver

// int main(int argc, char** argv)
// {
//     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
//     rclcpp::init(argc, argv);
//     rclcpp::executors::SingleThreadedExecutor exec;
//     const rclcpp::NodeOptions options;
//     auto usb_cam_node = std::make_shared<camera_driver::UsbCamNode>(options);
//     exec.add_node(usb_cam_node);
//     exec.spin();
//     rclcpp::shutdown();

//     return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::UsbCamNode)