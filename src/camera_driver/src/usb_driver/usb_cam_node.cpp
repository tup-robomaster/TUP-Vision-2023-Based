/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:53
 * @LastEditTime: 2023-06-02 22:54:12
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/usb_driver/usb_cam_node.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;
namespace camera_driver
{
    UsbCamNode::UsbCamNode(const rclcpp::NodeOptions& option)
    : Node("usb_driver", option), is_filpped_(false)
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
        
        rclcpp::QoS qos(0);
        qos.keep_last(1);
        qos.reliable();
        qos.durability_volatile();
        // qos.durability();
        // qos.best_effort();
        // qos.transient_local();

        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 5;

        image_msg_pub_ = this->create_publisher<sensor_msgs::msg::Image>("usb_img_armor_node", qos);
        
        // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
        // camera_info_pub = image_transport::create_camera_publisher(this, "image");

        // auto camera_calibr_file = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
        // cam_info_manager->loadCameraInfo(camera_calibr_file);
        
        this->declare_parameter<bool>("save_video", false);
        save_video_ = this->get_parameter("save_video").as_bool();
        
        this->declare_parameter<bool>("using_video", true);
        using_video_ = this->get_parameter("using_video").as_bool();

        // sleep(10);
        if(using_video_)
        {
            string pkg_share_pth = get_package_share_directory("camera_driver");
            this->declare_parameter<std::string>("video_path", " ");
            video_path_ = pkg_share_pth + this->get_parameter("video_path").as_string();
            RCLCPP_WARN_ONCE(this->get_logger(), "Video path:%s", video_path_.c_str());

            this->declare_parameter<bool>("using_ros2bag", false);
            using_ros2bag_ = this->get_parameter("using_ros2bag").as_bool();
            if (using_ros2bag_)
            {
                rosbag2_storage::StorageOptions storage_options({video_path_, "sqlite3"});
                rosbag2_cpp::ConverterOptions converter_options(
                    {
                        rmw_get_serialization_format(),
                        rmw_get_serialization_format()      
                    }
                );
                reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
                reader_->open(storage_options, converter_options);
            }
            else
            {
                cap_.open(video_path_);
                if(!cap_.isOpened())
                {
                    RCLCPP_ERROR(this->get_logger(), "Open camera failed!");
                }
            }
        }
        else
        {
            cap_.open(usb_cam_params_.camera_id);
            if(cap_.isOpened())
            {
                RCLCPP_INFO(this->get_logger(), "Open camera success!");
            }
        }

        cap_.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_height);

        // last_time_ = this->get_clock()->now();
        img_pub_timer_ = this->create_wall_timer(1ms, std::bind(&UsbCamNode::image_callback, this));

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
        
        if(save_video_)
        {   // Video save.
            RCLCPP_INFO(this->get_logger(), "Saving video...");
            time_t tmpcal_ptr;
            tm *tmp_ptr = nullptr;
            tmpcal_ptr = time(nullptr);
            tmp_ptr = localtime(&tmpcal_ptr);
            char now[64];
            strftime(now, 64, "%Y-%m-%d_%H_%M_%S", tmp_ptr);  // 以时间为名字
            std::string now_string(now);
            string pkg_path = get_package_share_directory("camera_driver");
            std::string path = pkg_path + save_path_ + now_string;
            RCLCPP_WARN_ONCE(this->get_logger(), "Save path:%s", path.c_str());

            writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
            rosbag2_storage::StorageOptions storage_options({path, "sqlite3"});
            rosbag2_cpp::ConverterOptions converter_options({
                rmw_get_serialization_format(),
                rmw_get_serialization_format()
            });
            writer_->open(storage_options, converter_options);
            writer_->create_topic({
                "usb_img",
                "sensor_msgs::msg::Image",
                rmw_get_serialization_format(),
                ""
            });
        }
        else
            RCLCPP_WARN_ONCE(this->get_logger(), "Not save video...");
    }

    UsbCamNode::~UsbCamNode()
    {
    }

    std::shared_ptr<sensor_msgs::msg::Image> UsbCamNode::convert_frame_to_message(cv::Mat &frame)
    {
        std_msgs::msg::Header header;
        sensor_msgs::msg::Image ros_image;

        // if(frame.rows != usb_cam_params_.image_width || frame.cols != usb_cam_params_.image_height)
        // { 
        //     cv::resize(frame, frame, cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height));
        //     RCLCPP_INFO(this->get_logger(), "Resize frame...");
        // }

        ros_image.header.frame_id = "usb_camera_link";
        ros_image.header.stamp = this->get_clock()->now();
        ros_image.height = frame.rows;
        ros_image.width = frame.cols;
        ros_image.encoding = "bgr8";
        
        ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        ros_image.is_bigendian = false;
        ros_image.data.assign(frame.datastart, frame.dataend);

        // RCLCPP_INFO(this->get_logger(), "Copy frame...");
        // ros_image.is_bigendian = (std::endian::native == std::endian::big);
        // ros_image.step = frame.cols * frame.elemSize();
        // size_t size = ros_image.step * frame.rows;
        // ros_image.data.resize(size);
        // RCLCPP_INFO(this->get_logger(), "resize ros frame...");
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
        return msg_ptr;
    }

    void UsbCamNode::image_callback()
    {
        sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>();
        if (using_ros2bag_)
        {
            if (reader_->has_next())
            {
                // sensor_msgs::msg::Image image_msg;
                auto serializer = rclcpp::Serialization<sensor_msgs::msg::Image>();
                std::shared_ptr<rosbag2_storage::SerializedBagMessage> image_serialized_bag_msg = reader_->read_next();
                auto serialized_msg = rclcpp::SerializedMessage(*image_serialized_bag_msg->serialized_data);
                serializer.deserialize_message(&serialized_msg, &image_msg_);
                // frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
                msg = std::make_unique<sensor_msgs::msg::Image>(image_msg_);
            }
            else
            {
                return;
            }
        }
        else
        {
            cap_ >> frame_;
            auto now = this->get_clock()->now();
            auto dt = (now.nanoseconds() - last_time_.nanoseconds()) / 1e9;
            if(!frame_.empty())
            {
                last_time_ = now;
                
                // if(frame_.rows != usb_cam_params_.image_width || frame_.cols != usb_cam_params_.image_height)
                // { 
                //     cv::resize(frame_, frame_, cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height));
                //     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Resize frame...");
                // }
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
                // image_msg->header.stamp = timestamp;
                // image_msg->header.frame_id = frame_id;
                // camera_info_msg->header.stamp = timestamp;
                // camera_info_msg->header.frame_id = frame_id;
                msg->width = frame_.cols;
                msg->height = frame_.rows;
                msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
                msg->is_bigendian = false;
                msg->data.assign(frame_.datastart, frame_.dataend);
                save_video_ = this->get_parameter("save_video").as_bool();
                if (save_video_)
                {   // Video recorder.
                    ++frame_cnt_;
                    if (frame_cnt_ % 50 == 0)
                    {
                        sensor_msgs::msg::Image image_msg = *msg;
                        auto serializer = rclcpp::Serialization<sensor_msgs::msg::Image>();
                        auto serialized_msg = rclcpp::SerializedMessage();
                        serializer.serialize_message(&image_msg, &serialized_msg);
                        auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
                        bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                            new rcutils_uint8_array_t,
                            [this](rcutils_uint8_array_t* msg)
                            {
                                if (rcutils_uint8_array_fini(msg) != RCUTILS_RET_OK)
                                {
                                    RCLCPP_ERROR(this->get_logger(), "RCUTILS_RET_INVALID_ARGUMENT OR RCUTILS_RET_ERROR");
                                }
                                delete msg;
                            }
                        );
                        *bag_msg->serialized_data = serialized_msg.release_rcl_serialized_message();
                        bag_msg->topic_name = "usb_img";
                        bag_msg->time_stamp = now.nanoseconds();
                        writer_->write(bag_msg);
                    }
                }
            }
            else
            {
                return;
            }
        }

        rclcpp::Time now = this->get_clock()->now();
        msg->header.frame_id = usb_cam_params_.frame_id;
        msg->header.stamp = now;
        msg->encoding = "bgr8";
        
        // camera_info_pub.publish(image_msg, camera_info_msg);
        cout << "pub2armor..." << endl;
        image_msg_pub_->publish(std::move(msg));

        bool show_img = this->get_parameter("show_img").as_bool();
        if(show_img)
        {
            cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
            cv::imshow("raw_image", frame_);
            cv::waitKey(1);
        }

        // if(using_video_)
        //     usleep(2000);
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
        this->declare_parameter("image_width", 640);
        this->declare_parameter("image_height", 480);
        this->declare_parameter("fps", 30);
        this->declare_parameter<bool>("show_img", false);
        usb_cam_params_.camera_id = this->get_parameter("camera_id").as_int();
        usb_cam_params_.frame_id = this->get_parameter("frame_id").as_string();
        usb_cam_params_.image_width = this->get_parameter("image_width").as_int();
        usb_cam_params_.image_height = this->get_parameter("image_height").as_int();
        usb_cam_params_.fps = this->get_parameter("fps").as_int();

        this->declare_parameter<string>("save_path", "/recorder/video/");
        save_path_ = this->get_parameter("save_path").as_string();

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