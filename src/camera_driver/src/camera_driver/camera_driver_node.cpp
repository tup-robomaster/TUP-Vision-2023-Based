#include "../../include/camera_driver/camera_driver_node.hpp"    

namespace camera_driver
{
    // template<class T>
    // CameraBaseNode<T>::~CameraBaseNode()
    // {
    //     if(using_shared_memory_)
    //     {
    //         if(!destorySharedMemory(shared_memory_param_))
    //             RCLCPP_ERROR(this->get_logger(), "Destory shared memory failed...");
    //     }
    // }

    // template<class T>
    // CameraBaseNode<T>::CameraBaseNode(string node_name, const rclcpp::NodeOptions& options)
    // : Node(node_name, options)
    // {
    //     RCLCPP_WARN(this->get_logger(), "Camera driver node...");
    //     try
    //     {   // Camera params initialize.
    //         cam_driver_ = init_cam_driver();
    //     }
    //     catch(const std::exception& e)
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "Error while initializing camera: %s", e.what());
    //     }

    //     this->declare_parameter<bool>("save_video", false);
    //     save_video_ = this->get_parameter("save_video").as_bool();
    //     if(save_video_)
    //     {   // Video save.
    //         videoRecorder(video_record_param_);
    //         RCLCPP_INFO(this->get_logger(), "Saving video...");
    //     }
    //     else
    //         RCLCPP_INFO(this->get_logger(), "No save video...");

    //     //QoS    
    //     rclcpp::QoS qos(0);
    //     qos.keep_last(1);
    //     qos.lifespan(8ms);
    //     qos.deadline();
    //     qos.best_effort();
    //     qos.reliable();
    //     qos.durability();
    //     qos.transient_local();
    //     qos.durability_volatile();
    
    //     // Camera type.
    //     this->declare_parameter<int>("camera_type", DaHeng);
    //     int camera_type = this->get_parameter("camera_type").as_int();

    //     // Subscriptions transport type.
    //     string transport_type = "raw";
    
    //     image_size_ = image_info_.image_size_map[camera_type];
    //     string camera_topic = image_info_.camera_topic_map[camera_type];

    //     // Open camera.
    //     if(!cam_driver_->open())
    //         RCLCPP_ERROR(this->get_logger(), "Open failed!");

    //     // Use shared memory.
    //     this->declare_parameter("using_shared_memory", false);
    //     using_shared_memory_ = this->get_parameter("using_shared_memory").as_bool();
    //     if(using_shared_memory_)
    //     {
    //         try
    //         {
    //             if(!setSharedMemory(shared_memory_param_, 5, image_size_.width, image_size_.height))
    //                 RCLCPP_ERROR(this->get_logger(), "Shared memory init failed...");
    //         }
    //         catch(const std::exception& e)
    //         {
    //             RCLCPP_FATAL(this->get_logger(), "Fatal while initializing shared memory...");
    //         }

    //         // 内存写入线程
    //         memory_write_thread_ = std::thread(&CameraBaseNode::image_callback, this);        
    //         RCLCPP_INFO(this->get_logger(), "Using shared memory...");
    //     }
    //     else
    //     {
    //         // Create img publisher.
    //         this->image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(camera_topic, qos);
            
    //         timer_ = this->create_wall_timer(1ms, std::bind(&CameraBaseNode::image_callback, this));
    //         RCLCPP_INFO(this->get_logger(), "Using image callback func...");
    //     }
    // }

    // template<class T>
    // void CameraBaseNode<T>::image_callback()
    // {
    //     if(using_shared_memory_)
    //     {
    //         while(1)
    //         {
    //             cv::Mat frame;
    //             if(!cam_driver_->get_frame(frame))
    //             {
    //                 RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
    //                 // Reopen camera.
    //                 if(!cam_driver_->open())
    //                     RCLCPP_ERROR(this->get_logger(), "Open failed!");
    //                 sleep(1);
    //                 return;
    //             }

    //             // rclcpp::Time now = this->get_clock()->now();
    //             // RCLCPP_WARN(this->get_logger(), "img_pub:%.4fs", now.nanoseconds() / 1e9);

    //             if(!frame.empty())
    //                 memcpy(shared_memory_param_.shared_memory_ptr, frame.data, this->image_size_.width * this->image_size_.height * 3);
    //             else
    //                 RCLCPP_ERROR(this->get_logger(), "Frame is empty...");

    //             save_video_ = this->get_parameter("save_video").as_bool();
    //             if(save_video_)
    //             {   // Video recorder.
    //                 videoRecorder(video_record_param_, &frame);
    //             }

    //             bool show_img = this->get_parameter("show_img").as_bool();
    //             if(show_img)
    //             {
    //                 cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
    //                 cv::imshow("frame", frame);
    //                 cv::waitKey(1);
    //             }
    //         }
    //     }
    //     else
    //     {
    //         // cv::Mat frame;
    //         if(!cam_driver_->get_frame(frame_))
    //         {
    //             RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
    //             // Reopen camera.
    //             if(!cam_driver_->open())
    //                 RCLCPP_ERROR(this->get_logger(), "Open failed!");
    //             sleep(1);
    //             return;
    //         }

    //         // Start!
    //         rclcpp::Time now = this->get_clock()->now();
    //         sensor_msgs::msg::Image::UniquePtr msg = convert_frame_to_msg(frame_);
    //         // sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header_, "bgr8", frame_).toImageMsg();

    //         msg->header.stamp = now;
    //         // rclcpp::Time time = msg->header.stamp;
    //         image_pub_->publish(std::move(msg));
    //         // RCLCPP_WARN(this->get_logger(), "img_pub_timestamp:%.4fs", now.nanoseconds() / 1e9);

    //         // save_video_ = this->get_parameter("save_video").as_bool();
    //         // if(save_video_)
    //         // {   // Video recorder.
    //         //     videoRecorder(video_record_param_, &frame);
    //         // }
            
    //         // bool show_img = this->get_parameter("show_img").as_bool();
    //         // if(show_img)
    //         // {
    //         //     cv::namedWindow("frame", cv::WINDOW_AUTOSIZE);
    //         //     cv::imshow("frame", frame);
    //         //     cv::waitKey(1);
    //         // }
    //     }
    // }

    // template<class T>
    // std::unique_ptr<sensor_msgs::msg::Image> CameraBaseNode<T>::convert_frame_to_msg(cv::Mat frame)
    // {
    //     std_msgs::msg::Header header;
    //     sensor_msgs::msg::Image ros_image;

    //     if(frame.size().width != this->image_size_.width || frame.size().height != this->image_size_.height)
    //     {
    //         cv::resize(frame, frame, cv::Size(this->image_size_.width, this->image_size_.height));
    //         RCLCPP_WARN(this->get_logger(), "Resize frame: width:%d height:%d", this->image_size_.width, this->image_size_.height);
    //     }

    //     ros_image.header.frame_id = this->frame_id_;
    //     ros_image.header.stamp = this->get_clock()->now();
    //     ros_image.width = this->image_size_.width;
    //     ros_image.height = this->image_size_.height;
    //     ros_image.encoding = "bgr8";
        
    //     ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);  
    //     ros_image.is_bigendian = false;
    //     ros_image.data.assign(frame.datastart, frame.dataend);

    //     auto msg_ptr = std::make_unique<sensor_msgs::msg::Image>(ros_image);      
    //     return msg_ptr;
    // }

    // template<class T>
    // std::unique_ptr<T> CameraBaseNode<T>::init_cam_driver()
    // {
    //     param_map_ = 
    //     {
    //         {"exposure_time", 0},
    //         {"exposure_gain", 1},
    //         {"balance_b", 2},
    //         {"balance_g", 3},
    //         {"balance_r", 4}
    //     };

    //     this->declare_parameter("cam_id", 1);
    //     this->declare_parameter("image_width", 1280);
    //     this->declare_parameter("image_height", 1024);
    //     this->declare_parameter("width_scale", 1);
    //     this->declare_parameter("height_scale", 1);
    //     this->declare_parameter("exposure_time", 6000);
    //     this->declare_parameter("exposure_gain", 14);
    //     this->declare_parameter("exposure_gain_b", 0);
    //     this->declare_parameter("exposure_gain_g", 0);
    //     this->declare_parameter("exposure_gain_r", 0);
    //     this->declare_parameter("auto_balance", false);
    //     this->declare_parameter("balance_b", 1.56);
    //     this->declare_parameter("balance_g", 1.0); 
    //     this->declare_parameter("balance_r", 1.548);
    //     this->declare_parameter("show_img", false);
    //     this->declare_parameter("using_video", false);
    //     this->declare_parameter("fps", 30);
    //     this->declare_parameter("video_path", "\0");

    //     camera_params_.cam_id = this->get_parameter("cam_id").as_int();
    //     camera_params_.image_width = this->get_parameter("image_width").as_int();
    //     camera_params_.image_height = this->get_parameter("image_height").as_int();
    //     camera_params_.width_scale = this->get_parameter("width_scale").as_int();
    //     camera_params_.height_scale = this->get_parameter("height_scale").as_int();
    //     camera_params_.exposure_time = this->get_parameter("exposure_time").as_int();
    //     camera_params_.exposure_gain = this->get_parameter("exposure_gain").as_int();
    //     camera_params_.exposure_gain_b = this->get_parameter("exposure_gain_b").as_int();
    //     camera_params_.exposure_gain_g = this->get_parameter("exposure_gain_g").as_int();
    //     camera_params_.exposure_gain_r = this->get_parameter("exposure_gain_r").as_int();
    //     camera_params_.auto_balance = this->get_parameter("auto_balance").as_bool();
    //     camera_params_.balance_b = this->get_parameter("balance_b").as_double();
    //     camera_params_.balance_g = this->get_parameter("balance_g").as_double();
    //     camera_params_.balance_r = this->get_parameter("balance_r").as_double();
    //     camera_params_.using_video = this->get_parameter("using_video").as_bool();
    //     camera_params_.fps = this->get_parameter("fps").as_int();
    //     camera_params_.video_path = this->get_parameter("video_path").as_string();

    //     return std::make_unique<T>(camera_params_);
    // }

} //namespace camera_driver