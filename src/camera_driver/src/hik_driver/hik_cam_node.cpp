/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 14:30:38
 * @LastEditTime: 2022-09-30 14:03:08
 * @FilePath: /tup_2023/src/camera_driver/src/hik_driver/hik_cam_node.cpp
 */
#include "../../include/hik_driver/hik_cam_node.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{
    hik_cam_node::hik_cam_node(const rclcpp::NodeOptions &options)
    : Node("hik_camera", options)
    {
        // camera params initialize 
        try
        {
            hik_cam = init_hik_cam();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        RCLCPP_INFO(this->get_logger(), "1...");
        // create img publisher
        this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("hik_img", 1);
        
        this->image_width = this->get_parameter("image_width").as_int();
        this->image_height = this->get_parameter("image_height").as_int();
        this->hik_cam_id = this->get_parameter("hik_cam_id").as_int();

        // this->declare_parameter("image_width", 1440);
        // this->declare_parameter("image_height", 1080);
        // this->declare_parameter("hik_cam_id", 0);

        this->declare_parameter("frame_id", "hik_camera");
        this->frame_id = this->get_parameter("frame_id").as_string();

        // printf("%d %d %d \n", this->image_width, this->image_height, this->hik_cam_id);
        
        // acquisition system clock
        last_frame = std::chrono::steady_clock::now();

        // open hik_camera
        if(!hik_cam->open())
        {
            RCLCPP_INFO(this->get_logger(), "Camera open failed!");
        }
        
        timer = this->create_wall_timer(1ms, std::bind(&hik_cam_node::image_callback, this));
    }

    std::unique_ptr<hik_camera> hik_cam_node::init_hik_cam()
    {
        hik_cam_params hik_cam_params_;
        
        this->declare_parameter("hik_cam_id", 0);
        this->declare_parameter("image_width", 1440);
        this->declare_parameter("image_height", 1080);
        this->declare_parameter("exposure_time", 3000);
        this->declare_parameter("exposure_gain", 7);
        this->declare_parameter("exposure_gain_b", 0);
        this->declare_parameter("exposure_gain_g", 0);
        this->declare_parameter("exposure_gain_r", 0);
        this->declare_parameter("auto_balance", false);
        this->declare_parameter("balance_b", 1690);
        this->declare_parameter("balance_g", 1024); 
        this->declare_parameter("balance_r", 2022);

        hik_cam_params_.hik_cam_id = this->get_parameter("hik_cam_id").as_int();
        hik_cam_params_.image_width = this->get_parameter("image_width").as_int();
        hik_cam_params_.image_height = this->get_parameter("image_height").as_int();
        hik_cam_params_.exposure_time = this->get_parameter("exposure_time").as_int();
        hik_cam_params_.exposure_gain = this->get_parameter("exposure_gain").as_int();
        hik_cam_params_.exposure_gain_b = this->get_parameter("exposure_gain_b").as_int();
        hik_cam_params_.exposure_gain_g = this->get_parameter("exposure_gain_g").as_int();
        hik_cam_params_.exposure_gain_r = this->get_parameter("exposure_gain_r").as_int();
        hik_cam_params_.auto_balance = this->get_parameter("auto_balance").as_bool();
        hik_cam_params_.balance_b = this->get_parameter("balance_b").as_int();
        hik_cam_params_.balance_g = this->get_parameter("balance_g").as_int();
        hik_cam_params_.balance_r = this->get_parameter("balance_r").as_int();

        // RCLCPP_INFO(this->get_logger(), "1...");

        return std::make_unique<hik_camera>(hik_cam_params_);
    }

    std::unique_ptr<sensor_msgs::msg::Image> hik_cam_node::convert_frame_to_msg(cv::Mat frame)
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

    void hik_cam_node::image_callback()
    {
        if(!hik_cam->get_frame(frame))
        {
            RCLCPP_ERROR(this->get_logger(), "Get frame failed!");
        }

        auto now = std::chrono::steady_clock::now();

        if(!frame.empty())
        {
            this->last_frame = now;

            rclcpp::Time timestamp = this->get_clock()->now();

            sensor_msgs::msg::Image::UniquePtr msg = convert_frame_to_msg(frame);

            image_pub->publish(std::move(msg));
        }

        // cv::namedWindow("hik_cam_frame", cv::WINDOW_AUTOSIZE);
        // cv::imshow("hik_cam_frame", frame);
        // cv::waitKey(1);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::hik_cam_node)

