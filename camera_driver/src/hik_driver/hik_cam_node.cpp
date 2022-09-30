/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 14:30:38
 * @LastEditTime: 2022-09-30 10:05:12
 * @FilePath: /tup_2023/src/camera_driver/src/hik_driver/hik_cam_node.cpp
 */
#include "../../include/hik_driver/hik_cam_node.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{
    hik_cam_node::hik_cam_node(const rclcpp::NodeOptions &options)
    : Node("hik_camera", options)
    {
        printf("1\n");
        this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("hik_image_data", 1);
        
        this->image_width = this->declare_parameter("image_width", 1440);
        this->image_height = this->declare_parameter("image_height", 1080);
        this->hik_cam_id = this->declare_parameter("hik_cam_id", 0);
        this->frame_id = this->declare_parameter("frame_id", "camera");

        last_frame = std::chrono::steady_clock::now();
        printf("8\n");
        if(!hik_cam->open())
        {
            RCLCPP_INFO(this->get_logger(), "Camera open failed!");
        }
        printf("3\n");
        
        timer = this->create_wall_timer(1ms, std::bind(&hik_cam_node::image_callback, this));
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
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::hik_cam_node)

