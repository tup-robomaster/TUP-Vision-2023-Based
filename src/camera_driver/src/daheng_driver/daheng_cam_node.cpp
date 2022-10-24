/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-09 14:25:39
 * @LastEditTime: 2022-10-23 22:57:15
 * @FilePath: /TUP-Vision-2023/src/camera_driver/src/daheng_driver/daheng_cam_node.cpp
 */
#include "../../include/daheng_driver/daheng_cam_node.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{
    daheng_cam_node::daheng_cam_node(const rclcpp::NodeOptions &options)
    : Node("daheng_driver", options)
    {
        // camera params initialize 
        daheng_cam = init_daheng_cam();

        // create img publisher
        this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("daheng_img", 1);
        
        // this->declare_parameter("image_width", 1280);
        // this->image_height = this->declare_parameter("image_height", 1024);
        // this->daheng_cam_id = this->declare_parameter("daheng_cam_id", 0);
        this->frame_id = this->declare_parameter("frame_id", "daheng_cam");

        this->image_width = this->get_parameter("image_width").as_int();
        this->image_height = this->get_parameter("image_height").as_int();
        this->frame_id = this->get_parameter("frame_id").as_string();

        // acquisition system clock
        last_frame = std::chrono::steady_clock::now();

        // open daheng_camera
        if(!daheng_cam->open())
        {
            RCLCPP_WARN(this->get_logger(), "Camera open failed!");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Camera open success!");
        }
        
        timer = this->create_wall_timer(1ms, std::bind(&daheng_cam_node::image_callback, this));
    }

    std::unique_ptr<daheng_camera> daheng_cam_node::init_daheng_cam()
    {
        daheng_cam_param daheng_cam_param_;
        
        this->declare_parameter("daheng_cam_id", 1);
        this->declare_parameter("image_width", 1080);
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
        daheng_cam_param_.width_scale = this->declare_parameter("width_scale", 1);
        daheng_cam_param_.height_scale = this->declare_parameter("height_scale", 1);
        daheng_cam_param_.exposure_time = this->get_parameter("exposure_time").as_int();
        daheng_cam_param_.exposure_gain = this->get_parameter("exposure_gain").as_int();
        daheng_cam_param_.exposure_gain_b = this->get_parameter("exposure_gain_b").as_int();
        daheng_cam_param_.exposure_gain_g = this->get_parameter("exposure_gain_g").as_int();
        daheng_cam_param_.exposure_gain_r = this->get_parameter("exposure_gain_r").as_int();
        daheng_cam_param_.auto_balance = this->get_parameter("auto_balance").as_bool();
        daheng_cam_param_.balance_b = this->get_parameter("balance_b").as_double();
        daheng_cam_param_.balance_g = this->get_parameter("balance_g").as_double();
        daheng_cam_param_.balance_r = this->get_parameter("balance_r").as_double();

        return std::make_unique<daheng_camera>(daheng_cam_param_);
    }


    std::unique_ptr<sensor_msgs::msg::Image> daheng_cam_node::convert_frame_to_msg(cv::Mat frame)
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

    void daheng_cam_node::image_callback()
    {
        if(!daheng_cam->get_frame(frame))
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

        cv::namedWindow("daheng_cam_frame", cv::WINDOW_AUTOSIZE);
        cv::imshow("daheng_cam_frame", frame);
        cv::waitKey(1);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<camera_driver::daheng_cam_node>());
    rclcpp::shutdown();
    
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::daheng_cam_node)

