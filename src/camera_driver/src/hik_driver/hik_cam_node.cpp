/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 14:30:38
 * @LastEditTime: 2022-11-19 16:41:04
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/hik_driver/hik_cam_node.cpp
 */
#include "../../include/hik_driver/hik_cam_node.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{
    HikCamNode::HikCamNode(const rclcpp::NodeOptions &options)
    : Node("hik_driver", options)
    {
        RCLCPP_WARN(this->get_logger(), "Camera driver node...");

        // camera params initialize 
        try
        {
            hik_cam = init_hik_cam();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        // create img publisher
        this->image_pub = this->create_publisher<sensor_msgs::msg::Image>("hik_img", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile());
        
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
        
        timer = this->create_wall_timer(1ms, std::bind(&HikCamNode::image_callback, this));

        bool debug_;
        this->declare_parameter<bool>("debug", true);
        this->get_parameter("debug", debug_);
        if(debug_)
        {
            //动态调参回调
            callback_handle_ = this->add_on_set_parameters_callback(std::bind(&HikCamNode::paramsCallback, this, std::placeholders::_1));
        }
    }

    std::unique_ptr<HikCamera> HikCamNode::init_hik_cam()
    {
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

        return std::make_unique<HikCamera>(hik_cam_params_);
    }

    rcl_interfaces::msg::SetParametersResult HikCamNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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

                        this->hik_cam_params_.exposure_time = param.as_int();
                        this->hik_cam->set_exposure_time(this->hik_cam_params_.exposure_time);
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
                        this->hik_cam_params_.exposure_gain = param.as_int();
                        this->hik_cam->set_gain(3, this->hik_cam_params_.exposure_gain);
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
                        this->hik_cam_params_.balance_b = param.as_double();
                        this->hik_cam->set_balance(0, this->hik_cam_params_.balance_b);
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
                        this->hik_cam_params_.balance_g = param.as_double();
                        this->hik_cam->set_balance(1, this->hik_cam_params_.balance_g);
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
                        this->hik_cam_params_.balance_r = param.as_double();
                        this->hik_cam->set_balance(2, this->hik_cam_params_.balance_r);
                        result.successful = true;
                    }
                }
            }
        }
        
        return result;
    }

    std::unique_ptr<sensor_msgs::msg::Image> HikCamNode::convert_frame_to_msg(cv::Mat frame)
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

    void HikCamNode::image_callback()
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

int main(int argc, char** argv)
{
    // rclcpp::init(argc, argv);
    // const rclcpp::NodeOptions options;
    // auto cam_node = std::make_shared<camera_driver::HikCamNode>(options);
    // rclcpp::spin(cam_node);
    // rclcpp::shutdown();

    // printf("2\n");
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;

    const rclcpp::NodeOptions options;

    auto HikCamNode = std::make_shared<camera_driver::HikCamNode>(options);

    exec.add_node(HikCamNode);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::HikCamNode)

