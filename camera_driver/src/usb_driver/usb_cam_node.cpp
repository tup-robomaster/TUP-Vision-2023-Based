/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-28 17:12:53
 * @LastEditTime: 2022-09-29 19:56:19
 * @FilePath: /tup_2023/src/camera_driver/src/usb_driver/usb_cam_node.cpp
 */
#include "../../include/usb_driver/usb_cam_node.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{
    usb_cam_node::usb_cam_node(const rclcpp::NodeOptions& option)
    : Node("camera_driver", option), is_filpped(false)
    {
        frame_pub = this->create_publisher<sensor_msgs::msg::Image>("image_data", 1);
        frame_id = this->declare_parameter("frame_id", "camera");
        image_width = this->declare_parameter("image_width", 480);
        image_height = this->declare_parameter("image_height", 480);
        fps = this->declare_parameter("fps", 30);

        camera_id = this->declare_parameter("camera_id", 0);


        // rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
        // camera_info_pub = image_transport::create_camera_publisher(this, "image");

        // auto camera_calibr_file = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
        // cam_info_manager->loadCameraInfo(camera_calibr_file);

        cap.open(camera_id);
        if(cap.isOpened())
        {
            RCLCPP_INFO(this->get_logger(), "open camera success!");
        }

        cap.set(cv::CAP_PROP_FRAME_WIDTH, image_width);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, image_height);

        last_frame = std::chrono::steady_clock::now();

        timer = this->create_wall_timer(1ms, std::bind(&usb_cam_node::image_callback, this));
    }

    std::shared_ptr<sensor_msgs::msg::Image> usb_cam_node::convert_frame_to_message(cv::Mat &frame)
    {
        std_msgs::msg::Header header;

        sensor_msgs::msg::Image ros_image;

        // cv::namedWindow("raw", cv::WINDOW_AUTOSIZE);
        // cv::imshow("raw", frame);
        // cv::waitKey(0);

        if(frame.rows != image_width || frame.cols != image_height)
        { 
            cv::resize(frame, frame, cv::Size(image_width, image_height));
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
        
        if(!frame.empty() && 
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame).count() > 1 / fps * 1000)
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
            msg->header.frame_id = frame_id;
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

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::usb_cam_node)