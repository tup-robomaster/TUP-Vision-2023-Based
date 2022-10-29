/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2022-10-28 19:48:56
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/serialport/serialport_node.cpp
 */
#include "../../include/serialport/serialport_node.hpp"

namespace serialport
{
    serial_driver::serial_driver(const rclcpp::NodeOptions& options)
    : Node("serial_driver", options), device_name_("ttyACM0"), baud_(115200)
    {
        RCLCPP_INFO(this->get_logger(), "Serialport node...");
        
        serial_port_ = init_serial_port();

        // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_state", rclcpp::QoS(rclcpp::KeepLast(10)));
        
        
        // if(serial_port->debug_without_port())
        // {
        //     serial_port->open(device_name, baud);
        //     if(!serial_port->is_open)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Serial open failed!");
        //     }
        //     else
        //     {
        //         run();
        //     }

        // }

        //world
        this->declare_parameter<std::string>("base_frame", "world");
        this->get_parameter("base_frame", this->base_frame_);

        //declare and acquire 'imu_frame' parameter
        this->declare_parameter<std::string>("imu_frame", "gyro");
        this->get_parameter("imu_frame", this->base_frame_);

        //camera
        this->declare_parameter<std::string>("camera_frame", "camera");
        this->get_parameter("camera_frame", this->camera_frame_);

        //static transform 
        this->declare_parameter<std::vector<double>>("static_transform", {0, 0, 0});
        this->get_parameter("static_transform", static_transform_);

        //initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        gimbal_motion_sub_ = this->create_subscription<global_interface::msg::Target>(
            "armor_detector/armor_info", 
            rclcpp::SensorDataQoS(rclcpp::KeepLast(10)),
            std::bind(&serial_driver::send_data, this, std::placeholders::_1)
        );

        if(serial_port_->debug_without_port())
        {
            serial_port_->open(device_name_, baud_);
            if(!serial_port_->is_open)
            {
                RCLCPP_INFO(this->get_logger(), "Serial open failed!");
            }
            else
            {
                receive_thread_ = std::thread(&serial_driver::receive_data, this);
            }
        }
    }

    serial_driver::~serial_driver()
    {
        // if(buffer) delete buffer;
        // buffer = NULL;
    }

    void handle_imu_data(const std::shared_ptr<global_interface::msg::Imu>& msg)
    {
        
    }

    void serial_driver::run()
    {
        while(true)
        {
            receive_data();
        }
    }

    std::unique_ptr<serialport> serial_driver::init_serial_port()
    {
        std::string id = "483/5740/200";
        int baud = 115200;

        return std::make_unique<serialport>(id, baud);
    }    

    void serial_driver::receive_data()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data(sizeof(ReceivePacket));

        while (rclcpp::ok())
        {
            if(serial_port_->debug_without_port())
            {
                serial_port_->read(header);
                if(header.at(0) == 0xA5)
                {
                    serial_port_->read(data);
                    data.insert(data.begin(), header.at(0));

                    ReceivePacket packet = vec_to_packet(data);

                    bool crc_16_check = Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
                    if(crc_16_check)
                    {
                        sensor_msgs::msg::JointState joint_state;
                        joint_state.header.stamp = this->now();
                        joint_state.name.push_back("pitch_angle");
                        joint_state.name.push_back("yaw_angle");
                        joint_state.position.push_back(packet.joint_pitch);
                        joint_state.position.push_back(packet.joint_yaw);
                        joint_state_pub_->publish(joint_state);

                        // std::cout << "bullet_speed:" << packet.bullet_speed << std::endl;
                        RCLCPP_INFO(this->get_logger(), "bullet_speed:%f", packet.bullet_speed);

                        //tf2
                        rclcpp::Time now = this->get_clock()->now();
                        geometry_msgs::msg::TransformStamped transform_;
                        transform_.header.stamp = now;
                        transform_.header.frame_id = this->base_frame_;
                        transform_.child_frame_id = this->imu_frame_;

                        //TODO:add translation vector
                        transform_.transform.translation.x = 0;
                        transform_.transform.translation.y = 0;
                        transform_.transform.translation.z = 0;

                        //rotation
                        transform_.transform.rotation.w = packet.quat[0];
                        transform_.transform.rotation.x = packet.quat[1];
                        transform_.transform.rotation.y = packet.quat[2];
                        transform_.transform.rotation.z = packet.quat[3];
                        
                        //imu rotation
                        quat_ = {packet.quat[0], packet.quat[1], packet.quat[2], packet.quat[3]};
                        rmat_imu_ = quat_.toRotationMatrix();

                        //send the transformation
                        tf_broadcaster_->sendTransform(transform_);
                    }        
                    else
                    {
                        // std::cout << "Crc check error!" << std::endl;

                        RCLCPP_ERROR(this->get_logger(), "Crc check error!");
                    }   
                }
                else
                {
                    // std::cout << "Invalid header:" << header.at(0) << std::endl;
                    RCLCPP_ERROR(this->get_logger(), "Invalid header: %02X", header.at(0));
                }
            }
        }
    }

    void serial_driver::send_data(global_interface::msg::Target::SharedPtr target_info) 
    {
        global_interface::msg::Gimbal msg;
        
        Eigen::Vector3d aiming_point = {target_info->aiming_point.x, target_info->aiming_point.y, target_info->aiming_point.z};
        
        auto angle = coordsolver_.getAngle(aiming_point, rmat_imu_);
        
        if(serial_port_->debug_without_port())
        {
            SendPacket packet;
            packet.header = 0xA5;
            packet.pitch_angle = angle[0];
            packet.yaw_angle = angle[1];
            packet.dis = 0;
            packet.isFindTarget = false;
            packet.isSwitched = false;
            packet.isSpinning = false;
            packet.isMiddle = false;
            
            Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
            
            std::vector<uint8_t> data = packet_to_vec(packet);
            serial_port_->write((data));
            
            // if(!serial_port_->write((data)))
            // {
            //     RCLCPP_ERROR(this->get_logger(), "serial data sends failed!");
            // }
        }
    }

    void serial_driver::send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg) const
    {
        rclcpp::Time now;

        //send transform
        send_tf2_transforms(msg, imu_frame_, camera_frame_, now);

        //send transform
        send_tf2_transforms(msg, base_frame_, imu_frame_, now);

        geometry_msgs::msg::TransformStamped world_tf;

        world_tf.transform.translation.x = static_transform_[0];
        world_tf.transform.translation.y = static_transform_[1];
        world_tf.transform.translation.z = static_transform_[2];

        world_tf.transform.rotation.x = 0;
        world_tf.transform.rotation.y = 0;
        world_tf.transform.rotation.z = 0;
        world_tf.transform.rotation.w = 0;
        world_tf.header.stamp = now;

        tf_broadcaster_->sendTransform(world_tf);
    }

    void serial_driver::send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg,
        const std::string& header_frame_id, const std::string& child_frame_id) const
    {
        rclcpp::Time now;
        send_tf2_transforms(msg, header_frame_id, child_frame_id, now);
    }

    void serial_driver::send_tf2_transforms(const global_interface::msg::Target::SharedPtr msg,
        const std::string& header_frame_id,
        const std::string& child_frame_id,
        const rclcpp::Time& time) const
    {
        rclcpp::Time now;
        //camera->base_link transform
        geometry_msgs::msg::TransformStamped transform;

        transform.transform.translation.x = msg->aiming_point.x;
        transform.transform.translation.y = msg->aiming_point.y;
        transform.transform.translation.z = msg->aiming_point.z;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        transform.header.frame_id = header_frame_id;
        transform.child_frame_id = child_frame_id;
        transform.header.stamp = now;

        tf_broadcaster_->sendTransform(transform);
    }

    
} //serialport

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(serialport::serial_driver)