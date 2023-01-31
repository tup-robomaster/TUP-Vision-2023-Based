/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2023-02-01 00:00:50
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/serialport_node.cpp
 */
#include "../include/serialport_node.hpp"

using namespace std::placeholders;
namespace serialport
{
    SerialPortNode::SerialPortNode(const rclcpp::NodeOptions& options)
    : Node("serial_port", options), device_name_("ttyACM0"), baud_(115200)
    {
        RCLCPP_WARN(this->get_logger(), "Serialport node...");
        try
        {
            serial_port_ = init_serial_port();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(10);
        qos.best_effort();
        qos.reliable();
        qos.durability();
        qos.durability_volatile();
        
        // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_state", rclcpp::QoS(rclcpp::KeepLast(10)));
        
        // if(serial_port->debug_without_port())
        // {
        //     serial_port->open(device_name, baud);
        //     if(!serial_port->is_open)
        //         RCLCPP_INFO(this->get_logger(), "Serial open failed!");
        //     else
        //         run();
        // }

        // world
        // this->declare_parameter<std::string>("base_frame", "world");
        // this->get_parameter("base_frame", this->base_frame_);

        // Declare and acquire 'imu_frame' parameter.
        // this->declare_parameter<std::string>("imu_frame", "gyro");
        // this->get_parameter("imu_frame", this->base_frame_);

        // camera
        // this->declare_parameter<std::string>("camera_frame", "camera");
        // this->get_parameter("camera_frame", this->camera_frame_);

        // Static transformation. 
        // this->declare_parameter<std::vector<double>>("static_transform", {0, 0, 0});
        // this->get_parameter("static_transform", static_transform_);
        
        // tf2
        // target_frame_ = this->declare_parameter<std::string>("target_frame", "world");

        // std::chrono::duration<int> buffer_timeout(1);

        // tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        // auto time_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        //     this->get_node_base_interface(),
        //     this->get_node_timers_interface()
        // );

        // tf2_buffer_->setCreateTimerInterface(time_interface);
        // tf2_listener_ = 
        //     std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // gimbal_info_sub_.subscriber(this, "/gimbal_info");
        // tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<GimbalMsg>>(
        //     gimbal_info_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
        //     this->get_node_clock_interface(), buffer_timeout);

        // Callback func.
        // tf2_filter_->registerCallback(&);

        // Initialize the transform broadcaster.
        // tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        if(!is_sentry_mode_)
        {
            autoaim_info_sub_ = this->create_subscription<GimbalMsg>(
                "/armor_processor/gimbal_info", 
                qos,
                std::bind(&SerialPortNode::send_armor_data, this, _1)
            );

            use_buff_mode_ = this->get_parameter("use_buff_mode").as_bool();
            if(use_buff_mode_)
            {
                buff_info_sub_ = this->create_subscription<GimbalMsg>(
                    "/buff_processor/gimbal_info",
                    qos,
                    std::bind(&SerialPortNode::send_buff_data, this, _1)
                );
            }
            // if(serial_port_->debug_without_port())
            // {
            //     serial_port_->open(device_name_, baud_);
            //     if(!serial_port_->is_open)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "Serial open failed!");
            //     }
            //     else
            //     {
            //         receive_thread_ = std::thread(&SerialPortNode::receive_data, this);
            //     }
            // }

            if(!debug_without_port_)
            {   // Use serial port.
                if(serial_port_->initSerialPort())
                {
                    imu_data_pub_ = this->create_publisher<ImuMsg>("/imu_msg", qos);
                    receive_thread_ = std::thread(&SerialPortNode::receive_data, this);
                }
            }
        }
        else
        {
            
        }
    }

    SerialPortNode::~SerialPortNode()
    {
        // if(buffer) delete buffer;
        // buffer = NULL;
    }

    // void SerialPortNode::handle_imu_data(const std::shared_ptr<global_interface::msg::Imu>& msg)
    // {
        
    // }

    // void SerialPortNode::run()
    // {
    //     receive_data();
    // }

    void SerialPortNode::receive_data()
    {
        // std::vector<uchar> header(1);
        // std::vector<uchar> data(sizeof(ReceivePacket));
        // while (rclcpp::ok())
        // {
        //     if(serial_port_->debug_without_port())
        //     {
        //         std::cout << 6 << std::endl;
        //         serial_port_->read(header);
        //         // std::cout << header.at(0) << std::endl;
        //         if(header.at(0) == 0xA5)
        //         {
        //             serial_port_->read(data);
        //             data.insert(data.begin(), header.at(0));

        //             ReceivePacket packet = vec_to_packet(data);

        //             bool crc_16_check = Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
        //             if(crc_16_check)
        //             {
        //                 sensor_msgs::msg::JointState joint_state;
        //                 joint_state.header.stamp = this->now();
        //                 joint_state.name.push_back("pitch_angle");
        //                 joint_state.name.push_back("yaw_angle");
        //                 joint_state.position.push_back(packet.joint_pitch);
        //                 joint_state.position.push_back(packet.joint_yaw);
        //                 joint_state_pub_->publish(joint_state);

        //                 // std::cout << "bullet_speed:" << packet.bullet_speed << std::endl;
        //                 RCLCPP_INFO(this->get_logger(), "bullet_speed:%f", packet.bullet_speed);

        //                 //tf2
        //                 rclcpp::Time now = this->get_clock()->now();
        //                 geometry_msgs::msg::TransformStamped transform_;
        //                 transform_.header.stamp = now;
        //                 transform_.header.frame_id = this->base_frame_;
        //                 transform_.child_frame_id = this->imu_frame_;

        //                 //TODO:add translation vector
        //                 transform_.transform.translation.x = 0;
        //                 transform_.transform.translation.y = 0;
        //                 transform_.transform.translation.z = 0;

        //                 //rotation
        //                 transform_.transform.rotation.w = packet.quat[0];
        //                 transform_.transform.rotation.x = packet.quat[1];
        //                 transform_.transform.rotation.y = packet.quat[2];
        //                 transform_.transform.rotation.z = packet.quat[3];
                        
        //                 //imu rotation
        //                 quat_ = {packet.quat[0], packet.quat[1], packet.quat[2], packet.quat[3]};
        //                 rmat_imu_ = quat_.toRotationMatrix();

        //                 //send the transformation
        //                 tf_broadcaster_->sendTransform(transform_);
        //             }        
        //             else
        //             {
        //                 // std::cout << "Crc check error!" << std::endl;

        //                 RCLCPP_ERROR(this->get_logger(), "Crc check error!");
        //             }   
        //         }
        //         else
        //         {
        //             // std::cout << "Invalid header:" << header.at(0) << std::endl;
        //             // RCLCPP_ERROR(this->get_logger(), "Invalid header: %02X", header.at(0));
        //         }
        //     }
        // }

        while(1)
        {
            // 若串口离线则跳过数据发送
            if (!serial_port_->serial_data_.is_initialized)
            {
                std::cout << "Offline..." << std::endl;
                usleep(5000);
                continue;
            }
            //数据读取不成功进行循环
            while (!serial_port_->get_Mode(bytes_num_))
                ;
            // auto time_cap = std::chrono::steady_clock::now();

            // auto imu_timestamp = serial.timestamp;
            // auto delta_t = (int)(std::chrono::duration<double,std::micro>(imu_timestamp - image_timestamp).count());
            // cout << "cam_imu data delta_t: " << delta_t << "us" <<  endl;
            // cout<<"Quat: "<<serial.quat[0]<<" "<<serial.quat[1]<<" "<<serial.quat[2]<<" "<<serial.quat[3]<<" "<<endl;
            // Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
            
            // FIXME:注意此处mode设置
            // int mode = serial_port_->serial_data_.mode;
            // float bullet_speed = serial_port_->bullet_speed_;
            
            // int mode = 2;
            // Eigen::Quaterniond quat = {serial_port_->quat[0], serial_port_->quat[1], serial_port_->quat[2], serial_port_->quat[3]};
            // Eigen::Vector3d acc = {serial_port_->acc[0], serial_port_->acc[1], serial_port_->acc[2]};
            // Eigen::Vector3d gyro = {serial_port_->gyro[0], serial_port_->gyro[1], serial_port_->gyro[2]};
            
            // tf2
            // geometry_msgs::msg::TransformStamped transform_;
            // transform_.header.stamp = this->get_clock()->now();
            // transform_.header.frame_id = this->base_frame_;
            // transform_.child_frame_id = this->imu_frame_;

            // TODO:Add translation vector.
            // transform_.transform.translation.x = 0;
            // transform_.transform.translation.y = 0;
            // transform_.transform.translation.z = 0;

            // Rotation.
            // transform_.transform.rotation.w = packet.quat[0];
            // transform_.transform.rotation.x = packet.quat[1];
            // transform_.transform.rotation.y = packet.quat[2];
            // transform_.transform.rotation.z = packet.quat[3];

            // transform_.transform.rotation.w = serial_port_->quat[0];
            // transform_.transform.rotation.x = serial_port_->quat[1];
            // transform_.transform.rotation.y = serial_port_->quat[2];
            // transform_.transform.rotation.z = serial_port_->quat[3];
            
            // imu rotation.
            // quat_ = {serial_port_->quat[0], serial_port_->quat[1], serial_port_->quat[2], serial_port_->quat[3]};
            // rmat_imu_ = quat_.toRotationMatrix();

            // Send the transformation.
            // tf_broadcaster_->sendTransform(transform_);
            
            // Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2, 1, 0);
            // std::cout << "Euler : " << vec[0] * 180.f / CV_PI << " " << vec[1] * 180.f / CV_PI << " " << vec[2] * 180.f / CV_PI << std::endl;
            // std::cout << "Transmitting..." << std::endl;
            
            // imu info pub.
            ImuMsg imu_info;
            imu_info.imu.header.frame_id = "imu_link";
            imu_info.imu.header.stamp = this->get_clock()->now();
            imu_info.bullet_speed = serial_port_->bullet_speed_;
            imu_info.imu.orientation.w = serial_port_->quat[0];
            imu_info.imu.orientation.x = serial_port_->quat[1];
            imu_info.imu.orientation.y = serial_port_->quat[2];
            imu_info.imu.orientation.z = serial_port_->quat[3];
            imu_info.imu.angular_velocity.x = serial_port_->gyro[0];
            imu_info.imu.angular_velocity.y = serial_port_->gyro[1];
            imu_info.imu.angular_velocity.z = serial_port_->gyro[2];
            imu_info.imu.linear_acceleration.x = serial_port_->acc[0];
            imu_info.imu.linear_acceleration.y = serial_port_->acc[1];
            imu_info.imu.linear_acceleration.z = serial_port_->acc[2];
            
            imu_data_pub_->publish(std::move(imu_info));
        }
    }

    void SerialPortNode::send_armor_data(GimbalMsg::SharedPtr target_info) 
    {
        // GimbalMsg msg;
        // Eigen::Vector3d aiming_point = {target_info->aiming_point.x, target_info->aiming_point.y, target_info->aiming_point.z};
        
        // auto angle = coordsolver_.getAngle(aiming_point, rmat_imu_);
        
        // if(serial_port_->debug_without_port())
        // {
        //     SendPacket packet;
        //     packet.header = 0xA5;
        //     packet.pitch_angle = angle[0];
        //     packet.yaw_angle = angle[1];
        //     packet.dis = 0;
        //     packet.isFindTarget = false;
        //     packet.isSwitched = false;
        //     packet.isSpinning = false;
        //     packet.isMiddle = false;
            
        //     Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
            
        //     std::vector<uint8_t> data = packet_to_vec(packet);
        //     serial_port_->write((data));
            
            //  if(!serial_port_->write((data)))
            // {
            //     RCLCPP_ERROR(this->get_logger(), "serial data sends failed!");
            // }
        // }
        
        if(!this->debug_without_port_)
        {
            if(serial_port_->mode == 1 || serial_port_->mode == 2)
            {
                RCLCPP_INFO(this->get_logger(), "Currently mode is %d", serial_port_->mode);
                // RCLCPP_INFO(this->get_logger(), "Send armor data...");
                VisionData transmition_data = {target_info->pitch, target_info->yaw, target_info->distance, target_info->is_switched, 1, target_info->is_spinning, 0};
                serial_port_->transformData(transmition_data);
                serial_port_->send();
            }
        }
        else
        {   // Debug without com.
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub autoaim msg...");
        }
    }

    void SerialPortNode::send_buff_data(GimbalMsg::SharedPtr target_info) 
    {
        if(!this->debug_without_port_)
        {
            if(serial_port_->mode == 3 || serial_port_->mode == 4)
            {
                RCLCPP_INFO(this->get_logger(), "Currently mode is %d", serial_port_->mode);
                // RCLCPP_INFO(this->get_logger(), "Send buff data...");
                VisionData transmition_data = {target_info->pitch, target_info->yaw, target_info->distance, target_info->is_switched, 1, target_info->is_spinning, 0};
                serial_port_->transformData(transmition_data);
                serial_port_->send();
            }
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub buff msg...");
        }
    }

    // void SerialPortNode::send_tf2_transforms(const TransformMsg::SharedPtr msg) 
    // {
    //     rclcpp::Time now = this->get_clock()->now();

    //     // Send transformation.
    //     send_tf2_transforms(msg, imu_frame_, camera_frame_, now);

    //     // Send transformation.
    //     send_tf2_transforms(msg, base_frame_, imu_frame_, now);

    //     geometry_msgs::msg::TransformStamped world_tf;

    //     world_tf.transform.translation.x = static_transform_[0];
    //     world_tf.transform.translation.y = static_transform_[1];
    //     world_tf.transform.translation.z = static_transform_[2];

    //     world_tf.header.stamp = now;
    //     world_tf.transform.rotation.x = 0;
    //     world_tf.transform.rotation.y = 0;
    //     world_tf.transform.rotation.z = 0;
    //     world_tf.transform.rotation.w = 0;

    //     tf_broadcaster_->sendTransform(world_tf);
    // }

    // void SerialPortNode::send_tf2_transforms(const TransformMsg::SharedPtr msg,
    //     const std::string& header_frame_id, const std::string& child_frame_id)
    // {
    //     rclcpp::Time now = this->get_clock()->now();
    //     send_tf2_transforms(msg, header_frame_id, child_frame_id, now);
    // }

    // void SerialPortNode::send_tf2_transforms(const TransformMsg::SharedPtr msg,
    //     const std::string& header_frame_id,
    //     const std::string& child_frame_id,
    //     const rclcpp::Time& time) 
    // {
    //     // rclcpp::Time now = this->get_clock()->now();
    //     // camera->base_link transform.
    //     geometry_msgs::msg::TransformStamped transform;

    //     transform.transform.translation.x = msg->transform.translation.x;
    //     transform.transform.translation.y = msg->transform.translation.y;
    //     transform.transform.translation.z = msg->transform.translation.z;
        
    //     // tf2::Quaternion q;
    //     // q.setRPY(0, 0, 0);
    //     transform.transform.rotation.x = msg->transform.rotation.x;
    //     transform.transform.rotation.y = msg->transform.rotation.y;
    //     transform.transform.rotation.z = msg->transform.rotation.z;
    //     transform.transform.rotation.w = msg->transform.rotation.w;

    //     transform.header.frame_id = header_frame_id;
    //     transform.child_frame_id = child_frame_id;
    //     transform.header.stamp = msg->header.stamp;

    //     tf_broadcaster_->sendTransform(transform);
    // }

    bool SerialPortNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = params_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->debug_without_port_ = param.as_bool();
            break;
        case 1:
            this->baud_ = param.as_int();
            break;
        case 2:
            this->bytes_num_ = param.as_int();
            RCLCPP_WARN(this->get_logger(), "Now bytes_num = %d", this->bytes_num_);
            break;
        default:
            break;
        }
        return true;
    }
    
    rcl_interfaces::msg::SetParametersResult SerialPortNode::paramsCallback(const std::vector<rclcpp::Parameter>& params)
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

    std::unique_ptr<SerialPort> SerialPortNode::init_serial_port()
    {
        params_map_ =
        {
            {"debug_without_com", 0},
            {"baud", 1},
            {"bytes_num", 2}
        };

        this->declare_parameter<std::string>("port_id", "483/5740/200");
        this->declare_parameter<int>("baud", 115200);
        this->declare_parameter<bool>("debug_without_com", true);
        this->declare_parameter<bool>("use_buff_mode", false);
        this->declare_parameter<int>("bytes_num", 50);
        this->declare_parameter<bool>("is_sentry_mode", false);

        is_sentry_mode_ = this->get_parameter("is_sentry_mode").as_bool();
        id_ = this->get_parameter("port_id").as_string();
        baud_ = this->get_parameter("baud").as_int();
        debug_without_port_ = this->get_parameter("debug_without_com").as_bool();
        bytes_num_ = this->get_parameter("bytes_num").as_int();
        RCLCPP_INFO(this->get_logger(), "bytes_num: %d", bytes_num_);

        return std::make_unique<SerialPort>(id_, baud_, debug_without_port_, is_sentry_mode_);
    }
} //namespace serialport

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialport::SerialPortNode>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serialport::SerialPortNode)