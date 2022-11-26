/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2022-11-26 16:05:39
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/serialport/serialport_node.cpp
 */
#include "../../include/serialport/serialport_node.hpp"

namespace serialport
{
    serial_driver::serial_driver(const rclcpp::NodeOptions& options)
    : Node("serial_driver", options), device_name_("ttyACM0"), baud_(115200)
    {
        RCLCPP_WARN(this->get_logger(), "Serialport node...");
        
        // serial_port_ = init_serial_port();

        serial_port_old_ = init_serial_port_old();

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

        this->declare_parameter<bool>("debug_without_com", false);
        this->get_parameter("debug_without_com", debug_without_port);

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
         
        //tf2
        target_frame_ = this->declare_parameter<std::string>("target_frame", "world");

        std::chrono::duration<int> buffer_timeout(1);

        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

        auto time_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface()
        );

        tf2_buffer_->setCreateTimerInterface(time_interface);
        tf2_listener_ = 
            std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        gimbal_info_sub_.subscriber(this, "/gimbal_info");
        tf2_filter_ = std::make_shared<tf2_ros::MessageFilter<global_interface::msg::Gimbal>>(
            gimbal_info_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
            this->get_node_clock_interface(), buffer_timeout
        );

        //callback func
        // tf2_filter_->registerCallback(&);

        //initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // std::cout << 1 << std::endl;
        gimbal_motion_sub_ = this->create_subscription<global_interface::msg::Gimbal>(
            "/gimbal_info", 
            rclcpp::SensorDataQoS(),
            std::bind(&serial_driver::send_data, this, std::placeholders::_1)
        );

        // if(serial_port_->debug_without_port())
        // {
        //     serial_port_->open(device_name_, baud_);
        //     if(!serial_port_->is_open)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Serial open failed!");
        //     }
        //     else
        //     {
        //         receive_thread_ = std::thread(&serial_driver::receive_data, this);
        //     }
        // }

        if(!debug_without_port)
        {   //use serial port
            if(serial_port_old_->initSerialPort())
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

    std::unique_ptr<SerialPort> serial_driver::init_serial_port_old()
    {
        std::string id = "483/5740/200";
        int baud = 115200;
        this->declare_parameter("debug_without_com", true);
        bool debug_without_com = this->get_parameter("debug_without_com").as_bool();

        return std::make_unique<SerialPort>(id, baud, debug_without_com);
    }

    void serial_driver::handle_imu_data(const std::shared_ptr<global_interface::msg::Imu>& msg)
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

        this->declare_parameter("debug_without_com", true);
        bool debug_without_com = this->get_parameter("debug_without_com").as_bool();
        
        return std::make_unique<serialport>(id, baud, debug_without_com);
    }    

    void serial_driver::receive_data()
    {
        std::vector<uchar> header(1);
        std::vector<uchar> data(sizeof(ReceivePacket));

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
            //若串口离线则跳过数据发送
            if (serial_port_old_->need_init == true)
            {
                // cout<<"offline..."<<endl;
                usleep(5000);
                continue;
            }
            //数据读取不成功进行循环
            while (!serial_port_old_->get_Mode())
                ;
            // auto time_cap = std::chrono::steady_clock::now();

    // #ifdef USING_IMU_C_BOARD 
    //         auto imu_timestamp = serial.timestamp;
    //         auto delta_t = (int)(std::chrono::duration<double,std::micro>(imu_timestamp - image_timestamp).count());
    //         cout << "cam_imu data delta_t: " << delta_t << "us" <<  endl;
    // #endif
            // cout<<"Quat: "<<serial.quat[0]<<" "<<serial.quat[1]<<" "<<serial.quat[2]<<" "<<serial.quat[3]<<" "<<endl;
            // Eigen::Quaterniond quat = {serial.quat[0],serial.quat[1],serial.quat[2],serial.quat[3]};
            //FIXME:注意此处mode设置
            int mode = serial_port_old_->mode;
            float bullet_speed = serial_port_old_->bullet_speed;
            
            // int mode = 2;
            // Eigen::Quaterniond quat = {serial_port_old_->quat[0], serial_port_old_->quat[1], serial_port_old_->quat[2], serial_port_old_->quat[3]};
            // Eigen::Vector3d acc = {serial_port_old_->acc[0], serial_port_old_->acc[1], serial_port_old_->acc[2]};
            // Eigen::Vector3d gyro = {serial_port_old_->gyro[0], serial_port_old_->gyro[1], serial_port_old_->gyro[2]};
            
            // std::cout << 3 << std::endl;
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
            // transform_.transform.rotation.w = packet.quat[0];
            // transform_.transform.rotation.x = packet.quat[1];
            // transform_.transform.rotation.y = packet.quat[2];
            // transform_.transform.rotation.z = packet.quat[3];

            transform_.transform.rotation.w = serial_port_old_->quat[0];
            transform_.transform.rotation.x = serial_port_old_->quat[1];
            transform_.transform.rotation.y = serial_port_old_->quat[2];
            transform_.transform.rotation.z = serial_port_old_->quat[3];
            
            //imu rotation
            quat_ = {serial_port_old_->quat[0], serial_port_old_->quat[1], serial_port_old_->quat[2], serial_port_old_->quat[3]};
            rmat_imu_ = quat_.toRotationMatrix();

            //send the transformation
            tf_broadcaster_->sendTransform(transform_);
            // Eigen::Vector3d vec = quat.toRotationMatrix().eulerAngles(2,1,0);
            // cout<<"Euler : "<<vec[0] * 180.f / CV_PI<<" "<<vec[1] * 180.f / CV_PI<<" "<<vec[2] * 180.f / CV_PI<<endl;
            // cout<<"transmitting..."<<endl;
        }
    }

    void serial_driver::send_data(global_interface::msg::Gimbal::SharedPtr target_info) 
    {
        // std::cout << 4 << std::endl;

        // global_interface::msg::Gimbal msg;
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
            
        //     // if(!serial_port_->write((data)))
        //     // {
        //     //     RCLCPP_ERROR(this->get_logger(), "serial data sends failed!");
        //     // }
        // }
        if(!this->debug_without_port)
        {
            VisionData transmition_data = {target_info->pitch, target_info->yaw, target_info->distance, target_info->is_switched, 1, target_info->is_spinning, 0};
            serial_port_old_->TransformData(transmition_data);
            serial_port_old_->send();

            // RCLCPP_INFO(this->get_logger(), "send data...");
        }
        else
        {
            // std::cout << 4 << std::endl;
            //debug without com
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