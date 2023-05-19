/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2023-05-06 23:42:38
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
            serial_port_ = initSerialPort();
            data_transform_ = initDataTransform();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
        }

        // QoS
        rclcpp::QoS qos(0);
        qos.keep_last(5);
        qos.reliable();
        qos.durability();
        qos.deadline();
        // qos.best_effort();
        // qos.durability_volatile();
   
        rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
        rmw_qos.depth = 5;
        
        //自瞄msg订阅
        if (!tracking_target_)
        {   //预测信息订阅
            RCLCPP_WARN(this->get_logger(), "Prediciton!!!");
            autoaim_info_sub_ = this->create_subscription<GimbalMsg>(
                "/armor_processor/gimbal_msg", 
                qos,
                std::bind(&SerialPortNode::armorMsgCallback, this, _1)
            );
        }
        else
        {   //跟踪信息订阅
            RCLCPP_WARN(this->get_logger(), "Tracking!!!");
            autoaim_tracking_sub_ = this->create_subscription<GimbalMsg>(
                "/armor_processor/tracking_msg", 
                qos,
                std::bind(&SerialPortNode::armorMsgCallback, this, _1)
            );
        }
        
        //能量机关msg订阅
        buff_info_sub_ = this->create_subscription<GimbalMsg>(
            "/buff_processor/gimbal_msg",
            qos,
            std::bind(&SerialPortNode::buffMsgCallback, this, _1)
        );

        // //决策消息订阅
        // decision_msg_sub_ = this->create_subscription<DecisionMsg>(
        //     "robot_decision/decision",
        //     qos,
        //     std::bind(&SerialPortNode::decisionMsgCallback, this, _1)
        // );
        
        //创建发送数据定时器
        // timer_ = this->create_wall_timer(5ms, std::bind(&SerialPortNode::sendData, this));
        // send_timer_ = rclcpp::create_timer(this, this->get_clock(), 30ms, std::bind(&SerialPortNode::sendingData, this));

        // tf2
        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // if (using_port_)
        // {   // Use serial port.
            // if (serial_port_->openPort())
            // {
                serial_msg_pub_ = this->create_publisher<SerialMsg>("/serial_msg", qos);
                joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", qos);
                car_pos_pub_ = this->create_publisher<CarPosMsg>("/car_pos", qos);
                obj_hp_pub_ = this->create_publisher<ObjHPMsg>("/obj_hp", qos);
                game_msg_pub_ = this->create_publisher<GameMsg>("/game_info", qos);
                receive_thread_ = std::make_unique<std::thread>(&SerialPortNode::receiveData, this);
                // receive_timer_ = rclcpp::create_timer(this, this->get_clock(), 5ms, std::bind(&SerialPortNode::receiveData, this));
                sentry_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel",
                    rclcpp::SensorDataQoS(), 
                    std::bind(&SerialPortNode::sentryNavCallback, this, _1)
                );
                watch_timer_ = rclcpp::create_timer(this, this->get_clock(), 500ms, std::bind(&SerialPortNode::serialWatcher, this));
            // }
        // }
    }

    SerialPortNode::~SerialPortNode()
    {
        // if (receive_thread_.joinable())
        //     receive_thread_.join();
    }

    void SerialPortNode::decisionMsgCallback(DecisionMsg::SharedPtr msg)
    {
        decision_mutex_.lock();
        decision_msg_ = *msg;
        decision_msg_.header.stamp = this->get_clock()->now();
        decision_mutex_.unlock();
    }

    /**
     * @brief 串口监管线程
     * 
     */
    void SerialPortNode::serialWatcher()
    {
        if (access(serial_port_->serial_data_.device.path.c_str(), F_OK) == -1 || !serial_port_->serial_data_.is_initialized)
        {
            serial_port_->serial_data_.is_initialized = true;
            if (!serial_port_->openPort())
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Port open failed!!!");
            }
        }
    }

    /**
     * @brief 数据发送线程
     * @details 下位机分三个包发送数据，标志位为0xA5的包包含模式位、陀螺仪数据、弹速，标志位为0xB5的包包含14个float型的场地车辆位置数据（x,y），
     * @details 标志位为0xC5的包包含6个float型的场地车辆位置信息、10个short型的全场车辆HP信息以及一个short型的比赛进行时间戳信息。
     * 
     */
    void SerialPortNode::receiveData()
    {
        vector<float> vehicle_pos_info;
        while (1)
        {
            // stamp_ = this->get_clock()->now();
            // rclcpp::Time now = this->get_clock()->now();
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "rec_delay:%.3fms", (now.nanoseconds() - stamp_.nanoseconds()) / 1e6);

            if (!using_port_)
            {
                geometry_msgs::msg::TransformStamped t;

                // Read message content and assign it to corresponding tf variables
                t.header.stamp = this->get_clock()->now();
                t.header.frame_id = "base_link";
                t.child_frame_id = "imu_link";
                
                // Translation
                t.transform.translation.x = 0.0;
                t.transform.translation.y = 0.0;
                t.transform.translation.z = 0.0;

                // Rotation
                t.transform.rotation.x = 0.0;
                t.transform.rotation.y = 0.0;
                t.transform.rotation.z = 0.0;
                t.transform.rotation.w = 1.0;

                // Send the transformation
                tf_broadcaster_->sendTransform(t);
            }
            else
            {
                // 若串口离线则跳过数据发送
                if (!serial_port_->serial_data_.is_initialized)
                {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "Serial port offline!!!");
                    usleep(1000);
                    continue;
                }

                // 数据读取不成功进行循环
                bool is_receive_data = false; 
                while (!is_receive_data)
                {
                    mutex_.lock();
                    is_receive_data = serial_port_->receiveData();
                    mutex_.unlock();
                    if(!is_receive_data)
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "CHECKSUM FAILED OR NO DATA RECVIED!!!");
                        // continue;
                        // usleep(1000);
                    }
                }
                
                uchar flag = serial_port_->serial_data_.rdata[0];
                uchar mode = serial_port_->serial_data_.rdata[1];
                mode_ = mode;
                // RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "mode:%d", mode);
                // RCLCPP_INFO(this->get_logger(), "mode:%d", mode);
                
                if (flag == 0xA5)
                {
                    // RCLCPP_INFO_THROTTLE(this->get_logger(), this->serial_port_->steady_clock_, 1000, "mode:%d", mode);
                    std::vector<float> quat;
                    std::vector<float> gyro;
                    std::vector<float> acc;
                    float bullet_speed;
                    float theta;
                    float pitch;
                    data_transform_->getThetaAngle(&serial_port_->serial_data_.rdata[47], theta);
                    data_transform_->getThetaAngle(&serial_port_->serial_data_.rdata[51], pitch);
                    //Process IMU Datas
                    data_transform_->getQuatData(&serial_port_->serial_data_.rdata[3], quat);
                    data_transform_->getGyroData(&serial_port_->serial_data_.rdata[19], gyro);
                    data_transform_->getAccData(&serial_port_->serial_data_.rdata[31], acc);
                    data_transform_->getBulletSpeed(&serial_port_->serial_data_.rdata[43], bullet_speed);
                    
                    // Gimbal angle
                    // float yaw_angle = 0.0, pitch_angle = 0.0;
                    // data_transform_->getYawAngle(flag, &serial_port_->serial_data_.rdata[55], yaw_angle);
                    // data_transform_->getPitchAngle(flag, &serial_port_->serial_data_.rdata[59], pitch_angle);
                    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "yaw_angle:%.2f", yaw_angle);
                    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "pitch_angle:%.2f", pitch_angle);
                    if (print_serial_info_)
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "mode:%d", mode);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "quat:[%f %f %f %f]", quat[0], quat[1], quat[2], quat[3]);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "gyro:[%f %f %f]", gyro[0], gyro[1], gyro[2]);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "acc:[%f %f %f]", acc[0], acc[1], acc[2]);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, "bullet_speed::%f", bullet_speed);
                    }

                    rclcpp::Time now = this->get_clock()->now();
                    SerialMsg serial_msg;
                    serial_msg.header.frame_id = "serial";
                    serial_msg.header.stamp = now;
                    serial_msg.imu.header.frame_id = "imu_link";
                    serial_msg.imu.header.stamp = now;
                    serial_msg.mode = mode;
                    serial_msg.bullet_speed = bullet_speed;
                    serial_msg.imu.orientation.w = quat[0];
                    serial_msg.imu.orientation.x = quat[1];
                    serial_msg.imu.orientation.y = quat[2];
                    serial_msg.imu.orientation.z = quat[3];
                    serial_msg.imu.angular_velocity.x = gyro[0];
                    serial_msg.imu.angular_velocity.y = gyro[1];
                    serial_msg.imu.angular_velocity.z = gyro[2];
                    serial_msg.imu.linear_acceleration.x = acc[0];
                    serial_msg.imu.linear_acceleration.y = acc[1];
                    serial_msg.imu.linear_acceleration.z = acc[2];
                    
                    geometry_msgs::msg::TransformStamped t;

                    // Read message content and assign it to corresponding tf variables
                    t.header.stamp = this->get_clock()->now();
                    t.header.frame_id = "base_link";
                    t.child_frame_id = "imu_link";

                    // Translation
                    t.transform.translation.x = 0.0;
                    t.transform.translation.y = 0.0;
                    t.transform.translation.z = 0.0;

                    // Rotation
                    t.transform.rotation.x = serial_msg.imu.orientation.x;
                    t.transform.rotation.y = serial_msg.imu.orientation.y;
                    t.transform.rotation.z = serial_msg.imu.orientation.z;
                    t.transform.rotation.w = serial_msg.imu.orientation.w;

                    // Send the transformation
                    tf_broadcaster_->sendTransform(t);

                    // Pub serial msg
                    serial_msg_pub_->publish(std::move(serial_msg));

                    // RCLCPP_WARN(this->get_logger(), "serial_msg_pub:%.3fs", now.nanoseconds() / 1e9);

                    sensor_msgs::msg::JointState joint_state;
                    joint_state.header.stamp = this->get_clock()->now();
                    joint_state.name.push_back("gimbal_yaw_joint");
                    joint_state.name.push_back("gimbal_pitch_joint");
                    joint_state.position.push_back(theta);
                    joint_state.position.push_back(-pitch);
                    joint_state_pub_->publish(joint_state);
                }
                else if (flag == 0xB5)
                {
                    data_transform_->getPosInfo(flag, &serial_port_->serial_data_.rdata[3], vehicle_pos_info);
                }
                else if (flag == 0xC5)
                {
                    vector<ushort> hp;
                    ushort timestamp;
                    data_transform_->getPosInfo(flag, &serial_port_->serial_data_.rdata[3], vehicle_pos_info);
                    data_transform_->getHPInfo(flag, &serial_port_->serial_data_.rdata[27], hp);
                    data_transform_->getGameInfo(flag, &serial_port_->serial_data_.rdata[47], timestamp);

                    CarPosMsg car_pos_msg;
                    ObjHPMsg obj_hp_msg;
                    GameMsg game_msg;

                    for(int ii = 0; ii < 20; ii+=2)
                    {
                        car_pos_msg.pos[ii].x = vehicle_pos_info[ii];
                        car_pos_msg.pos[ii].y = vehicle_pos_info[ii+1];
                        obj_hp_msg.hp[ii/2] = hp[ii/2];
                    }

                    if (print_referee_info_)
                    {
                        for(int ii = 0; ii < 20; ii++)
                            RCLCPP_INFO(this->get_logger(), "Pos:%.2f", vehicle_pos_info[ii]);
                        for(int ii = 0; ii < 10; ii++)
                            RCLCPP_INFO(this->get_logger(), "HP:%.2d", hp[ii]);
                        RCLCPP_INFO(this->get_logger(), "timestamp:%.2d", timestamp);
                    }

                    rclcpp::Time now = this->get_clock()->now();
                    car_pos_msg.header.frame_id = "";
                    car_pos_msg.header.stamp = now;
                    car_pos_pub_->publish(move(car_pos_msg));
                    
                    obj_hp_msg.header.frame_id = "";
                    obj_hp_msg.header.stamp = now;
                    obj_hp_pub_->publish(move(obj_hp_msg));

                    game_msg.header.frame_id = "";
                    game_msg.header.stamp = now;
                    game_msg.timestamp = timestamp;
                    game_msg_pub_->publish(move(game_msg));

                    vehicle_pos_info.clear();
                }
            }

            // stamp_ = now;
            // rclcpp::Time now = this->get_clock()->now();        
            // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "rec_delay:%.3fms", (now.nanoseconds() - stamp_.nanoseconds()) / 1e6);

        }

    }

    /**
     * @brief 数据发送函数
     * 
     * @param target_info 云台信息（pitch、yaw轴偏转角度等）
     * @return true 
     * @return false 
     */
    bool SerialPortNode::sendData(GimbalMsg::SharedPtr target_info)
    {
        int mode = mode_;
        // if (mode == SENTRY_NORMAL)
        // {
        //     decision_mutex_.lock();
        //     mode = decision_msg_.mode;
        //     decision_mutex_.unlock();
        // }
        // RCLCPP_WARN(this->get_logger(), "Mode:%d", mode);
        if (this->using_port_)
        {   
            VisionAimData vision_data;
            if (mode == AUTOAIM || mode == HERO_SLING || mode == OUTPOST_ROTATION_MODE
            || mode == SMALL_BUFF || mode == BIG_BUFF || mode == SENTRY_NORMAL)
            {
                // RCLCPP_WARN(this->get_logger(), "Sub autoaim msg!!!");
                vision_data = 
                {
                    (serial_port_->steady_clock_.now().nanoseconds() / 1e6),
                    (float)target_info->pitch, 
                    (float)target_info->yaw, 
                    (float)target_info->distance, 
                    (target_info->is_switched || target_info->is_spinning_switched), 
                    target_info->is_target, 
                    target_info->is_spinning, 
                    target_info->is_prediction,
                    target_info->is_shooting,
                    {target_info->meas_point_cam.x, target_info->meas_point_cam.y, target_info->meas_point_cam.z},
                    {target_info->pred_point_cam.x, target_info->pred_point_cam.y, target_info->pred_point_cam.z}
                };
                RCLCPP_WARN_EXPRESSION(this->get_logger(), (target_info->is_switched || target_info->is_spinning_switched), "Target switched!!!");
            }
            else 
                return false;

            //根据不同mode进行对应的数据转换
            data_transform_->transformData(mode, vision_data, serial_port_->Tdata);
            
            // End!
            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Time start = target_info->header.stamp;
            // builtin_interfaces::msg::Time now_timestamp = now;
            // double dura = (now_timestamp.nanosec - target_info->header.stamp.nanosec) / 1e6;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "All_delay:%.2fms", (now.nanoseconds() - start.nanoseconds()) / 1e6);
            
            //数据发送
            mutex_.lock();
            serial_port_->sendData();
            mutex_.unlock();
            // flag_ = true;
            return true;
        }
        else
            return false;
    }

    /**
     * @brief 自瞄消息订阅回调函数
     * 
     * @param target_info 目标信息
     */
    void SerialPortNode::armorMsgCallback(GimbalMsg::SharedPtr target_info) 
    {
        int mode = mode_;
        if ((mode == SENTRY_NORMAL && decision_msg_.decision_id == AUTOAIM)
        || (mode == AUTOAIM || mode == HERO_SLING))
        {
            if (!sendData(target_info))
            {   // Debug without com.
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub autoaim msg...");
            }
        }
        return;
    }

    /**
     * @brief 能量机关消息订阅回调函数
     * 
     * @param target_info 目标信息
     */
    void SerialPortNode::buffMsgCallback(GimbalMsg::SharedPtr target_info) 
    {
        int mode = mode_;
        if (mode == SMALL_BUFF || mode == BIG_BUFF)
        {
            if (!sendData(target_info))
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub buff msg...");
        }
        return;
    }

    /**
     * @brief 哨兵消息订阅回调函数
     * 
     * @param msg 目标信息
     */
    void SerialPortNode::sentryNavCallback(geometry_msgs::msg::Twist::SharedPtr msg)
    {
        int mode = mode_;
        // RCLCPP_WARN(this->get_logger(), "Mode:%d", mode);
        if (this->using_port_)
        {   
            VisionNavData vision_data;
            vision_data.linear_velocity[0] = msg->linear.x;
            vision_data.linear_velocity[1] = msg->linear.y;
            vision_data.linear_velocity[2] = msg->linear.z;
            vision_data.angular_velocity[0] = msg->angular.x;
            vision_data.angular_velocity[1] = msg->angular.y;
            vision_data.angular_velocity[2] = msg->angular.z * 0.003;
            //根据不同mode进行对应的数据转换
            data_transform_->transformData(mode, vision_data, serial_port_->Tdata);
            //数据发送
            mutex_.lock();
            serial_port_->sendData();
            mutex_.unlock();
            return;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub buff msg...");
        }
    }

    /**
     * @brief 修改参数
     * 
     * @param param 参数服务器变动的参数
     * @return true 
     * @return false 
     */
    bool SerialPortNode::setParam(rclcpp::Parameter param)
    {
        auto param_idx = params_map_[param.get_name()];
        switch (param_idx)
        {
        case 0:
            this->using_port_ = param.as_bool();
            break;
        case 1:
            this->baud_ = param.as_int();
            break;
        case 2:
            this->tracking_target_ = param.as_bool();
            break;
        case 3:
            this->print_serial_info_ = param.as_bool();
            break;
        case 4:
            this->print_referee_info_= param.as_bool();
            break;
        default:
            break;
        }
        return true;
    }
    
    /**
     * @brief 参数回调函数
     * 
     * @param params 参数服务器变动的参数
     * @return rcl_interfaces::msg::SetParametersResult 
     */
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

    /**
     * @brief 初始化串口类
     * 
     * @return std::unique_ptr<SerialPort> 
     */
    std::unique_ptr<SerialPort> SerialPortNode::initSerialPort()
    {
        params_map_ =
        {
            {"using_port", 0},
            {"baud", 1},
            {"tracking_target", 2},
            {"print_serial_info", 3},
            {"print_referee_info", 4}
        };

        this->declare_parameter<std::string>("port_id", "483/5740/200");
        this->get_parameter("port_id", id_);

        this->declare_parameter<int>("baud", 115200);
        this->get_parameter("baud", baud_);

        this->declare_parameter<bool>("using_port", true);
        this->get_parameter("using_port", using_port_);

        this->declare_parameter<bool>("tracking_target", false);
        this->get_parameter("tracking_target", tracking_target_);

        this->declare_parameter("print_serial_info", false);
        this->get_parameter("print_serial_info", this->print_serial_info_);

        this->declare_parameter("print_referee_info", false);
        this->get_parameter("print_referee_info", this->print_referee_info_);

        return std::make_unique<SerialPort>(id_, baud_, using_port_);
    }

    /**
     * @brief 初始化数据转换类
     * 
     * @return std::unique_ptr<DataTransform> 
     */
    std::unique_ptr<DataTransform> SerialPortNode::initDataTransform()
    {
        return std::make_unique<DataTransform>();
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