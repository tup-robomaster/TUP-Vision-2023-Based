/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2023-05-31 17:42:16
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
        autoaim_info_sub_ = this->create_subscription<GimbalMsg>(
            "/armor_processor/gimbal_msg", 
            qos,
            std::bind(&SerialPortNode::armorMsgCallback, this, _1)
        );
        
        //能量机关msg订阅
        buff_info_sub_ = this->create_subscription<GimbalMsg>(
            "/buff_processor/gimbal_msg",
            qos,
            std::bind(&SerialPortNode::buffMsgCallback, this, _1)
        );

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        if (using_port_)
        {
            serial_msg_pub_ = this->create_publisher<SerialMsg>("/serial_msg", qos);
            // receive_timer_ = rclcpp::create_timer(this, this->get_clock(), 5ms, std::bind(&SerialPortNode::receiveData, this));
            
            watch_timer_ = rclcpp::create_timer(
                this, 
                this->get_clock(), 
                100ms, 
                std::bind(&SerialPortNode::serialWatcher, this)
            );
        }

        receive_thread_ = std::make_unique<std::thread>(&SerialPortNode::receiveData, this);
    }

    SerialPortNode::~SerialPortNode()
    {
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
     * @details 标志位为0xA5的package包含模式位、陀螺仪数据、弹速以及上一发弹丸的发弹延迟
     * 
     */
    void SerialPortNode::receiveData()
    {
        vector<float> vehicle_pos_info;
        while (1)
        {
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
                        usleep(1000);
                        continue;
                    }
                }
                
                uchar flag = serial_port_->serial_data_.rdata[0];
                uchar mode = serial_port_->serial_data_.rdata[1];
                mode_ = mode;

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(), 
                    100,
                    "mode:%d", 
                    mode
                );
                
                if (flag == 0xA5)
                {
                    std::vector<float> quat;
                    std::vector<float> gyro;
                    std::vector<float> acc;
                    float bullet_speed = 0.0;
                    float shoot_delay = 0.0;
                    
                    // Process IMU Datas.
                    data_transform_->getQuatData(&serial_port_->serial_data_.rdata[3], quat);
                    data_transform_->getGyroData(&serial_port_->serial_data_.rdata[19], gyro);
                    data_transform_->getAccData(&serial_port_->serial_data_.rdata[31], acc);
                    data_transform_->getBulletSpeed(&serial_port_->serial_data_.rdata[43], bullet_speed);
                    data_transform_->getShootDelay(&serial_port_->serial_data_.rdata[47], shoot_delay);
                    
                    // Gimbal angle.
                    // float yaw_angle = 0.0, pitch_angle = 0.0;
                    // data_transform_->getYawAngle(flag, &serial_port_->serial_data_.rdata[55], yaw_angle);
                    // data_transform_->getPitchAngle(flag, &serial_port_->serial_data_.rdata[59], pitch_angle);
                    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "yaw_angle:%.2f", yaw_angle);
                    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "pitch_angle:%.2f", pitch_angle);
                    if (print_serial_info_)
                    {
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "quat:[%.3f %.3f %.3f %.3f]", quat[0], quat[1], quat[2], quat[3]);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "gyro:[%.3f %.3f %.3f]", gyro[0], gyro[1], gyro[2]);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "acc:[%.3f %.3f %.3f]", acc[0], acc[1], acc[2]);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "bullet_speed::%.3f", bullet_speed);
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100, "shoot_delay:%.3f", shoot_delay);
                    }

                    rclcpp::Time now = this->get_clock()->now();
                    SerialMsg serial_msg;
                    serial_msg.header.frame_id = "serial";
                    serial_msg.header.stamp = now;
                    serial_msg.imu.header.frame_id = "imu_link";
                    serial_msg.imu.header.stamp = now;
                    serial_msg.mode = mode;
                    serial_msg.bullet_speed = bullet_speed;
                    serial_msg.shoot_delay = shoot_delay;
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
                }
            }
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
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), 
            *this->get_clock(),
            100,
            "sending_mode: %d", 
            mode
        );

        if (this->using_port_)
        {   
            VisionAimData vision_data;
            if (mode == AUTOAIM_TRACKING || mode == AUTOAIM_NORMAL ||
                mode == AUTOAIM_SLING || mode == SMALL_BUFF || 
                mode == BIG_BUFF || mode == OUTPOST_ROTATION_MODE ||
                mode == SENTRY_NORMAL
            )
            {
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
                
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(), 
                    50,
                    "is_target_switched: %d",
                    (target_info->is_switched || target_info->is_spinning_switched)
                );
            }
            else 
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(),
                    100,
                    "Error mode: %d", 
                    mode
                );
                return false;
            }

            // 根据不同mode进行对应的数据转换
            data_transform_->transformData(mode, vision_data, serial_port_->Tdata);
            
            // Time of entire loop.
            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Time start = target_info->header.stamp;
            double duration = (now.nanoseconds() - start.nanoseconds()) / 1e6;
            
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), 
                *this->get_clock(), 
                100, 
                "All_delay:%.2fms", 
                duration
            );
            
            // 数据发送
            mutex_.lock();
            serial_port_->sendData();
            mutex_.unlock();
            return true;
        }
        else
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Not use port...");
            return false;
        }
    }

    /**
     * @brief 自瞄消息订阅回调函数
     * 
     * @param gimbal_msg 云台转动消息
     */
    void SerialPortNode::armorMsgCallback(GimbalMsg::SharedPtr gimbal_msg) 
    {
        int mode = mode_;
        if (mode == AUTOAIM_TRACKING || mode == AUTOAIM_NORMAL ||
            mode == AUTOAIM_SLING || mode == OUTPOST_ROTATION_MODE ||
            mode == SENTRY_NORMAL
        )
        {
            if (!sendData(gimbal_msg))
            {   // Debug without com.
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(), 
                    500, 
                    "Sub autoaim msg..."
                );
            }
        }
    }

    /**
     * @brief 能量机关消息订阅回调函数
     * 
     * @param gimbal_msg 云台转动消息
     */
    void SerialPortNode::buffMsgCallback(GimbalMsg::SharedPtr gimbal_msg) 
    {
        int mode = mode_;
        if (mode == SMALL_BUFF || mode == BIG_BUFF)
        {
            if (!sendData(gimbal_msg))
            {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), 
                    *this->get_clock(), 
                    500, 
                    "Sub buff msg..."
                );
            }
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
        this->get_parameter("port_id", this->id_);

        this->declare_parameter<int>("baud", 115200);
        this->get_parameter("baud", this->baud_);

        this->declare_parameter<bool>("using_port", true);
        this->get_parameter("using_port", this->using_port_);

        this->declare_parameter<bool>("tracking_target", false);
        this->get_parameter("tracking_target", this->tracking_target_);

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