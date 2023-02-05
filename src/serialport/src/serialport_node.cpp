/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2023-02-06 01:17:46
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/serialport_node.cpp
 */
#include "../include/serialport_node.hpp"

// #define SENTRY_RECV_NORMAL 0x05
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

            if(!debug_without_port_)
            {   // Use serial port.
                if(serial_port_->initSerialPort())
                {
                    imu_data_pub_ = this->create_publisher<SerialMsg>("/imu_msg", qos);
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

    // /**
    //  * @brief 将4个uchar转换为float
    //  * @param data data首地址指针
    //  * @return
    //  */
    // float SerialPortNode::ucharRaw2Float(unsigned char *data)
    // {
    //     float float_data;
    //     float_data = *((float*)data);
    //     return float_data;
    // };

    // uchar* SerialPortNode::float2UcharRaw(float float_data)
    // {
    //     uchar* raw_data = nullptr;
    //     raw_data = (uchar*)(&float_data); 
    //     return std::move(raw_data);
    // }   

    // /**
    //  * @brief uchar原始数据转换为float vector
    //  * @param data 首地址指针
    //  * @param bytes 字节数
    //  * @param vec float vector地址
    //  */
    // bool SerialPortNode::ucharRaw2FloatVector(unsigned char *data, int bytes, std::vector<float> &vec)
    // {
    //     std::vector<unsigned char*> pts;
    //     assert(bytes % 4 == 0);
    //     for (int i = 0; i < bytes; i+=4)
    //     {
    //         vec.push_back(ucharRaw2Float(&data[i]));
    //     }
    //     return true;
    // }

    // bool SerialPortNode::float2UcharRawArray(float float_data[], int num, uchar* raw_data)
    // {
    //     for(int ii = 0; ii < num; ++ii)
    //     {
    //         raw_data[ii * 4] = float_data[ii];
    //     }
    //     return true;
    // }
    
    void SerialPortNode::receive_data()
    {
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
            {
                RCLCPP_WARN(this->get_logger(),"CHECKSUM FAILED OR NO DATA RECVIED!!!");
            }
            
            // imu info pub.
            //Process IMU Datas

            std::vector<float> quat(4);
            std::vector<float> gyro(3);
            std::vector<float> acc(3);
            double bullet_speed;
            double theta;
            uchar mode = serial_port_->serial_data_.rdata[1];

            if (mode == SENTRY_RECV_NORMAL)
            {
                // mode = serial_data_.rdata[1]; // 模式位（自瞄or能量机关）
                // getQuat(&serial_data_.rdata[3]);
                // getGyro(&serial_data_.rdata[19]);
                // getAcc(&serial_data_.rdata[31]);
                // getSpeed(&serial_data_.rdata[43]); //接收下位机发送的弹速
                serial_port_->ucharRaw2FloatVector(&serial_port_->serial_data_.rdata[3], 16, quat);
                serial_port_->ucharRaw2FloatVector(&serial_port_->serial_data_.rdata[19], 12, gyro);
                serial_port_->ucharRaw2FloatVector(&serial_port_->serial_data_.rdata[31], 12, acc);
                bullet_speed = serial_port_->ucharRaw2Float(&serial_port_->serial_data_.rdata[43]);
                theta = serial_port_->ucharRaw2Float(&serial_port_->serial_data_.rdata[47]);
                
                SerialMsg serial_msg;
                serial_msg.imu.header.frame_id = "imu_link";
                serial_msg.imu.header.stamp = this->get_clock()->now();
                serial_msg.bullet_speed = serial_port_->bullet_speed_;
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
                imu_data_pub_->publish(std::move(serial_msg));
            }
        }
    }

    void SerialPortNode::send_armor_data(GimbalMsg::SharedPtr target_info) 
    {
        
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
        this->declare_parameter<bool>("debug_without_com", false);
        this->declare_parameter<bool>("use_buff_mode", false);
        this->declare_parameter<int>("bytes_num", 64);
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