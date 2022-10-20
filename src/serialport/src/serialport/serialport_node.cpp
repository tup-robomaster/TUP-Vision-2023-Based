/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-25 23:42:42
 * @LastEditTime: 2022-09-28 16:50:18
 * @FilePath: /tup_2023/src/serialport/src/serialport/serialport_node.cpp
 */
#include "../../include/serialport/serialport_node.hpp"

namespace serialport
{
    serial_driver::serial_driver(const rclcpp::NodeOptions& options)
    : Node("serial_driver",options), device_name("ttyACM0"), baud(115200)
    {
        // node = std::make_shared<rclcpp::Node>("serial_driver", options);

        joint_state_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", rclcpp::QoS(rclcpp::KeepLast(10)));

        gimbal_motion_sub = this->create_subscription<global_interface::msg::Gimbal>
        ("gimbal_motion/gimbal", rclcpp::SensorDataQoS(),
        std::bind(&serial_driver::send_data, this, std::placeholders::_1));

        if(serial_port->debug_without_port())
        {
            serial_port->open(device_name, baud);
            if(!serial_port->is_open)
            {
                RCLCPP_INFO(this->get_logger(), "Serial open failed!");
            }
            
            thread_read = std::thread(&serial_driver::receive_data, this);
            thread_watch = std::thread(&serialport::init, this);
        }
    }

    serial_driver::~serial_driver()
    {
        // if(buffer) delete buffer;
        // buffer = NULL;
    }
    
    void serial_driver::receive_data()
    {
        std::vector<uint8_t> header(1);
        std::vector<uint8_t> data(sizeof(ReceivePacket));

        while (rclcpp::ok())
        {
            if(serial_port->debug_without_port())
            {
                serial_port->read(header);
                if(header.at(0) == 0xA5)
                {
                    serial_port->read(data);
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
                        joint_state_pub->publish(joint_state);
                    }        
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Crc check error!");
                    }   
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid header: %02X", header.at(0));
                }
            }
        }
    }

    void serial_driver::send_data(global_interface::msg::Gimbal::SharedPtr msg) 
    {
        if(serial_port->debug_without_port())
        {
            SendPacket packet;
            packet.header = 0xA5;
            packet.pitch_angle = msg->pitch;
            packet.yaw_angle = msg->yaw;
            packet.dis = msg->distance;
            packet.isFindTarget = msg->is_target;
            packet.isSwitched = msg->is_switched;
            packet.isSpinning = msg->is_spinning;
            packet.isMiddle = msg->is_middle;
            
            Append_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
            
            std::vector<uint8_t> data = packet_to_vec(packet);
            serial_port->write((data));
            
            // if(!serial_port->write((data)))
            // {
            //     RCLCPP_ERROR(this->get_logger(), "serial data sends failed!");
            // }
        }
    }
} //serialport

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(serialport::serial_driver)