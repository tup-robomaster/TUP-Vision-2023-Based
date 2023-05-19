#include "../../include/serialport/serialport.hpp"

namespace serialport
{
    /**
     *@brief 参数说明
    *@param  fd       类型  int  打开的串口文件句柄
    *@param  speed    类型  int  波特率
    *@param  databits 类型  int  数据位   取值 为 7 或者8
    *@param  stopbits 类型  int  停止位   取值为 1 或者2
    *@param  parity   类型  int  效验类型 取值为N,E,O,S
    *@param  portchar 类型  char* 串口路径
    */

    SerialPort::SerialPort(const string ID, const int BAUD, bool using_port)
    : using_port_(using_port), logger_(rclcpp::get_logger("serial_port"))
    {
        serial_data_.is_initialized = false;
        serial_data_.baud = BAUD;
        serial_id_ = ID;

        if(!this->using_port_) //无串口调试
        {
            RCLCPP_WARN(logger_, "Debug without com...");
            withoutSerialPort();
        }
        else
        {
            RCLCPP_INFO(logger_, "Initializing serial port...");
            openPort();
        }
    }

    SerialPort::SerialPort()
    : logger_(rclcpp::get_logger("serial_port"))
    {}

    SerialPort::~SerialPort()
    {}

    /**
     * @brief 数据接收函数
     * 
     * @param lens 
     * @return true 
     * @return false 
     */
    bool SerialPort::receiveData(int lens)
    {
        // timestamp_ = this->steady_clock_.now();
        // rclcpp::Time now = this->steady_clock_.now();
        // cout << "rec_delay:" << (now.nanoseconds() - timestamp_.nanoseconds()) / 1e6 << "ms" << endl;
        
        int bytes;
        char *name = ttyname(serial_data_.fd);
        if (name == NULL)
        {
            RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "tty is null...");
            serial_data_.is_initialized = false;
            return false;
        } 
        int result = ioctl(serial_data_.fd, FIONREAD, &bytes);
        if (result == -1)
        {
            serial_data_.is_initialized = false;
            // return false;
        }
        // if (bytes == 0)
        // {
        //     return false;
        // }
        
        // rclcpp::Time st = this->steady_clock_.now();
        // timestamp_ = this->steady_clock_.now();
        bytes = read(serial_data_.fd, serial_data_.rdata, (size_t)(lens));
        // rclcpp::Time now = this->steady_clock_.now();

        // cout << "rec_delay1:" << (now.nanoseconds() - timestamp_.nanoseconds()) / 1e6 << "ms" << endl;
        // timestamp_ = now;

        // bytes = read(serial_data_.fd, serial_data_.rdata, (size_t)(lens));
        // now = this->steady_clock_.now();

        // cout << "rec_delay2:" << (now.nanoseconds() - timestamp_.nanoseconds()) / 1e6 << "ms" << endl;

        // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 100, "rec_delay:%.3fms", (now.nanoseconds() - timestamp_.nanoseconds()) / 1e6);


        if ((serial_data_.rdata[0] == 0xA5 || serial_data_.rdata[0] == 0xB5 || serial_data_.rdata[0] == 0xC5)
            && crc_check_.Verify_CRC8_Check_Sum(serial_data_.rdata, 3)
            && crc_check_.Verify_CRC16_Check_Sum(serial_data_.rdata, (uint32_t)(lens)))
        {
            // cout << 1 << endl;
            // rclcpp::Time now = this->steady_clock_.now();
            // cout << "rec_delay:" << (now.nanoseconds() - timestamp_.nanoseconds()) / 1e6 << "ms" << endl;
            // timestamp_ = now;
            return true;
        }

        return false;
    }

    /**
     * @brief 数据发送函数
     * 
     */
    void SerialPort::sendData(int bytes_num)
    {
        auto write_stauts = write(serial_data_.fd, Tdata, bytes_num);
        RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "Sending msg...");
        return;
    }

    /**
     * @brief 打开串口
     * 
     * @return true 
     * @return false 
     */
    bool SerialPort::openPort()
    {
        serial_data_.device = setDeviceByID(listPorts());
        const string alias = "/dev/" + serial_data_.device.alias;

        if (alias.length() - 4 == 0)
            return false;

        close(serial_data_.last_fd);
        serial_data_.fd = open(alias.c_str(), O_RDWR | O_NOCTTY);

        serial_data_.speed = serial_data_.baud;
        serial_data_.databits = 8;
        serial_data_.stopbits = 1;
        serial_data_.parity = 'N';

        if (serial_data_.fd == -1)
        {
            perror(alias.c_str());
            return false;
        }

        RCLCPP_INFO(logger_, "Openning %s...", alias.c_str());
        setBrate();
        if (!setBit())
        {
            RCLCPP_WARN(logger_, "Set Parity Error.");
            exit(0);
        }
        RCLCPP_INFO(logger_, "Open successed...");

        serial_data_.last_fd = serial_data_.fd;
        serial_data_.is_initialized = true;
        return true;
    }

    /**
     * @brief 关闭通讯协议接口
     * 
     */
    void SerialPort::closePort()
    {
        close(serial_data_.fd);
    }

    /**
     * @brief 返回所有可用串口
     * **/
    std::vector<Device> SerialPort::listPorts()
    {
        vector<Device> devices;
        for(auto port_dir : DEFAULT_PORT)
        {
            std::vector<string> availible_path;
            auto general_path = "/sys/class/tty/"+ port_dir;
            for(int i = 0; i < MAX_ITER;i++)
            {
                auto tty_dir_path = general_path + to_string(i);

                if (access(tty_dir_path.c_str(),F_OK) != -1)
                {
                    Device dev;
                    availible_path.push_back(tty_dir_path);
                    auto real_path = symbolicToReal(tty_dir_path);
                    string info_path;
                    //需注意ttyACM与ttyUSB的uevent文件实际深度不同
                    if (port_dir == "ttyACM")
                        info_path = getParent(getParent(getParent(real_path)));
                    else if (port_dir == "ttyUSB")
                        info_path = getParent(getParent(getParent(getParent(real_path))));
                    dev = getDeviceInfo(info_path);
                    dev.alias = port_dir + to_string(i);
                    dev.path = tty_dir_path;
                    devices.push_back(dev);
                    // cout << dev.id << endl;
                }
            } 
        }
        return devices;
    }

    /**
     * @brief 获取串口有关信息
     * @return 串口类
     * **/
    Device SerialPort::getDeviceInfo(string path)
    {
        Device dev;
        auto uevent_path = path + "/uevent";
        auto texts = readLines(uevent_path);
        for(auto text : texts)
        {
            int equal_idx = text.find("=");
            string config_type = text.substr(0,equal_idx);
            string config_info = text.substr(equal_idx + 1);

            if (config_type == "PRODUCT")
            {
                dev.id = config_info;
            }
        }
        return dev;
    }

    /**
     * @brief 通过ID设置选取串口
     * @return 串口
     * **/
    Device SerialPort::setDeviceByID(std::vector<Device> devices)
    {
        for (auto dev : devices)
        {
            // cout << dev.id << endl;
            if (dev.id == serial_id_)
                return dev;
        }
        return Device();
    }

    // TODO: finish visual com port
    bool SerialPort::withoutSerialPort()
    {
        return true;
    }

    /**
     *@brief 设置波特率
    */
    void SerialPort::setBrate()
    {
        // int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
        // 				   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
        // int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
        // 				  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300};

        int speed_arr[] = {B921600, B460800, B230400, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
        int name_arr[] = {921600, 460800, 230400,115200, 38400, 19200, 9600, 4800, 2400, 1200,  300};
        int i;
        int status;
        struct termios Opt;
        tcgetattr(serial_data_.fd, &Opt);

        for (i = 0; i < sizeof(speed_arr) / sizeof(int);  ++i)
        {
            if (serial_data_.speed == name_arr[i])
            {
                tcflush(serial_data_.fd, TCIOFLUSH);//清空缓冲区的内容
                cfsetispeed(&Opt, speed_arr[i]);//设置接受和发送的波特率
                cfsetospeed(&Opt, speed_arr[i]);

                status = tcsetattr(serial_data_.fd, TCSANOW, &Opt); //使设置立即生效
                if (status != 0)
                {
                    perror("tcsetattr fd1");
                    return;
                }
                tcflush(serial_data_.fd, TCIOFLUSH);
            }
        }
    }

    /**
     *@brief 设置串口数据位，停止位和效验位
    */
    bool SerialPort::setBit()
    {
        struct termios termios_p;

        if (tcgetattr(serial_data_.fd, &termios_p)  !=  0)
        {
            perror("SetupSerial 1");
            return false;
        }

        termios_p.c_cflag |= (CLOCAL | CREAD);  //接受数据
        termios_p.c_cflag &= ~CSIZE;//设置数据位数

        switch (serial_data_.databits)
        {
        case 7:
            termios_p.c_cflag |= CS7;
            break;
        case 8:
            termios_p.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr, "Unsupported data size\n");
            return false;
        }

        //设置奇偶校验位double
        switch (serial_data_.parity)
        {
        case 'n':
        case 'N':
            termios_p.c_cflag &= ~PARENB;   /* Clear parity enable */
            termios_p.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            termios_p.c_cflag |= PARENB;     /* Enable parity */
            termios_p.c_cflag &= ~PARODD;   /* 转换为偶效验*/
            termios_p.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        case 'S':
        case 's':  /*as no parity*/
            termios_p.c_cflag &= ~PARENB;
            termios_p.c_cflag &= ~CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported parity\n");
            return false;
        }

        // 设置停止位
        switch (serial_data_.stopbits)
        {
        case 1:
            termios_p.c_cflag &= ~CSTOPB;
            break;
        case 2:
            termios_p.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr, "Unsupported stop bits\n");
            return false;
        }

        /* Set input parity option */
        if (serial_data_.parity != 'n')
            termios_p.c_iflag |= INPCK;

        tcflush(serial_data_.fd, TCIFLUSH); // 清除输入缓存区
        termios_p.c_cc[VTIME] = 150;        // 设置超时15 seconds
        termios_p.c_cc[VMIN] = 0;           //最小接收字符
        termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Input原始输入
        termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        termios_p.c_iflag &= ~(ICRNL | IGNCR);
        termios_p.c_oflag &= ~OPOST;   // Output禁用输出处理

        if (tcsetattr(serial_data_.fd, TCSANOW, &termios_p) != 0) /* Update the options and do it NOW */
        {
            perror("SetupSerial 3");
            return false;
        }
        return true;
    }
} // namespace serialport
