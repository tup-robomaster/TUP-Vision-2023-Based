/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-18 16:03:31
 * @LastEditTime: 2022-11-26 16:02:33
 * @FilePath: /TUP-Vision-2023-Based/src/serialport/src/serialport/serialport.cpp
 */
#include "../../include/serialport/serialport.hpp"

namespace serialport
{
    serialport::serialport()
    {
        
    }

    serialport::serialport(const std::string id, const int baud, bool debug_without_com) : serial_id(id), device_baud(baud),debug_without_com_(debug_without_com)
    {
        if(debug_without_com_)
        {
            init();
        }
    }

    serialport::~serialport()
    {

    }

    void serialport::init()
    {
        device = select_id(list_ports());
        const std::string alias = "/dev/" + device.alias;
        
        fd = ::open(alias.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1)
        {
            is_open = false;
            perror(alias.c_str());
            return;
        }
        ::close(last_fd);

        printf("Serial port is opening...\n");

        speed = device_baud;
        databits = 8;
        stopbits = 1;
        parity = 'N';

        if(!set_brate())
        {
            printf("Set baud error!\n");
            return;
        }

        if(!set_bit())
        {
            printf("Set parity error!\n");
            return;
        }

        printf("Open successed!\n");

        last_fd = fd;
        is_open = true;
    }

    bool serialport::open(const std::string id, const int baud)
    {
        // this->serial_id = id;
        // this->device_baud = baud;
        init();
        return is_open;
    }
   
    void serialport::close()
    {

    }

    void serialport::read(std::vector<uint8_t>& buffer)
    {
        name = ttyname(fd);
        // FD_ERROR(name, "tty is null!\n");

        result = ioctl(fd, FIONREAD, &bytes);
        if(result == -1 || !bytes)
            // return false;
        
        bytes = ::read(fd, &buffer, sizeof(buffer));
        if(buffer.empty())
        {
            std::cout << "Buffer is empty..." << std::endl;
        }
        else
        {
            for(auto& info : buffer)
            {
                std::cout << "data:" << info << std::endl;
            }
        }
    }

    void serialport::write(std::vector<uint8_t>& buffer)
    {
        auto send_ = ::write(fd, &buffer, sizeof(buffer));
        if(send_ == -1)
        {
            printf("send data failed!");
            // return false;
        }
    }

    std::string serialport::error_message()
    {
        return std::string();
    }

    std::vector<Device> serialport::list_ports()
    {
        std::vector<Device> devices;
        for(auto port_dir : DEFAULT_PORT)
        {
            std::vector<std::string> availible_path;
            auto general_path = "/sys/class/tty/"+ port_dir;
            for(int i = 0; i < MAX_ITER;i++)
            {
                auto tty_dir_path = general_path + std::to_string(i);

                if (access(tty_dir_path.c_str(),F_OK) != -1)
                {
                    Device dev;
                    // availible_path.push_back(tty_dir_path);
                    auto real_path = global_user::symbolicToReal(tty_dir_path);
                    std::string info_path;
                    //需注意ttyACM与ttyUSB的uevent文件实际深度不同
                    if (port_dir == "ttyACM")
                        info_path = global_user::getParent(global_user::getParent(global_user::getParent(real_path)));
                    else if (port_dir == "ttyUSB")
                        info_path = global_user::getParent(global_user::getParent(global_user::getParent(global_user::getParent(real_path))));
                    dev = getDeviceInfo(info_path);
                    dev.alias = port_dir + std::to_string(i);
                    dev.path = tty_dir_path;
                    devices.push_back(dev);
                }
            } 
        }
        return devices;
    }

    Device serialport::getDeviceInfo(std::string path)
    {
        Device dev;
        auto uevent_path = path + "/uevent";
        auto texts = global_user::readLines(uevent_path);
        for(auto text : texts)
        {
            int equal_idx = text.find("=");
            std::string config_type = text.substr(0, equal_idx);
            std::string config_info = text.substr(equal_idx + 1);

            if(config_type == "PRODUCT")
            {
                dev.id = config_info;
            }
        }

        return dev;
    }


    Device serialport::select_id(std::vector<Device> devices)
    {
        for(auto &dev : devices)
        {
            if(dev.id == serial_id)
            {
                return dev;
            }
        }
        return Device();
    }
    
    //TODO: virtual com port
    bool serialport::debug_without_port()
    {
        is_open = true;
        return is_open;
    }

    bool serialport::set_brate()
    {
        int speed_arr[] = {B921600, B460800, B230400, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
        int name_arr[] = {921600, 460800, 230400,115200, 38400, 19200, 9600, 4800, 2400, 1200,  300};
        
        struct termios opt;
        tcgetattr(fd, &opt);
        
        for(int ii = 0; ii < 11; ii++)
        {
            if(speed == name_arr[ii])
            {
                tcflush(fd, TCIOFLUSH); //清空缓冲区的内容
                cfsetispeed(&opt, speed_arr[ii]); //设置接受的波特率
                cfsetospeed(&opt, speed_arr[ii]); //设置发送的波特率
                // int status = 
                tcsetattr(fd, TCSANOW, &opt); //使设置立即生效

                // FD_ERROR(status, "tcsetattr fd error!");
            }
        }

        return true;
    }

    int serialport::set_bit()
    {
        /**
         *@brief   设置串口数据位，停止位和效验位
        */
        struct termios opt;
        // int status = 
        tcgetattr(fd, &opt);
        // FD_ERROR(status, "serial setup error!");

        opt.c_cflag |= (CLOCAL | CREAD); //接受数据
        opt.c_cflag &= ~CSIZE;           //设置数据位数

        switch(databits)
        {
            case 7:
                opt.c_cflag |= CS7;
                break;
            case 8:
                opt.c_cflag |= CS8;
                break;
            default:
                fprintf(stderr, "Unsupported data size\n");
                return false;
        }

        //设置奇偶校验位double
        switch (parity)
        {
        case 'n':
        case 'N':
            opt.c_cflag &= ~PARENB;   /* Clear parity enable */
            opt.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;

        case 'o':
        case 'O':
            opt.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            opt.c_iflag |= INPCK;             /* Disnable parity checking */
            break;

        case 'e':
        case 'E':
            opt.c_cflag |= PARENB;     /* Enable parity */
            opt.c_cflag &= ~PARODD;   /* 转换为偶效验*/
            opt.c_iflag |= INPCK;       /* Disnable parity checking */
            break;

        case 'S':
        case 's':  /*as no parity*/
            opt.c_cflag &= ~PARENB;
            opt.c_cflag &= ~CSTOPB;
            break;

        default:
            fprintf(stderr, "Unsupported parity\n");
            return false;
        }

        switch(stopbits)
        {
            case 1:
                opt.c_cflag &= ~CSTOPB;
                break;
            case 2:
                opt.c_cflag |= CSTOPB;
                break;
            default:
                fprintf(stderr, "Unsupported stop bits\n");
		        return false;
        }

        /* Set input parity option */
        if (parity != 'n')
            opt.c_iflag |= INPCK;

        tcflush(fd, TCIFLUSH); //清除输入缓存区
        opt.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
        opt.c_cc[VMIN] = 0;  //最小接收字符
        opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input原始输入*/
        opt.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        opt.c_iflag &= ~(ICRNL | IGNCR);
        opt.c_oflag &= ~OPOST;   /*Output禁用输出处理*/

        // status = 
        tcsetattr(fd, TCSANOW, &opt); /* Update the options and do it NOW */
        // FD_ERROR(status, "serial setup error!");
        
        return true;
    }

    bool serialport::get_quat(std::vector<uint8_t>& data)
    {
        return false;
    }

    bool serialport::get_gyro(std::vector<uint8_t>& data)
    {
        return false;
    }
    
    bool serialport::get_acc(std::vector<uint8_t>& data)
    {
        return false;
    }

} //serialport
