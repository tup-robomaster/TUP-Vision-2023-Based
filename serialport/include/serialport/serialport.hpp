/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-06 01:59:06
 * @LastEditTime: 2022-09-11 01:18:37
 * @FilePath: /tup_2023/src/serialport/include/serialport/serialport.hpp
 */
#include "global_user/include/global_user/global_user.hpp"
#include "rmoss_master/rmoss_core/rmoss_base/include/rmoss_base/transporter_interface.hpp"

#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>

namespace serialport 
{
    class serialport : public rmoss_base::TransporterInterface
    {
    public:
        serialport();
        virtual ~serialport() = 0;

        bool open() override;
        void close() override;
        bool is_open() override;
        int read(void* buffer, size_t len) override;
        int write(const void* buffer, size_t len) override;
        std::string error_message() override;
    
    private:    
        std::string device_path;
        int fd;
        bool is_open;
        int speed;
        int databits;
        int stopbits;
        int parity;
    };
}; //serialport