/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-09-11 01:22:07
 * @LastEditTime: 2022-09-11 02:21:56
 * @FilePath: /tup_2023/src/serialport/include/can_port/can_port.hpp
 */
#include <linux/can/raw.h>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <queue>
#include <memory>
#include <vector>
#include <termio.h>
#include <unistd.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <net/if.h>

#include "global_user/include/global_user/global_user.hpp"
#include "rmoss_master/rmoss_core/rmoss_base/include/rmoss_base/transporter_interface.hpp"

namespace can_port
{
    class can_port : public rmoss_base::TransporterInterface
    {
    public:
        can_port();
        virtual ~can_port() = 0;

        bool open() override;
        void close() override;
        bool is_open() override;
        int read(void* buffer, size_t len) override;
        int write(const void* buffer, size_t len) override;
        std::string error_message() override;

    private:
        int can_sock{};
        sockaddr_can can_addr{}; 
        iovec can_iov{};
        ifreq can_ifr{};

        canfd_frame can_send_frame{};
        canfd_frame can_read_frame{};

        int parse_data(void* buffer, canfd_frame&);
        int parse_can_frame(void* buffer, canfd_frame&);
    };  
}; //can_port