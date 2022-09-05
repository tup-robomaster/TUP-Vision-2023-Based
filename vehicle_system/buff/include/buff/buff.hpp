#include "params_common/include/params_common/params_common.hpp"

namespace buff
{
    class buff : robot_base::params_common
    {
    public:
        buff();
        virtual ~buff() = 0;
        virtual void buff_detector() = 0;
        virtual void prediction() = 0;
    };
};