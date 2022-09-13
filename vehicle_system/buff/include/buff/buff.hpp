#include "global_user/include/global_user/global_user.hpp"

namespace buff
{
    class buff : global_user::global_user
    {
    public:
        buff();
        virtual ~buff() = 0;
        virtual void buff_detector() = 0;
        virtual void buff_prediction() = 0;
    };
};