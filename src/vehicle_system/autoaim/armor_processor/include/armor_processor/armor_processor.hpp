/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2022-10-17 14:25:43
 * @FilePath: /tup_2023-10-16/src/vehicle_system/autoaim/armor_processor/include/armor_processor/armor_processor.hpp
 */
#include "global_user/include/coordsolver.hpp"
// #include "global_interface/msg/gimbal.hpp"
// #include "global_interface/msg/armor.hpp"
// #include "global_interface/msg/armors.hpp"
#include "global_interface/msg/target.hpp"

#include "../filter/particle_filter.hpp"

namespace autoaim
{
    class processor
    {

    public:
        processor();
        ~processor();

        //预测(接收armor_detector节点发布的目标信息进行预测)
        void predictor(global_interface::msg::Target& target_info);

        //参数配置
        void init_params();
        
    };
} //namespace autoaim