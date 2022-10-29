/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-17 00:27:33
 * @LastEditTime: 2022-10-25 23:46:51
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/include/armor_processor/armor_processor.hpp
 */
#include "global_user/include/coordsolver.hpp"
#include "global_interface/msg/target.hpp"
// #include "global_interface/msg/gimbal.hpp"
// #include "global_interface/msg/armor.hpp"
// #include "global_interface/msg/armors.hpp"

#include "../prediction/prediction.hpp"

typedef geometry_msgs::msg::Point GeometryPoint;

namespace armor_processor
{
    class Processor
    {
    public:
        Processor(const PredictParam& predict_param, DebugParam& debug_param, std::string coord_file);
        ~Processor();

        //预测(接收armor_detector节点发布的目标信息进行预测)
        void predictor(global_interface::msg::Target& target_info);

    private:
        Eigen::Vector3d aiming_point_;
        // GeometryPoint aiming_point_;

    public:
        ArmorPredictor armor_predictor_;
        coordsolver::coordsolver coordsolver_;

        Eigen::Matrix3d rmat_imu;
    };
} //namespace armor_processor