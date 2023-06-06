/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2023-03-20 19:46:36
 * @LastEditTime: 2023-06-04 00:47:02
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/include/predictor/param_struct.hpp
 */
#ifndef PARAM_STRUCT_HPP_
#define PARAM_STRUCT_HPP_

namespace buff_processor
{
    struct PredInfo
    {
        double timestamp;
        double angle_offset;
    };

    struct BuffAngleInfo
    {
        // double dist;
        bool is_switched;
        double abs_angle;
        double relative_angle;
        double delta_angle;
        double angle_offset;
        int64_t timestamp;
    };

    //目标信息
    struct BuffInfo
    {   
        int buff_mode;
        bool target_switched;
        double abs_meas_angle;
        double abs_fitting_angle;
        double abs_pred_angle;
        Eigen::Vector3d armor3d_world;
        Eigen::Vector3d hit_point_world;
        Eigen::Vector3d armor3d_cam;
        Eigen::Vector3d hit_point_cam;
        Eigen::Vector2d angle;
        Eigen::Matrix3d rmat_imu;

        BuffInfo()
        {
            buff_mode = 3;
            target_switched = true;
            abs_meas_angle = 0.0;
            abs_fitting_angle = 0.0;
            abs_pred_angle = 0.0;

            armor3d_cam = {0.0, 0.0, 0.0};
            armor3d_world = {0.0, 0.0, 0.0};
            hit_point_cam = {0.0, 0.0, 0.0};
            hit_point_world = {0.0, 0.0, 0.0};
            angle = {0.0, 0.0};
        }
    };

    struct PredictStatus
    {
        bool xyz_status[3];
    };

    struct PredictorParam
    {
        string pf_path;
        double bullet_speed;            //弹速
        double shoot_delay;             //发弹延迟
        double delay_small;             //小符发弹延迟
        double delay_big;               //大符发弹延迟
        double delay_coeff;             //延迟系数
        
        double max_timespan;            //最大时间跨度，大于该时间重置预测器(ms)
        double max_rmse;                //TODO:回归函数最大Cost
        double max_v;                   //设置最大速度,单位rad/s
        double max_a;                   //设置最大角加速度,单位rad/s^2
        
        int history_deque_len_cos;      //大符全部参数拟合队列长度
        int history_deque_len_phase;    //大符相位参数拟合队列长度
        int history_deque_len_uniform;  //小符转速求解队列长度

        int window_size;                //滑动窗口大小
        double fan_length;              //能量机关旋转半径
        int max_error_cnt;              //预测误差帧数
        double pred_error_high_thresh;  //预测误差高阈值
        double pred_error_low_thresh;   //预测误差低阈值
        int fitting_error_cnt;          //拟合误差帧数
        double fitting_error_thresh;    //拟合误差阈值
        double rmse_high_thresh;        //拟合rmse高阈值
        double rmse_low_thresh;         //拟合rmse低阈值

        vector<double> params_bound;

        PredictorParam()
        {
            pf_path = "src/global_user/config/filter_param.yaml";
            bullet_speed = 28.0;
            shoot_delay = 100.0;
            delay_coeff = 1.0;
            delay_big = 200.0;
            delay_small = 150.0;

            max_timespan = 50000;       
            max_rmse = 2.0;
            max_v = 3.0;
            max_a = 8.0;
            
            history_deque_len_cos = 250;
            history_deque_len_phase = 100;
            history_deque_len_uniform = 100;
            delay_small = 175.0;
            delay_big = 100.0;
            
            window_size = 2;
            fan_length = 0.7;
            max_error_cnt = 5;
            pred_error_high_thresh = 0.35;
            pred_error_low_thresh = 0.20;
            fitting_error_cnt = 5;
            fitting_error_thresh = 0.20;
            rmse_high_thresh = 2.0;
            rmse_low_thresh = 0.5;
        }     
    };

    struct PathParam
    {
        string camera_param_path;
        string camera_name;
        PathParam()
        {
            camera_name = "KE0200110075";
            camera_param_path = "src/global_user/config/camera.yaml";
        }
    };

    struct DebugParam
    {
        bool show_img;
        bool show_marker;
        bool show_fitting_curve;
        DebugParam()
        {
            show_img = false;
            show_marker = false;
            show_fitting_curve = false;
        }
    };

    struct CURVE_FITTING_COST_PHASE
    {
        CURVE_FITTING_COST_PHASE (double x, double t, double a, double omega, double dc)
        : _x(x), _t(t), _a(a), _omega(omega), _dc(dc){}

        // 残差的计算
        template <typename T>
        bool operator()
        (
            const T* phase, // 模型参数，有1维
            // const T* const_term,
            T* residual     // 残差
        ) const 
        {
            // residual[0] = -(T (_a) / omega[0]) * ceres::cos(omega[0] * (T(_t) + phase[0] / omega[0])) + T(_dc) * T(_t) + (T (_a) / omega[0]) * ceres::cos(phase[0]) - T (_x);
            // residual[0] = -(T (_a) / T(_omega)) * ceres::cos(T(_omega) * T(_t) + phase[0]) + T(_dc) * T(_t) + const_term[0] - T (_x);
            residual[0] = -(T (_a) / T(_omega)) * ceres::cos(T(_omega) * T(_t) + phase[0]) + T(_dc) * T(_t) + (T (_a) / T(_omega)) * ceres::cos(phase[0]) - T (_x);
            return true;
        }
        const double _x, _t, _a, _omega, _dc;    // x,t数据
    };

    struct CurveFittingCost
    {
        const double _angle, _t;
        CurveFittingCost(double angle, double t)
        : _angle(angle), _t(t){}

        template<class T>
        bool operator()
        (
            const T* params,
            T* residual
        ) const
        {
            //f(t)=-(a/w)cos(wt+theta)+bt+(a/w)cos(theta)
            // residual[0] = -(params[0] / params[1]) * ceres::cos(params[1] * (T(_t) + params[2])) + params[3] * T(_t) + (params[0] / params[1]) * ceres::cos(params[1] * params[2]) - T(_angle);
            // residual[0] = -(params[0] / params[1]) * ceres::cos(params[1] * (T(_t) + params[2] / params[1])) + params[3] * T(_t) + (params[0] / params[1]) * ceres::cos(params[2]) - T(_angle);
            residual[0] = -(params[0] / params[1]) * ceres::cos(params[1] * T(_t) + params[2]) + params[3] * T(_t) + (params[0] / params[1]) * ceres::cos(params[2]) - T(_angle);
            return true;
        }
    };
} //namespace buff_processor
#endif