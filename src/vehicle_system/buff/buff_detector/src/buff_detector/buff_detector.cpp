/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 15:56:01
 * @LastEditTime: 2022-12-22 23:15:06
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/src/buff_detector/buff_detector.cpp
 */
#include "../../include/buff_detector/buff_detector.hpp"

namespace buff_detector
{
    Detector::Detector()
    {
        lost_cnt = 0;
        is_last_target_exists = false;
        // input_size = {640,384};
        input_size = {640, 640};
        last_bullet_speed = 0;
    }

    Detector::Detector(const BuffParam& buff_param, const PathParam& path_param, const DebugParam& debug_param)
    : buff_param_(buff_param), path_param_(path_param), debug_param_(debug_param)
    {
        lost_cnt = 0;
        is_last_target_exists = false;
        input_size = {640, 640};
        last_bullet_speed = 0;
    }

    Detector::~Detector()
    {

    }

    bool Detector::run(TaskData& src, TargetInfo& target_info)
    {
        auto time_start = std::chrono::steady_clock::now();

        vector<BuffObject> objects;
        vector<Fan> fans;
        auto input = src.img;

        // TODO:放在节点类初始化时加载
        if(!is_initialized_)
        {
            buff_detector_.initModel(path_param_.network_path);
            coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            is_initialized_ = true;
        }

        if(!debug_param_.debug_without_com)
        {
            if (src.bullet_speed > 10)
            {
                double bullet_speed = 0.0;
                if (abs(src.bullet_speed - last_bullet_speed) < 0.5 || abs(src.bullet_speed - last_bullet_speed) > 1.5)
                {
                    bullet_speed = src.bullet_speed;
                    coordsolver_.setBulletSpeed(bullet_speed);
                    last_bullet_speed_ = bullet_speed;
                }
            }
        }

        if(debug_param_.using_imu)
            rmat_imu_ = src.quat.toRotationMatrix();
        else
            rmat_imu_= Eigen::Matrix3d::Identity();
        
        // TODO:修复ROI
        if(debug_param_.using_roi)
            roi_offset_ = cropImageByROI(input);

        if(debug_param_.assist_label)
        {
            auto img_name = path_param_.path_prefix + to_string(src.timestamp) + ".jpg";
            imwrite(img_name, input);
        }

        auto time_crop = std::chrono::steady_clock::now();

        if (!buff_detector_.detect(input, objects))
        {   //若未检测到目标
            lost_cnt_++;
            is_last_target_exists_ = false;
            last_target_area_ = 0;
            return false;
        }

        auto time_infer = std::chrono::steady_clock::now();

        // 创建扇叶对象
        for (auto object : objects)
        {
            if(debug_param_.detect_red)
                if (object.color != 1)
                    continue;
            if(!debug_param_.detect_red)
                if (object.color != 0)
                    continue;
            
            Fan fan;
            fan.id = object.cls;
            fan.color = object.color;
            fan.conf = object.prob;
            if (object.color == 0)
                fan.key = "B" + string(object.cls == 0 ? "Activated" : "Target");
            if (object.color == 1)
                fan.key = "R" + string(object.cls == 0 ? "Activated" : "Target");
            memcpy(fan.apex2d, object.apex, 5 * sizeof(cv::Point2f));
            for(int i = 0; i < 5; i++)
            {
                fan.apex2d[i] += Point2f((float)roi_offset.x, (float)roi_offset.y);
            }

            std::vector<Point2f> points_pic(fan.apex2d, fan.apex2d + 5);
            TargetType target_type = BUFF;

            // TODO:迭代法进行PnP解算
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu_, target_type, SOLVEPNP_ITERATIVE);
            // auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu_, target_type, SOLVEPNP_IPPE);

            fan.armor3d_cam = pnp_result.armor_cam;
            fan.armor3d_world = pnp_result.armor_world;
            fan.centerR3d_cam = pnp_result.R_cam;
            fan.centerR3d_world = pnp_result.R_world;

            fan.euler = pnp_result.euler;
            fan.rmat = pnp_result.rmat;

            fans.push_back(fan);
        }

        // 维护Tracker队列，删除过旧的Tracker
        if (trackers_.size() != 0)
        {
            for (auto iter = trackers_.begin(); iter != trackers_.end();)
            {
                //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                if ((src.timestamp - (*iter).last_timestamp) > buff_param_.max_delta_t)
                    next = trackers_.erase(iter);
                else
                    ++next;
                iter = next;
            }
        }

        // 分配或创建扇叶追踪器（fan tracker）
        // TODO:增加防抖
        std::vector<FanTracker> trackers_tmp;
        for (auto fan = fans.begin(); fan != fans.end(); ++fan)
        {
            if (trackers.size() == 0)
            {
                FanTracker fan_tracker((*fan), src.timestamp);
                trackers_tmp.push_back(fan_tracker);
            }
            else
            {
                double min_v = 1e9;
                int min_last_delta_t = 1e9;
                bool is_best_candidate_exist = false;
                std::vector<FanTracker>::iterator best_candidate;
                for (auto iter = trackers_.begin(); iter != trackers_.end(); iter++)
                {
                    double delta_t;
                    Eigen::AngleAxisd angle_axisd;
                    double rotate_speed;
                    double sign;
                    //----------------------------计算角度,求解转速----------------------------
                    // 若该扇叶完成初始化,且隔一帧时间较短
                    if ((*iter).is_initialized && (src.timestamp - (*iter).prev_timestamp) < buff_param_.max_delta_t)
                    {
                        delta_t = src.timestamp - (*iter).prev_timestamp;
                        // 目前扇叶到上一次扇叶的旋转矩阵
                        auto relative_rmat = (*iter).prev_fan.rmat.transpose() * (*fan).rmat;
                        angle_axisd = Eigen::AngleAxisd(relative_rmat);
                        auto rotate_axis_world = (*iter).last_fan.rmat * angle_axisd.axis();
                        // auto rotate_axis_world = (*fan).rmat * angle_axisd.axis();
                        // auto rotate_axis_world = (*iter).last_fan.rmat  * angle_axisd.axis();
                        sign = ((*fan).centerR3d_world.dot(rotate_axis_world) > 0 ) ? 1 : -1;
                    }
                    else
                    {
                        delta_t = src.timestamp - (*iter).last_timestamp;
                        // 目前扇叶到上一次扇叶的旋转矩阵
                        auto relative_rmat = (*iter).last_fan.rmat.transpose() * (*fan).rmat;
                        // TODO:使用点乘判断旋转方向
                        angle_axisd = Eigen::AngleAxisd(relative_rmat);
                        auto rotate_axis_world = (*fan).rmat * angle_axisd.axis();
                        sign = ((*fan).centerR3d_world.dot(rotate_axis_world) > 0 ) ? 1 : -1;
                    }
                    // 计算角速度(rad/s)
                    rotate_speed = sign * (angle_axisd.angle()) / (delta_t / 1e3);
                    if (abs(rotate_speed) <= min_v && abs(rotate_speed) <= buff_param_.max_v && (src.timestamp - (*iter).last_timestamp) <= min_last_delta_t)
                    {
                        min_last_delta_t = src.timestamp - (*iter).last_timestamp;
                        min_v = rotate_speed;
                        best_candidate = iter;
                        is_best_candidate_exist = true;
                    }
                    // if (fabs(rotate_speed) <= max_v)
                    // {
                    //     (*iter).update((*fan), src.timestamp);
                    //     (*iter).rotate_speed = rotate_speed;
                    //     break;
                    // }
                }
                if (is_best_candidate_exist)
                {
                    (*best_candidate).update((*fan), src.timestamp);
                    (*best_candidate).rotate_speed = min_v;
                }
                else
                {
                    FanTracker fan_tracker((*fan), src.timestamp);
                    trackers_tmp.push_back(fan_tracker);
                }
            }
        }
        for (auto new_tracker : trackers_tmp)
            trackers_.push_back(new_tracker);

        // 检查待激活扇叶是否存在
        Fan target;
        bool is_target_exists = chooseTarget(fans, target);

        // 若不存在待击打扇叶则返回false
        if (!is_target_exists)
        {
            if(debug_param_.show_all_fans)
            {
                for (auto fan : fans)
                {
                    putText(src.img, fmt::format("{:.2f}", fan.conf),fan.apex2d[4],FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
                    if (fan.color == 0)
                        putText(src.img, fmt::format("{}",fan.key), fan.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
                    if (fan.color == 1)
                        putText(src.img, fmt::format("{}",fan.key), fan.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
                    for(int i = 0; i < 5; i++)
                        line(src.img, fan.apex2d[i % 5], fan.apex2d[(i + 1) % 5], Scalar(0,255,0), 1);
                    auto fan_armor_center = coordsolver.reproject(fan.armor3d_cam);
                    circle(src.img, fan_armor_center, 4, {0, 0, 255}, 2);
                }
            }

            lost_cnt_++;
            is_last_target_exists_ = false;
            return false;
        }

        int avail_tracker_cnt = 0;
        double rotate_speed_sum = 0;
        double mean_rotate_speed = 0;
        Eigen::Vector3d r_center_sum = {0, 0, 0};
        Eigen::Vector3d mean_r_center = {0, 0, 0};

        // 计算平均转速与平均R字中心坐标
        for(auto tracker : trackers_)
        {
            if (tracker.is_last_fan_exists && tracker.last_timestamp == src.timestamp)
            {
                rotate_speed_sum += tracker.rotate_speed;
                r_center_sum += tracker.last_fan.centerR3d_world;
                avail_tracker_cnt++;
            }
        }

        // 若不存在可用的扇叶则返回false
        if (avail_tracker_cnt == 0)
        {
            lost_cnt_++;
            return false;
        }

        mean_rotate_speed = rotate_speed_sum / avail_tracker_cnt;
        mean_r_center = r_center_sum / avail_tracker_cnt;
        auto r_center_cam = coordsolver_.worldToCam(target.centerR3d_world, rmat_imu_);
        auto center2d_src = coordsolver_.reproject(r_center_cam);

        target_info.rotate_speed = mean_rotate_speed;
        target_info.r_center = mean_r_center;

        // 判断扇叶是否发生切换
        bool is_switched = false;
        double delta_t = src.timestamp - last_timestamp_;
        auto relative_rmat = last_fan_.rmat.transpose() * target.rmat;
        auto angle_axisd = Eigen::AngleAxisd(relative_rmat);

        double rotate_spd = (angle_axisd.angle() / delta_t * 1e3);
        if(abs(rotate_spd) > buff_param_.max_v)
            is_switched = true;
        target_info.target_switched = is_switched;

        lost_cnt_ = 0;
        last_roi_center_ = center2d_src;
        last_timestamp_ = src.timestamp;
        last_fan_ = target;
        is_last_target_exists_ = true;
        
        return true;
    }

    bool Detector::chooseTarget(std::vector<Fan> &fans, Fan &target)
    {
        float max_area = 0;
        int target_idx = 0;
        int target_fan_cnt = 0;
        for (auto fan : fans)
        {
            if (fan.id == 1)
            {
                target = fan;
                target_fan_cnt++;
            }
        }
        if (target_fan_cnt == 1)
            return true;
        else
            return false;
    }

    cv::Point2i Detector::cropImageByROI(cv::Mat& img)
    {
        if (!is_last_target_exists)
        {
            // 当丢失目标帧数过多或lost_cnt为初值
            if (lost_cnt > buff_param_.max_lost_cnt || lost_cnt == 0)
            {
                return Point2i(0,0);
            }
        }

        // 若目标大小大于阈值
        if ((last_target_area / img.size().area()) > buff_param_.no_crop_thres)
        {
            return Point2i(0,0);
        }
        
        // 处理X越界
        if (last_roi_center.x <= input_size.width / 2)
            last_roi_center.x = input_size.width / 2;
        else if (last_roi_center.x > (img.size().width - input_size.width / 2))
            last_roi_center.x = img.size().width - input_size.width / 2;
        // 处理Y越界
        if (last_roi_center.y <= input_size.height / 2)
            last_roi_center.y = input_size.height / 2;
        else if (last_roi_center.y > (img.size().height - input_size.height / 2))
            last_roi_center.y = img.size().height - input_size.height / 2;

        // 左上角顶点
        auto offset = last_roi_center - Point2i(input_size.width / 2, input_size.height / 2);
        Rect roi_rect = Rect(offset, input_size);
        img(roi_rect).copyTo(img);

        return offset;
    }

    void Detector::setDetectorParam(double& param, int idx)
    {
        switch (idx)
        {
        case 1:
            buff_param_.fan_length = param;
            break;
        case 2:
            buff_param_.max_delta_t = param;
            break;
        case 3:
            buff_param_.max_lost_cnt = param;
            break;
        case 4:
            buff_param_.max_v = param;
            break;
        case 5:
            buff_param_.no_crop_thres = param;
            break;
        default:
            break;
        }
    }

    void Detector::setDebugParam(bool& param, int idx)
    {
        switch (idx)
        {
        case 1:
            debug_param_.assist_label = param;
            break;
        case 2:
            debug_param_.detect_red = param;
            break;
        case 3:
            debug_param_.prinf_latency = param;
            break;
        case 4:
            debug_param_.print_target_info = param;
            break;
        case 5:
            debug_param_.show_all_fans = param;
            break;
        case 6:
            debug_param_.show_fps = param;
            break;
        case 7:
            debug_param_.using_imu = param;
            break;
        case 8:
            debug_param_.using_roi = param;
            break;
        default:
            break;
        }
    }

} // namespace buff_detector