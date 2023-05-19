/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-20 15:56:01
 * @LastEditTime: 2023-03-20 12:16:22
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/test/src/buff_detector/buff_detector.cpp
 */
#include "../../include/buff_detector/buff_detector.hpp"

namespace buff_detector
{
    Detector::Detector()
    : logger_(rclcpp::get_logger("buff_detector"))
    {
        is_initialized_ = false;
        lost_cnt_ = 0;
        is_last_target_exists_ = false;
        // input_size_ = {640,384};
        input_size_ = {640, 640};
        last_bullet_speed_ = 0;
        last_angle_ = 0.0;
    }

    Detector::Detector(const BuffParam& buff_param, const PathParam& path_param, const DebugParam& debug_param)
    : buff_param_(buff_param), path_param_(path_param), debug_param_(debug_param),
    logger_(rclcpp::get_logger("buff_detector"))
    {
        is_initialized_ = false;
        lost_cnt_ = 0;
        is_last_target_exists_ = false;
        input_size_ = {640, 640};
        last_bullet_speed_ = 0;
        last_angle_ = 0.0;
    }

    Detector::~Detector()
    {

    }

    bool Detector::run(TaskData& src, TargetInfo& target_info)
    {
        auto time_start = steady_clock_.now();

        vector<BuffObject> objects;
        // vector<Fan> fans_;
        auto input = src.img;

        // TODO:放在节点类初始化时加载
        if(!is_initialized_)
        {
            buff_detector_.initModel(path_param_.network_path);
            coordsolver_.loadParam(path_param_.camera_param_path, path_param_.camera_name);
            is_initialized_ = true;
        }

        if (!debug_param_.using_imu && src.bullet_speed > 10)
        {
            double bullet_speed = 0.0;
            if (abs(src.bullet_speed - last_bullet_speed_) < 0.5 || abs(src.bullet_speed - last_bullet_speed_) > 1.5)
            {
                bullet_speed = src.bullet_speed;
                coordsolver_.setBulletSpeed(bullet_speed);
                last_bullet_speed_ = bullet_speed;
                target_info.bullet_speed = bullet_speed;
                RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "bullet speed: %.2fm/s", bullet_speed);
            }
        }

        if (debug_param_.using_imu)
        {
            rmat_imu_ = src.quat.toRotationMatrix();
            RCLCPP_INFO_THROTTLE(logger_, this->steady_clock_, 500, "Using imu...");
        }
        else
        {
            rmat_imu_= Eigen::Matrix3d::Identity();
            RCLCPP_WARN_THROTTLE(logger_, this->steady_clock_, 500, "No imu...");
        }
        
        // TODO:修复ROI
        if (debug_param_.using_roi)
        {
            roi_offset_ = cropImageByROI(input);
            RCLCPP_INFO_ONCE(logger_, "Using roi...");
        }

        if (debug_param_.assist_label)
        {
            auto img_name = path_param_.path_prefix + to_string(src.timestamp) + ".jpg";
            imwrite(img_name, input);
            RCLCPP_INFO_ONCE(logger_, "Auto label...");
        }

        auto time_crop = steady_clock_.now();
        // objects.clear();
        fans_.clear();
        if (!buff_detector_.detect(input, objects))
        {   //若未检测到目标
            lost_cnt_++;
            is_last_target_exists_ = false;
            last_target_area_ = 0;
            target_info.find_target = false;
            RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "No buff target is detected...");
            return false;
        }
        auto time_infer = steady_clock_.now();

        vector<cv::Point2f> center_vec;
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

            if (object.color == 1)
                fan.key = "R" + string(object.cls == 1 ? "Target" : "Activated");
            if (object.color == 0)
                fan.key = "B" + string(object.cls == 1 ? "Target" : "Activated");

            memcpy(fan.apex2d, object.apex, 5 * sizeof(cv::Point2f));
            for(int i = 0; i < 5; i++)
            {
                fan.apex2d[i] += Point2f((float)roi_offset_.x, (float)roi_offset_.y);
            }
            
            std::vector<Point2f> points_pic(fan.apex2d, fan.apex2d + 5);
            std::vector<cv::Point2f> points_rect;
            cv::Point2f center_r;
            for(int ii = 0; ii < 5; ii++)
            {
                if(ii == 0)
                    center_r = fan.apex2d[ii];
                else
                    points_rect.push_back(fan.apex2d[ii]);
            }
            center_vec.push_back(center_r);
            cv::RotatedRect r_rect = cv::minAreaRect(points_pic);
            // cv::RotatedRect armor_rect = cv::minAreaRect(points_rect);
            // if(debug_param_.show_img)
            // {
            //     // last_angle_ = cur_angle_;
            //     cv::Point2f vertex[4];
            //     r_rect.points(vertex);
            //     cv::Point2f vertex_sum;
            //     for(int ii = 0; ii < 4; ii++)
            //     {
            //         vertex_sum += vertex[ii];
            //         cv::line(src.img, vertex[ii % 4], vertex[(ii+1)%4], {255, 0, 255}, 4);
            //     }
            //     char ch[20];
            //     sprintf(ch, "%.2f", r_rect.angle);
            //     std::string angle_str = ch;
            //     putText(src.img, "Angle:" + angle_str, (vertex_sum / 4.0), FONT_HERSHEY_TRIPLEX, 1, {255, 255, 0});
            // }
            
            // TODO:迭代法进行PnP解算
            TargetType target_type = BUFF;
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu_, target_type, SOLVEPNP_ITERATIVE);
            // auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu_, target_type, SOLVEPNP_IPPE);

            fan.armor3d_cam = pnp_result.armor_cam;
            fan.armor3d_world = pnp_result.armor_world;
            fan.centerR3d_cam = pnp_result.R_cam;
            fan.centerR3d_world = pnp_result.R_world;
            fan.euler = pnp_result.euler;
            fan.rmat = pnp_result.rmat;

            // Eigen::Vector2d center_yaw_pitch;
            // center_yaw_pitch[0] = fan.centerR3d_world[1] / fan.centerR3d_world[0];
            // center_yaw_pitch[1] = fan.centerR3d_world[2] / fan.centerR3d_world[0];
            // if(centerR3d_vec_.size() < 50)
            // {
            //     centerR3d_vec_.push_back(fan.centerR3d_world);
            //     center_yaw_pitch_vec_.push_back(center_yaw_pitch);
            // }
            // else
            // {
            //     centerR3d_vec_.pop_front();
            //     centerR3d_vec_.push_back(fan.centerR3d_world);
            //     center_yaw_pitch_vec_.pop_front();
            //     center_yaw_pitch_vec_.push_back(center_yaw_pitch);
            // }

            // Eigen::Vector2d yaw_pitch;
            // yaw_pitch[0] = fan.armor3d_world[1] / fan.armor3d_world[0];
            // yaw_pitch[1] = fan.armor3d_world[2] / fan.armor3d_world[0];
            // if(armor3d_vec_.size() < 50)
            // {
            //     armor3d_vec_.push_back(fan.armor3d_world);
            //     yaw_pitch_vec_.push_back(yaw_pitch);
            // }
            // else
            // {
            //     armor3d_vec_.pop_front();
            //     armor3d_vec_.push_back(fan.armor3d_world);
            //     yaw_pitch_vec_.pop_front();
            //     yaw_pitch_vec_.push_back(yaw_pitch);
            // }

            cv::Point2f r_aver;
            cv::Point2f r_sum;
            cv::Point2f tar_center;
            cv::Point2f tar_sum;
            // for(auto center : center_vec)
            //     r_sum += center;
            // r_aver = (r_sum / (float)(center_vec.size()));

            // last_angle_ = cur_angle_;
            // std::vector<cv::Point2f> tar_pic;
            // for(int ii = 0; ii < 5; ii++)
            // {
            //     // tar_pic.push_back(fan.apex2d[ii]);
            //     if(ii != 2)
            //         tar_sum += fan.apex2d[ii];
            //     else
            //         r_aver = fan.apex2d[ii];
            // }
            // tar_center = (tar_sum / 4.0);
            
            // double dx = (tar_center.x - r_aver.x);
            // double dy = -(tar_center.y - r_aver.y);
            // double r_dis = sqrt(pow(dy, 2) + pow(dx, 2));
            // double sin_theta = asin(dy / r_dis);
            // fan.dx = dx;
            // fan.dz = dy;
            // fan.angle = abs(sin_theta);
            // cur_angle_ = atan2((tar_center.y - r_aver.y), (tar_center.x - r_aver.x)) * (180 / CV_PI);

            double dz = (fan.armor3d_world[2] - fan.centerR3d_world[2]);
            double dx = (fan.armor3d_world[1] - fan.centerR3d_world[1]);
            double dy = (fan.armor3d_world[0] - fan.centerR3d_world[0]);
            double r_dis = sqrt(pow(dy, 2) + pow(dx, 2) + pow(dz, 2));
            // double sin_theta = asin(dz / r_dis) * (180 / CV_PI);
            double sin_theta = asin(dz / r_dis);
            // RCLCPP_INFO(logger_, "R_radius: %lf theta: %lf", r_dis, sin_theta);
            fan.dx = dx;
            fan.dz = dz;
            fan.angle = abs(sin_theta);
            fans_.push_back(fan);
        }

        // 维护Tracker队列，删除过旧的Tracker
        if (trackers_.size() != 0)
        {
            for (auto iter = trackers_.begin(); iter != trackers_.end();)
            {
                //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                if (((src.timestamp - (*iter).last_timestamp) / 1e6) > buff_param_.max_delta_t)
                    next = trackers_.erase(iter);
                else
                    ++next;
                iter = next;
            }
        }

        //求出中心R标的均值
        // Eigen::Vector3d centerR_sum = {0, 0, 0};
        // Eigen::Vector3d centerR_ave = {0, 0, 0};
        // deque<Eigen::Vector3d> ratio_vec;
        // if((int)centerR3d_vec_.size() == 50)
        // {
        //     for(auto &centerR : centerR3d_vec_)
        //         centerR_sum += centerR;
        //     centerR_ave = centerR_sum / (int)centerR3d_vec_.size();
        //     // RCLCPP_INFO(logger_, "r_center_ave: %lf %lf %lf", centerR_ave[0], centerR_ave[1], centerR_ave[2]);
            
        //     // if((int)armor3d_vec_.size() == 10)
        //     // {
        //     //     double yaw_ave = 0;
        //     //     double yaw_sum = 0;
        //     //     for(auto armor3d : armor3d_vec_)
        //     //     {
        //     //         double yaw_angle = (atan2(abs(armor3d[0] - centerR_ave[0]), abs(armor3d[1] - centerR_ave[1]))) * (180 / CV_PI);
        //     //         // RCLCPP_INFO(logger_, "yaw_angle: %lf", yaw_angle);
        //     //         yaw_sum += yaw_angle;
        //     //     }
        //     //     yaw_ave = yaw_sum / (int)armor3d_vec_.size();
        //     //     // RCLCPP_INFO(logger_, "yaw_angle_ave: %lf", yaw_ave);
        //     // }
            
        //     for(auto &centerR : centerR3d_vec_)
        //     {
        //         Eigen::Vector3d ratio;
        //         ratio[0] = centerR_ave[0] / centerR[0];
        //         ratio[1] = centerR_ave[1] / centerR[1];
        //         ratio[2] = centerR_ave[2] / centerR[2];
        //         ratio_vec.push_back(ratio);
        //     }

        //     int ii = 0;
        //     for(auto &armor3d : armor3d_vec_)
        //     {
        //         armor3d[0] = ratio_vec[ii][0] * armor3d[0];
        //         armor3d[1] = ratio_vec[ii][1] * armor3d[1];
        //         armor3d[2] = ratio_vec[ii][2] * armor3d[2];
        //         ii++;
        //     }

        //     // int jj = 0;
        //     // for(auto& fan : fans_)
        //     // {
        //     //     int idx = (int)centerR3d_vec_.size() - (int)fans_.size() - 1 + jj;
        //     //     // fan.centerR3d_world = centerR_ave;
        //     //     fan.centerR3d_world[0] = ratio_vec[idx][0] * fan.centerR3d_world[0];
        //     //     fan.centerR3d_world[1] = center_yaw_pitch_vec_[idx][0] * fan.centerR3d_world[0];
        //     //     fan.centerR3d_world[2] = center_yaw_pitch_vec_[idx][1] * fan.centerR3d_world[0];
        //     //     fan.armor3d_world[0] = ratio_vec[idx][0] * fan.armor3d_world[0];
        //     //     fan.armor3d_world[1] = yaw_pitch_vec_[idx][0] * fan.armor3d_world[0];
        //     //     fan.armor3d_world[2] = yaw_pitch_vec_[idx][1] * fan.armor3d_world[0];
        //     //     // RCLCPP_INFO(logger_, "armor3d: %lf %lf %lf", fan.centerR3d_world[0], fan.centerR3d_world[1], fan.centerR3d_world[2]);
        //     //     jj++;
        //     // }
        //     // std::cout << std::endl;
        // }

        // 分配或创建扇叶追踪器（fan tracker）
        // TODO:增加防抖
        std::vector<FanTracker> trackers_tmp;
        for (auto fan = fans_.begin(); fan != fans_.end(); ++fan)
        {
            if (trackers_.size() == 0)
            {
                FanTracker fan_tracker((*fan), src.timestamp);
                trackers_tmp.push_back(fan_tracker);
            }
            else
            {
                double min_angle = 1e9;
                double min_last_delta_t = 1e9;
                bool is_best_candidate_exist = false;
                std::vector<FanTracker>::iterator best_candidate;
                // double min_last_dt = 1e9;
                // double min_rotation_angle = 1e9;
                // bool is_best_candidate_exist1 = false;
                // std::vector<FanTracker>::iterator best_candidate1;
                for (auto iter = trackers_.begin(); iter != trackers_.end(); iter++)
                {
                    double delta_t;
                    Eigen::AngleAxisd angle_axisd;
                    double delta_angle;
                    double rotation_angle;
                    int sign = 0;
                    //----------------------------计算角度,求解转速----------------------------
                    // 若该扇叶完成初始化,且隔一帧时间较短
                    if ((*iter).is_initialized && (src.timestamp - (*iter).prev_timestamp) / 1e6 < buff_param_.max_delta_t)
                    {
                        delta_t = src.timestamp - (*iter).prev_timestamp;
                        bool flag = (((*fan).dz / (*iter).prev_fan.dz) > 0 && (*fan).dx / (*iter).prev_fan.dx > 0) ? 1 : 0; 
                        if (!flag)
                            continue;

                        // 当前扇叶与上一帧扇叶的角度差值
                        delta_angle = (*fan).angle - (*iter).prev_fan.angle;
                        if (((*fan).dx > 0 && (*fan).dz > 0) || ((*fan).dx < 0 && (*fan).dz < 0))
                        {
                            if (delta_angle < 0)
                                delta_angle = abs(delta_angle);
                            else if (delta_angle > 0)
                                delta_angle = -abs(delta_angle);
                        }

                        // 目前扇叶到上一次扇叶的旋转矩阵
                        auto relative_rmat = (*iter).prev_fan.rmat.transpose() * (*fan).rmat;
                        angle_axisd = Eigen::AngleAxisd(relative_rmat);
                    }
                    else
                    {
                        delta_t = src.timestamp - (*iter).last_timestamp;
                        bool flag = (((*fan).dz / (*iter).last_fan.dz) > 0 && (*fan).dx / (*iter).last_fan.dx > 0) ? 1 : 0; 
                        if (!flag)
                            continue;

                        // 目前扇叶到上一次扇叶的旋转矩阵
                        delta_angle = (*fan).angle - (*iter).last_fan.angle;
                        if (((*fan).dx > 0 && (*fan).dz > 0) || ((*fan).dx < 0 && (*fan).dz < 0))
                        {
                            if (delta_angle < 0)
                                delta_angle = abs(delta_angle);
                            else if (delta_angle > 0)
                                delta_angle = -abs(delta_angle);
                        }

                        auto relative_rmat = (*iter).last_fan.rmat.transpose() * (*fan).rmat;
                        angle_axisd = Eigen::AngleAxisd(relative_rmat);
                    }

                    delta_t = ((src.timestamp - (*iter).last_timestamp) / 1e6);
                    // rotation_angle = angle_axisd.angle();
                    // RCLCPP_INFO(logger_, "Rotation angle:%lf", rotation_angle);
                    if (abs(delta_angle) <= abs(min_angle) && abs(delta_angle) < buff_param_.max_angle && delta_t <= min_last_delta_t)
                    {
                        min_last_delta_t = delta_t;
                        min_angle = delta_angle;
                        best_candidate = iter;
                        is_best_candidate_exist = true;
                    }
                    
                    // if(abs(rotation_angle) <= abs(min_rotation_angle) && abs(rotation_angle) < buff_param_.max_angle && delta_t <= min_last_dt)
                    // {
                    //     min_last_dt = delta_t;
                    //     min_rotation_angle = rotation_angle;
                    //     best_candidate1 = iter;
                    //     is_best_candidate_exist1 = true;
                    // }
                }
                // if(is_best_candidate_exist1)
                // {
                //     RCLCPP_INFO_THROTTLE(logger_, "Rotation angle:%lf", min_rotation_angle * (180 / CV_PI));
                // }
                if (is_best_candidate_exist)
                {
                    (*best_candidate).update((*fan), src.timestamp);
                    (*best_candidate).delta_angle = min_angle;
                    delta_angle_vec_.push_back(min_angle);
                    // RCLCPP_INFO(logger_, "delta angle:%lf", min_angle * (180 / CV_PI));
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
        bool is_target_exists = chooseTarget(fans_, target);
        // 若不存在待击打扇叶则返回false
        if (!is_target_exists)
        {
            if (debug_param_.show_all_fans)
            {
                RCLCPP_DEBUG_ONCE(logger_, "Show all fans...");
                showFans(src);
            }

            lost_cnt_++;
            is_last_target_exists_ = false;
            target_info.find_target = false;
            RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "No active target...");
            return false;
        }
        else
        {
            // cv::Point2f r_aver;
            // cv::Point2f r_sum;
            // cv::Point2f tar_center;
            // cv::Point2f tar_sum;
            // for(auto center : center_vec)
            //     r_sum += center;
            // r_aver = (r_sum / (float)(center_vec.size()));

            // last_angle_ = cur_angle_;
            // std::vector<cv::Point2f> tar_pic;
            // for(int ii = 0; ii < 5; ii++)
            // {
            //     tar_pic.push_back(target.apex2d[ii]);
            //     if(ii != 2)
            //         tar_sum += target.apex2d[ii];
            // }
            // tar_center = (tar_sum / 4.0);
            // cur_angle_ = atan2((tar_center.y - r_aver.y), (tar_center.x - r_aver.x)) * (180 / CV_PI);
            
            // double dz = (target.armor3d_world[2] - target.centerR3d_world[2]);
            // double dx = (target.armor3d_world[1] - target.centerR3d_world[1]);
            // double dy = (target.armor3d_world[0] - target.centerR3d_world[0]);
            // double r_dis = sqrt(pow(dy, 2) + pow(dx, 2) + pow(dz, 2));
            // double sin_theta = asin(dz / r_dis) * (180 / CV_PI);
            // RCLCPP_INFO(logger_, "R_radius: %lf theta: %lf", r_dis, sin_theta);

            //角度连续化处理
            // double final_angle = normalizeAngle(sin_theta, dz);

            //角度融合

            // if((int)centerR3d_vec_.size() == 50)
            // {
            //     // double cos_theta = acos(sqrt(pow(dx, 2) + pow(dy, 2)) / r_dis) * (180 / CV_PI);
            //     // double tan_theta = atan(dz / sqrt(pow(dy, 2) + pow(dx, 2))) * (180 / CV_PI);
            //     // double yaw_angle = (atan2((target.armor3d_world[0] - target.centerR3d_world[0]), (target.armor3d_world[1] - target.centerR3d_world[1]))) * (180 / CV_PI);
            //     // double angle = (atan2(abs(target.armor3d_world[2] - centerR_ave[2]), abs(target.armor3d_world[1] - centerR_ave[1]))) * (180 / CV_PI);
            //     // RCLCPP_INFO(logger_, "angle: %lf", angle);
            //     // RCLCPP_INFO(logger_, "target3d: %lf %lf %lf", target.armor3d_world[0], target.armor3d_world[1], target.armor3d_world[2]);
            //     std::cout << std::endl;
            // }
            // RCLCPP_INFO(logger_, "r_center: %lf %lf %lf", fan.centerR3d_cam[0], fan.centerR3d_cam[1], fan.centerR3d_cam[2]);

            // cv::RotatedRect tar_rect = cv::fitEllipse(tar_pic);
            // cur_angle_ = tar_rect.angle;
            // if(last_angle_ != 0)
            //     RCLCPP_INFO(logger_, "last_angle: %f cur_angle: %f delta_angle: %f", last_angle_, cur_angle_, (cur_angle_ - last_angle_));
        }

        int avail_tracker_cnt = 0;
        double delta_angle_sum = 0.0;
        double mean_delta_angle = 0.0; //TODO:先剔除掉错误数据，再对当前待激活扇叶的角度变化进行插值或融合已激活扇叶的角度变化
        Eigen::Vector3d r_center_sum = {0, 0, 0};
        Eigen::Vector3d mean_r_center = {0, 0, 0};

        // 计算平均转速与平均R字中心坐标
        for (auto tracker : trackers_)
        {
            if (tracker.is_last_fan_exists && tracker.last_timestamp == src.timestamp)
            {
                delta_angle_sum += tracker.delta_angle;
                r_center_sum += tracker.last_fan.centerR3d_world;
                avail_tracker_cnt++;
            }
        }

        // 若不存在可用的扇叶则返回false
        if (avail_tracker_cnt == 0)
        {
            if (debug_param_.show_all_fans)
            {
                showFans(src);
            }
            lost_cnt_++;
            target_info.find_target = false;
            return false;
        }

        //FIXME:用角度变化量的均值对当前待激活扇叶的角度变化进行校正
        mean_delta_angle = delta_angle_sum / avail_tracker_cnt;
        mean_r_center = r_center_sum / avail_tracker_cnt;
        auto r_center_cam = coordsolver_.worldToCam(target.centerR3d_world, rmat_imu_);
        auto center2d_src = coordsolver_.reproject(r_center_cam);
        auto angle = coordsolver_.getAngle(target.armor3d_cam, rmat_imu_);

        // RCLCPP_INFO(logger_, "armor3d_cam: %lf %lf %lf", target.armor3d_cam[0], target.armor3d_cam[1], target.armor3d_cam[2]);

        // 判断扇叶是否发生切换
        bool is_switched = false;
        is_switched = ((target.dz / last_fan_.dz) > 0 && target.dx / last_fan_.dx > 0) ? 0 : 1;
        double delta_angle = (target.angle - last_fan_.angle);
        if ((target.dx > 0 && target.dz > 0) || (target.dx < 0 && target.dz < 0))
            delta_angle = (delta_angle < 0) ? abs(delta_angle) : (delta_angle > 0 ? (-delta_angle) : delta_angle);
        // RCLCPP_INFO(logger_, "target.angle: %lf last_fan.angle:%lf", target.angle, last_fan_.angle);
        
        if (delta_angle_vec_.size() > 1)
        {
            double angle_sum = 0.0;
            for(auto& angle : delta_angle_vec_)
                angle_sum += angle;
            // angle_sum += delta_angle;
            delta_angle = angle_sum / (int)(delta_angle_vec_.size());
        }
        delta_angle_vec_.clear();
        RCLCPP_INFO(logger_, "delta_angle: %lf", delta_angle);

        if (!is_switched)
        {
            if(abs(delta_angle) > buff_param_.max_angle)
                is_switched = true;
        }
        
        if (is_switched)
        {
            target_info.angle_offset = target.angle - last_fan_.angle;
            double delta_angle_sum = 0.0;
            if (delta_angle_vec_.size() > 1)
            {
                for(auto angle : delta_angle_vec_)
                   delta_angle_sum += angle;
                delta_angle_sum -= delta_angle;
                delta_angle = delta_angle_sum / (int)(delta_angle_vec_.size() - 1);
            }
            else
            {
                delta_angle = (src.timestamp - last_timestamp_) / (last_timestamp_ - last_last_timestamp_) * last_delta_angle_;
            }
        }
        else
            target_info.angle_offset = 0.0;

        target_info.points2d[0].x = target.apex2d[0].x;
        target_info.points2d[0].y = target.apex2d[0].y;
        target_info.points2d[1].x = target.apex2d[1].x;
        target_info.points2d[1].y = target.apex2d[1].y;
        target_info.points2d[2].x = target.apex2d[2].x;
        target_info.points2d[2].y = target.apex2d[2].y;
        target_info.points2d[3].x = target.apex2d[3].x;
        target_info.points2d[3].y = target.apex2d[3].y;
        target_info.points2d[4].x = target.apex2d[4].x;
        target_info.points2d[4].y = target.apex2d[4].y;

        target_info.rmat = target.rmat;
        target_info.angle = target.angle;
        target_info.r_center = mean_r_center;
        target_info.delta_angle = delta_angle;
        target_info.target_switched = is_switched;
        target_info.armor3d_world = target.armor3d_world;
        target_info.armor3d_cam = target.armor3d_cam;
        
        // RCLCPP_INFO(logger_, "Target's angle: %lf", target_info.angle);
        // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 200, "Target is switched: %d", (int)(is_switched));

        lost_cnt_ = 0;
        last_roi_center_ = center2d_src;
        last_last_timestamp_ = last_timestamp_;
        last_timestamp_ = src.timestamp;
        last_fan_ = target;
        last_last_delta_angle_ = last_delta_angle_;
        last_delta_angle_ = delta_angle;
        is_last_target_exists_ = true;

        if (isnan(angle[0]) || isnan(angle[1]) || abs(angle[0]) > 45 || abs(angle[1]) > 45)
        {
            target_info.find_target = false;
            return false;
        }
        else
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 1000, "pitch: %lf yaw: %lf", angle[0], angle[1]);
        
        auto time_detect = steady_clock_.now();
        double dr_full_ns = (time_detect - time_start).nanoseconds();
        double dr_crop_ns = (time_crop - time_start).nanoseconds();
        double dr_infer_ns = (time_infer - time_start).nanoseconds();

        if (debug_param_.show_all_fans)
        {
            RCLCPP_DEBUG_ONCE(logger_, "Show all fans...");
            showFans(src);
        }

        if (debug_param_.show_fps)
        {
            RCLCPP_DEBUG_ONCE(logger_, "Show fps...");
            char ch[20];
            sprintf(ch, "%.2f", (1e9 / dr_full_ns));
            std::string fps_str = ch;
            putText(src.img, fps_str, {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
        }

        if (debug_param_.prinf_latency)
        {
            RCLCPP_DEBUG_ONCE(logger_, "Print latency...");
            //降低输出频率，避免影响帧率
            if ((int)(src.timestamp) % 10 == 0)
            {
                RCLCPP_INFO(logger_, "-----------TIME------------");
                RCLCPP_INFO(logger_, "Crop: %lfms", (dr_crop_ns / 1e6));
                RCLCPP_INFO(logger_, "Infer: %lfms", (dr_infer_ns / 1e6));
                RCLCPP_INFO(logger_, "Total: %lfms", (dr_full_ns / 1e6));
            }
        }
        if (debug_param_.print_target_info)
        {
            RCLCPP_DEBUG_ONCE(logger_, "Print target_info...");
            RCLCPP_INFO(logger_, "-----------INFO------------");
            RCLCPP_INFO(logger_, "Yaw: %lf", angle[0]);
            RCLCPP_INFO(logger_, "Pitch: %lf", angle[1]);
            RCLCPP_INFO(logger_, "Dist: %f m", (float)target.armor3d_cam.norm());
            RCLCPP_INFO(logger_, "Is switched: %d", (int)(is_switched));
        }

        target_info.find_target = true;
        return true;
    }

    void Detector::showFans(TaskData& src)
    {
        for (auto fan : fans_)
        {
            char ch[20];
            sprintf(ch, "%.2f", fan.conf);
            std::string conf_str = ch;
            putText(src.img, conf_str, fan.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

            if (fan.color == 0 || fan.color == 1 || fan.color == 2 || fan.color == 3)
                putText(src.img, fan.key, fan.apex2d[3], FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 2);
            for (int i = 0; i < 5; i++)
                line(src.img, fan.apex2d[i % 5], fan.apex2d[(i + 1) % 5], Scalar(125, 0, 255), 2);
            auto fan_armor_center = coordsolver_.reproject(fan.armor3d_cam);
            
            if (fan.id == 1)
                circle(src.img, fan_armor_center, 9, {125, 255, 125}, 2);
            else if (fan.id == 0)
                circle(src.img, fan_armor_center, 9, {0, 0, 255}, 2);
        }
    }

    bool Detector::chooseTarget(std::vector<Fan> &fans, Fan &target)
    {
        // float max_area = 0;
        // int target_idx = 0;
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
        if (!is_last_target_exists_)
        {
            // 当丢失目标帧数过多或lost_cnt为初值
            if (lost_cnt_ > buff_param_.max_lost_cnt || lost_cnt_ == 0)
            {
                return Point2i(0,0);
            }
        }

        // 若目标大小大于阈值
        if ((last_target_area_ / img.size().area()) > buff_param_.no_crop_thres)
        {
            return Point2i(0,0);
        }
        
        // 处理X越界
        if (last_roi_center_.x <= input_size_.width / 2)
            last_roi_center_.x = input_size_.width / 2;
        else if (last_roi_center_.x > (img.size().width - input_size_.width / 2))
            last_roi_center_.x = img.size().width - input_size_.width / 2;
        // 处理Y越界
        if (last_roi_center_.y <= input_size_.height / 2)
            last_roi_center_.y = input_size_.height / 2;
        else if (last_roi_center_.y > (img.size().height - input_size_.height / 2))
            last_roi_center_.y = img.size().height - input_size_.height / 2;

        // 左上角顶点
        auto offset = last_roi_center_ - Point2i(input_size_.width / 2, input_size_.height / 2);
        Rect roi_rect = Rect(offset, input_size_);
        img(roi_rect).copyTo(img);

        return offset;
    }
} // namespace buff_detector