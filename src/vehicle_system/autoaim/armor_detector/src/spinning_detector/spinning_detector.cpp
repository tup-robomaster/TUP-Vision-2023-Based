/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 21:39:01
 * @LastEditTime: 2023-04-16 21:47:14
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/spinning_detector/spinning_detector.cpp
 */
#include "../../include/spinning_detector/spinning_detector.hpp"

namespace armor_detector
{
    SpinningDetector::SpinningDetector()
    : logger_(rclcpp::get_logger("spinning_detector"))
    {
        detector_info_.last_add_tracker_timestamp = 0;
        detector_info_.new_add_tracker_timestamp = 0;
        this->detect_color = RED;
        this->gyro_params_.max_dead_buffer = 2;
        this->gyro_params_.max_delta_dist = 3.0;
        this->gyro_params_.max_delta_t = 30.0;
        this->gyro_params_.hero_danger_zone = 4.0;
        this->gyro_params_.anti_spin_judge_high_thres = 4e3;
        this->gyro_params_.anti_spin_judge_low_thres = 2e3;
        this->gyro_params_.anti_spin_max_r_multiple = 3;
        last_timestamp_ = 0.0;
        max_hop_period_ = 3000.0;
        is_dead_= false;
        // normal_gyro_status_counter_ = 0;
        // switch_gyro_status_counter_ = 0;
        // last_yaw_diff_ = 0.0;
        // last_pitch_diff_ = 0.0;
    }

    SpinningDetector::SpinningDetector(Color color, GyroParam gyro_params)
    : logger_(rclcpp::get_logger("spinning_detector"))
    {
        detector_info_.last_add_tracker_timestamp = 0;
        detector_info_.new_add_tracker_timestamp = 0;
        this->detect_color = color;
        this->gyro_params_.max_dead_buffer = gyro_params.max_dead_buffer;
        this->gyro_params_.max_delta_dist = gyro_params.max_delta_dist;
        this->gyro_params_.max_delta_t = gyro_params.max_delta_t;
        this->gyro_params_.hero_danger_zone = gyro_params.hero_danger_zone;
        this->gyro_params_.anti_spin_judge_high_thres = gyro_params.anti_spin_judge_high_thres;
        this->gyro_params_.anti_spin_judge_low_thres = gyro_params.anti_spin_judge_low_thres;
        this->gyro_params_.anti_spin_max_r_multiple = gyro_params.anti_spin_max_r_multiple;
        last_timestamp_ = 0.0;
        max_hop_period_ = 3000.0;
        is_dead_= false;
        // normal_gyro_status_counter_ = 0;
        // switch_gyro_status_counter_ = 0;
        // last_yaw_diff_ = 0.0;
        // last_pitch_diff_ = 0.0;
    }

    SpinningDetector::~SpinningDetector()
    {
        
    }

    /**
     * @brief 更新目标的旋转分数
     * 用于判断目标是否处于小陀螺状态
     * @return true 
     * @return false 
     */
    bool SpinningDetector::updateSpinScore()
    {
        /**
         * @brief 更新陀螺Score，函数关系在MATLAB中测试得出，在程序帧率恒定100fps
         * 的假设下，该函数关系可以在转速为 5rad/s -- 15rad/s 的情况下，
         * 在10到12次装甲板切换后识别出陀螺状态，无切换约0.5s-1s后自动退出陀螺状态
         * 
         * @return true 更新分数成功
         */
        for (auto score = spinning_map_.spin_score_map.begin(); score != spinning_map_.spin_score_map.end();)
        {
            SpinHeading spin_status;
            
            //若Status_Map不存在该元素
            if (spinning_map_.spin_score_map.count((*score).first) == 0)
                spin_status = UNKNOWN;
            else
                spin_status = spinning_map_.spin_status_map[(*score).first];
            
            RCLCPP_INFO(logger_, "Spin status: %d", (int)(spin_status));
            // 若分数过低移且目标陀螺状态已知除此元素
            if (abs((*score).second) <= gyro_params_.anti_spin_judge_low_thres && spin_status != UNKNOWN)
            {
                RCLCPP_INFO(logger_, "Removing %s", (*score).first.c_str());
                spinning_map_.spin_status_map.erase((*score).first);
                score = spinning_map_.spin_score_map.erase(score);
                continue;
            }
            
            if (abs((*score).second) > gyro_params_.anti_spin_judge_low_thres && spin_status != UNKNOWN)
            {   // 若陀螺分数大于低阈值且状态未确定，则对目前分数按陀螺状态按陀螺状态未确定的衰减函数进行衰减
                (*score).second = 0.978 * (*score).second - 1 * abs((*score).second) / (*score).second;
            }
            else
            {   // 若目前陀螺分数大于低阈值且陀螺状态已知，则对目前分数按陀螺状态已确定的衰减函数进行衰减
                (*score).second = 0.997 * (*score).second - 1 * abs((*score).second) / (*score).second;
            }
            
            RCLCPP_INFO(logger_, "Score: %lf", (*score).second);
            // 当小于该值时移除该元素
            if (abs((*score).second) < 3 || isnan((*score).second))
            {
                spinning_map_.spin_status_map.erase((*score).first);
                score = spinning_map_.spin_score_map.erase(score);
                continue;
            }
            else if (abs((*score).second) >= gyro_params_.anti_spin_judge_high_thres)
            {   // 若目标分数高于高阈值，则截断，避免分数过高
                (*score).second = gyro_params_.anti_spin_judge_high_thres * abs((*score).second) / (*score).second;
                if ((*score).second > 0)
                    spinning_map_.spin_status_map[(*score).first] = CLOCKWISE;
                else if((*score).second < 0)
                    spinning_map_.spin_status_map[(*score).first] = COUNTER_CLOCKWISE;
            }
            ++score;
        }
        return true;
    }

    /**
     * @brief 生成/分配ArmorTracker
     * 
     * @param trackers_map 追踪器multimap
     * @param armors 本帧检测到的装甲板对象
     * @param new_armors_cnt_map 不同车辆新增装甲板的数量map
     * @param timestamp 本帧对应的时间戳
     * @param dead_buffer_cnt 目标装甲板灯条灭掉的帧数
     */
    void SpinningDetector::createArmorTracker(std::multimap<std::string, ArmorTracker>& trackers_map, std::vector<Armor>& armors, std::map<std::string, int>& new_armors_cnt_map, int64_t timestamp, int dead_buffer_cnt)
    {
        new_armors_cnt_map.clear();

        //为装甲板分配或新建最佳ArmorTracker(注:将不会为灰色装甲板创建预测器，只会分配给现有的预测器)
        for (auto armor = armors.begin(); armor != armors.end(); ++armor)
        {
            //当装甲板颜色为灰色且当前dead_buffer小于max_dead_buffer
            string tracker_key;
            if ((*armor).color == 2)
            {   
                RCLCPP_WARN(logger_, "Gray armor...");
                if (dead_buffer_cnt >= gyro_params_.max_dead_buffer)
                {
                    RCLCPP_WARN(logger_, "dead buffer cnt: %d", dead_buffer_cnt);
                    is_dead_ = true;
                    continue;
                }

                if (detect_color == RED)
                    tracker_key = "R" + to_string((*armor).id);
                if (detect_color == BLUE)
                    tracker_key = "B" + to_string((*armor).id);
            }
            else
            {
                tracker_key = (*armor).key;
            }

            int predictors_with_same_key = trackers_map.count(tracker_key);
            if (predictors_with_same_key == 0 && (*armor).color != 2)
            {   // 当不存在该类型装甲板ArmorTracker且该装甲板Tracker类型不为灰色装甲板
                ArmorTracker tracker((*armor), timestamp);
                auto target_predictor = trackers_map.insert(make_pair((*armor).key, tracker));
                new_armors_cnt_map[(*armor).key]++;
            }
            else if(predictors_with_same_key == 1)
            {   // 当存在一个该类型ArmorTracker
                auto candidate = trackers_map.find(tracker_key);
                int64_t delta_t = timestamp - (*candidate).second.now;
                double delta_dist = abs(((*armor).armor3d_world - (*candidate).second.new_armor.armor3d_world).norm());
                // auto iou = (*candidate).second.last_armor.roi & (*armor);
                // auto velocity = (delta_dist / delta_t) * 1e3;
                 
                // 若匹配则使用此ArmorTracker
                if (delta_dist <= gyro_params_.max_delta_dist && delta_t > 0 && (*candidate).second.new_armor.roi.contains((*armor).center2d))
                {   // 若当前装甲板与上一次的距离小于阈值，并且当前装甲板的中心在上一次装甲板的roi范围内则视为同一装甲板目标，对此tracker进行更新
                    (*candidate).second.update((*armor), timestamp);
                }
                else if ((*armor).color != 2)
                {   // 若不匹配且不为灰色装甲板则创建新ArmorTracker（不为灰色装甲板分配新的追踪器）
                    ArmorTracker tracker((*armor), timestamp);
                    trackers_map.insert(make_pair((*armor).key, tracker));
                    new_armors_cnt_map[(*armor).key]++;
                }
            }
            else
            {   //当存在多个该类型装甲板ArmorTracker
                //1e9无实际意义，仅用于以非零初始化
                double min_delta_dist = 1e9;
                int64_t min_delta_t = (int64_t)9e19;
                bool is_best_candidate_exist = false;
                std::multimap<string, ArmorTracker>::iterator best_candidate;
                auto candiadates = trackers_map.equal_range(tracker_key);
                for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                {   // 遍历所有同Key预测器，匹配速度最小且更新时间最近的ArmorTracker
                    int64_t delta_t = timestamp - (*iter).second.now;
                    double delta_dist = abs((*armor).armor3d_world.norm() - (*iter).second.new_armor.armor3d_world.norm());
                    // double velocity = (delta_dist / delta_t) * 1e9;
                    
                    if ((*iter).second.new_armor.roi.contains((*armor).center2d) && delta_t > 0)
                    {   // 若当前预测器中的装甲板的roi包含当前装甲板的中心
                        // RCLCPP_WARN(logger_, "time:%ld dist:%.3f", delta_t, delta_dist);
                        if (delta_dist <= gyro_params_.max_delta_dist && delta_dist <= min_delta_dist && delta_t <= min_delta_t)
                        {   // 若两个装甲板的距离差小于阈值、距离小于当前最小距离，以及时间差小于当前最小时间差，则更新
                            min_delta_t = delta_t;
                            min_delta_dist = delta_dist;
                            best_candidate = iter;
                            is_best_candidate_exist = true;
                        }
                    }
                }
                if (is_best_candidate_exist)
                {   // 若找到速度最小且更新时间最近的tracker，则更新
                    // auto velocity = min_delta_dist;
                    // auto delta_t = min_delta_t;
                    (*best_candidate).second.update((*armor), timestamp);
                }
                else if ((*armor).color != 2)
                {   // 若未匹配到，则新建tracker（灰色装甲板只会分配给已有tracker，不会新建tracker）
                    ArmorTracker tracker((*armor), timestamp);
                    trackers_map.insert(make_pair((*armor).key, tracker));
                    new_armors_cnt_map[(*armor).key]++;
                }
            }
        }
<<<<<<< HEAD

=======
>>>>>>> origin/develop
        if (trackers_map.size() != 0)
        {   //维护预测器Map，删除过久之前的装甲板
            for (auto iter = trackers_map.begin(); iter != trackers_map.end();)
            {   //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                if ((timestamp - (*iter).second.now) / 1e6 > gyro_params_.max_delta_t)
                    next = trackers_map.erase(iter);
                else
                    ++next;
                iter = next;
            }       
        }

        if (spinning_map_.spinning_x_map.size() != 0)
        {
            for (auto iter = spinning_map_.spinning_x_map.begin(); iter != spinning_map_.spinning_x_map.end();)
            {   
                auto next = iter;
                if ((timestamp - (*iter).second.new_timestamp) / 1e6 > gyro_params_.switch_max_dt)
                    next = spinning_map_.spinning_x_map.erase(iter);
                else
                    ++next;
                iter = next;
            }
        }
    }

    /**
     * @brief 判断tracker是否处于陀螺状态
     * @param trackers_map 追踪器multimap 
     * @param now 当前帧时间戳
     * @note 此函数通过计算tracker前后帧装甲板在绝对系（云台系）下的pitch_diff_angle和yaw_diff_angle来判断目标是否处于陀螺状态
    */
    bool SpinningDetector::isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, int64_t now)
    {
        // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Spin detecting...");
        // for (auto & tracker : trackers_map)
        // {
        //     if (tracker.second.is_initialized)
        //     {
        //         if (spinning_map_.spin_counter_map.count(tracker.first) == 0)
        //         {
        //             spinning_map_.spin_counter_map.insert(make_pair(tracker.first, SpinCounter()));
        //         }

        //         if (spinning_map_.spin_status_map.count(tracker.first) == 0)
        //         {
        //             spinning_map_.spin_status_map.insert(make_pair(tracker.first, SpinState()));
        //         }

        //         //判断当前tracker是否存在当前帧的装甲板目标
        //         if ((tracker.second.now / 1e9) == (now / 1e9) && (tracker.second.last_timestamp / 1e9) == (last_timestamp_ / 1e9))
        //         {
        //             Eigen::Matrix3d relative_rmat = tracker.second.last_armor.rmat.transpose() * tracker.second.new_armor.rmat;
        //             Eigen::AngleAxisd axisd_angle = Eigen::AngleAxisd(relative_rmat);
        //             double relative_angle = axisd_angle.angle();
        //             if (spinning_map_.spin_status_map[tracker.first].switch_timestamp != 0)
        //             {
        //                 double dt = (now - spinning_map_.spin_status_map[tracker.first].switch_timestamp) / 1e6;
        //                 // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "dt:%.2fms", dt);
        //                 if (dt < max_hop_period_)
        //                 {   //当前帧与装甲板切换帧的时间戳差值小于最大跳变周期
        //                     if (relative_angle > 0)
        //                     {
        //                         ++spinning_map_.spin_counter_map[tracker.first].flag;
        //                     }
        //                     else if (relative_angle < 0)
        //                     {
        //                         --spinning_map_.spin_counter_map[tracker.first].flag;
        //                     }
        //                     tracker.second.relative_angle = relative_angle;

        //                     RCLCPP_WARN_THROTTLE(
        //                         logger_, 
        //                         steady_clock_, 
        //                         500, 
        //                         "relative_angle:%.2f",
        //                         relative_angle * (180 / CV_PI)
        //                     );

        //                     // SpinHeading spin_status = tracker.second.spin_status_;
        //                     SpinHeading spin_status = spinning_map_.spin_status_map[tracker.first].spin_state;
        //                     if (tracker.second.last_armor.armor3d_world.norm() < gyro_params_.max_conf_dis)
        //                     {
        //                         if (spin_status == CLOCKWISE && relative_angle < 0)
        //                         {
        //                             if (abs(relative_angle) >= gyro_params_.max_rotation_angle)
        //                             {   //对目标进行陀螺计数
        //                                 ++spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                             else if (abs(relative_angle) <= gyro_params_.min_rotation_angle)
        //                             {
        //                                 --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                         }
        //                         else if (spin_status == COUNTER_CLOCKWISE && relative_angle > 0)
        //                         {
        //                             if (relative_angle >= gyro_params_.max_rotation_angle)
        //                             {
        //                                 ++spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                             else if (relative_angle <= gyro_params_.min_rotation_angle)
        //                             {
        //                                 --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                         }
        //                         else if (spin_status == UNKNOWN)
        //                         {
        //                             if (abs(relative_angle) >= gyro_params_.max_rotation_angle)
        //                             {
        //                                 // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Spin state:{UNKNOWN} yaw_diff:%.2f max_yaw_hop:%.2f", abs(yaw_diff) * (180 / M_PI), gyro_params_.max_rotation_angle);
        //                                 ++spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                             else if (abs(relative_angle) <= gyro_params_.min_rotation_angle)
        //                             {
        //                                 --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                         }
        //                         else
        //                         {
        //                             --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                         }
        //                     }
        //                     else
        //                     {   //目标距离较远
        //                         if (spin_status == CLOCKWISE && relative_angle < 0)
        //                         {
        //                             if (abs(relative_angle) >= gyro_params_.max_rotation_angle)
        //                             {   //对目标进行陀螺计数
        //                                 ++spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                             else if (abs(relative_angle) <= gyro_params_.min_rotation_angle)
        //                             {
        //                                 --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                         }
        //                         else if (spin_status == COUNTER_CLOCKWISE && relative_angle > 0)
        //                         {
        //                             if (relative_angle >= gyro_params_.max_rotation_angle)
        //                             {
        //                                 ++spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                             else if (relative_angle <= gyro_params_.min_rotation_angle)
        //                             {
        //                                 --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                         }
        //                         else if (spin_status == UNKNOWN)
        //                         {
        //                             if (abs(relative_angle) >= gyro_params_.max_rotation_angle)
        //                             {
        //                                 // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Spin state:{UNKNOWN} yaw_diff:%.2f max_yaw_hop:%.2f", abs(yaw_diff) * (180 / M_PI), gyro_params_.max_rotation_angle * (180 / M_PI));
        //                                 ++spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                             else if (abs(relative_angle) <= gyro_params_.min_rotation_angle)
        //                             {
        //                                 --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                             }
        //                         }
        //                         else
        //                         {
        //                             --spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter;
        //                         }
        //                     }
        //                 }
        //                 else
        //                 {   //当前帧与装甲板切换帧的时间戳差值大于最大跳变周期，此时认为目标退出陀螺状态
        //                     tracker.second.relative_angle = relative_angle;
        //                     spinning_map_.spin_status_map[tracker.first].switch_timestamp = 0;
        //                     spinning_map_.spin_status_map[tracker.first].spin_state = UNKNOWN;
        //                     spinning_map_.spin_counter_map[tracker.first].flag = 0;
        //                     spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter = 0;
        //                     spinning_map_.spin_counter_map[tracker.first].switch_gyro_status_counter = 0;

        //                     RCLCPP_WARN_THROTTLE(
        //                         logger_, 
        //                         steady_clock_, 
        //                         500, 
        //                         "relative_angle:%.2f",
        //                         relative_angle * (180 / CV_PI)
        //                     );
        //                 }
        //             }
        //             else
        //             {   //追踪期间目标未发生装甲板切换，此时先不进入陀螺识别
        //                 // RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "[Spinning]: No hop...");
        //             }  
        //         }
        //     }
        // }

        // last_timestamp_ = now;
        return true;
    }

    /**
     * @brief 判断目标是否处于小陀螺状态
     * 
     * @param trackers_map 车辆追踪器multimap
     * @param new_armors_cnt_map 不同车辆新增装甲板的数量map
     * @param timestamp 本帧对应的时间戳
     * @return true 
     * @return false 
     */
    bool SpinningDetector::isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, std::map<std::string, int>& new_armors_cnt_map, int64_t now)
    {
        /**
         * @brief 检测装甲板变化情况，计算各车陀螺分数
        */
        for (auto cnt : new_armors_cnt_map)
        {   //只在该类别新增装甲板数量为1时计算陀螺分数
            if (cnt.second == 1)
            {
                int same_armors_cnt = trackers_map.count(cnt.first);
                if (same_armors_cnt >= 2)
                {   // 若相同key键的tracker存在两个，一个为新增，一个先前存在，且两个tracker本次都有更新，则视为一次陀螺动作（其实就是对应目标车辆小陀螺时两个装甲板同时出现在视野的情况）
                    // 遍历所有同Key预测器，确定左右侧的Tracker
                    ArmorTracker *new_tracker = nullptr;
                    ArmorTracker *last_tracker = nullptr;
                    double last_armor_center;
                    double new_armor_center;
                    int64_t last_armor_timestamp;
                    int64_t new_armor_timestamp;
                    int64_t best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳

                    //求出重复key键的元素数目，并返回两个map类型迭代器，分别指向该key的起始位置和结束位置的下一个元素位置
                    auto candiadates = trackers_map.equal_range(cnt.first);
                    for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                    {
                        // RCLCPP_WARN(logger_, "dt:%d now:%d", (*iter).second.now, now);
                        //若未完成初始化则视为新增tracker
                        if (!(*iter).second.is_initialized && (*iter).second.now == now)
                        {
                            // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[Spinning]: New tracker");
                            new_tracker = &(*iter).second;
                        }
                        else if ((*iter).second.is_initialized && (*iter).second.now > best_prev_timestamp)
                        {
                            // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500,  "[Spinning]: Last tracker");
                            best_prev_timestamp = (*iter).second.now;
                            last_tracker = &(*iter).second;
                        }
                    }
                    if (new_tracker != nullptr && last_tracker != nullptr)
                    {
                        RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[Spinning]: Switching...");

                        new_armor_center = new_tracker->new_armor.center2d.x;
                        new_armor_timestamp = new_tracker->now;
                        last_armor_center = last_tracker->new_armor.center2d.x;
                        last_armor_timestamp = last_tracker->now;
                        auto spin_movement = new_armor_center - last_armor_center;
                        auto spin_x_dis = last_tracker->new_armor.armor3d_world[1] - new_tracker->new_armor.armor3d_world[1];

                        // spinning_map_.spin_counter_map[cnt.first].flag += (spin_x_dis > 0) ? (-25) : 25; 
                        // if (spinning_map_.spin_status_map[cnt.first].spin_state != UNKNOWN
                        // && (new_armor_timestamp - spinning_map_.spin_status_map[cnt.first].switch_timestamp) / 1e6 > gyro_params_.switch_max_dt
                        // && (last_armor_timestamp - spinning_map_.spin_status_map[cnt.first].switch_timestamp) / 1e6 > gyro_params_.switch_max_dt)
                        // {
                        //     // last_tracker->normal_gyro_status_counter_ = 0;
                        //     // last_tracker->switch_gyro_status_counter_ = 0;
                        //     // new_tracker->normal_gyro_status_counter_ = 0;
                        //     // new_tracker->switch_gyro_status_counter_ = 0;
                        //     // last_tracker->flag_ = 0;
                        //     // last_tracker->spin_status_ = UNKNOWN;
                        //     // new_tracker->spin_status_ = UNKNOWN;
                        //     spinning_map_.spin_counter_map[cnt.first].flag = 0;
                        //     spinning_map_.spin_counter_map[cnt.first].normal_gyro_status_counter = 0;
                        //     spinning_map_.spin_counter_map[cnt.first].switch_gyro_status_counter = 0;
                            
                        //     spinning_map_.spin_status_map[cnt.first].switch_timestamp = 0;
                        //     spinning_map_.spin_status_map[cnt.first].spin_state = UNKNOWN;
                        // }
                        // RCLCPP_INFO(logger_, "[Spinning]: spin_x_dix:%.3f last_dt:%.5f new_dt:%.5f now:%.5f", abs(spin_x_dis), last_armor_timestamp / 1e9, new_armor_timestamp / 1e9, now / 1e9);

                        // RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "switch_dx:%.3f", abs(spin_x_dis));
                        if (abs(spin_x_dis) > 0.10 && new_armor_timestamp == now && last_armor_timestamp == now)
                        {
                            detector_info_.last_add_tracker_timestamp = detector_info_.new_add_tracker_timestamp;
                            detector_info_.new_add_tracker_timestamp = new_armor_timestamp;
                            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[Spinning]: Switched...");
                            
                            // 记录新增的装甲板对应的时间戳
                            // auto cnt = spinning_time_map.count(new_tracker->key);
                            // if(cnt == 0)
                            // {   // 若multimap中没有当前key键，则创建
                            //     Timestamp ts;
                            //     ts.new_timestamp = new_add_tracker_timestamp;
                            //     ts.last_timestamp = 0;
                            //     spinning_time_map.insert(make_pair(new_tracker->key, ts));
                            // }1 若multimap中存在上次新增装甲板对应的key键，则更新
                            //     auto candidate = spinning_time_map.find(new_tracker->key);
                            //     (*candidate).second.last_timestamp = (*candidate).second.new_timestamp;
                            //     (*candidate).second.new_timestamp = new_armor_timestamp;
                            // }

                            auto cnts = spinning_map_.spinning_x_map.count(new_tracker->key);
                            if(cnts == 0)
                            {
                                GyroInfo gyro_info;
                                gyro_info.last_rmat = last_tracker->last_armor.rmat;
                                gyro_info.new_rmat = new_tracker->last_armor.rmat;
                                gyro_info.new_x_font = last_tracker->last_armor.armor3d_world[1];
                                gyro_info.new_x_back = new_tracker->last_armor.armor3d_world[1];
                                gyro_info.new_y_font = last_tracker->last_armor.armor3d_world[0];
                                gyro_info.new_y_back = new_tracker->last_armor.armor3d_world[0]; 
                                gyro_info.new_timestamp = new_armor_timestamp;
                                gyro_info.last_x_back = 0;
                                gyro_info.last_x_back = 0;
                                gyro_info.last_y_back = 0;
                                gyro_info.last_y_font = 0;
                                gyro_info.last_timestamp = 0;
                                spinning_map_.spinning_x_map.insert(make_pair(new_tracker->key, gyro_info));
                            }
                            else
                            {
                                auto candidate = spinning_map_.spinning_x_map.find(new_tracker->key);
                                (*candidate).second.last_x_font = (*candidate).second.new_x_font;
                                (*candidate).second.last_x_back = (*candidate).second.new_x_back;
                                (*candidate).second.last_y_font = (*candidate).second.new_y_font;
                                (*candidate).second.last_y_back = (*candidate).second.new_y_back;
                                (*candidate).second.last_timestamp = (*candidate).second.new_timestamp;

                                (*candidate).second.new_x_font = last_tracker->last_armor.armor3d_world[1];
                                (*candidate).second.new_x_back = new_tracker->last_armor.armor3d_world[1];
                                (*candidate).second.new_y_font = last_tracker->last_armor.armor3d_world[0];
                                (*candidate).second.new_y_back = new_tracker->last_armor.armor3d_world[0];
                                (*candidate).second.new_timestamp = new_armor_timestamp;

                                (*candidate).second.last_rmat = last_tracker->last_armor.rmat;
                                (*candidate).second.new_rmat = new_tracker->last_armor.rmat;

                                // RCLCPP_INFO(logger_, "last_y_font:%lf last_y_back:%lf new_y_font:%lf new_y_back:%lf", (*candidate).second.last_y_font, (*candidate).second.last_y_back, (*candidate).second.new_y_font, (*candidate).second.new_y_back);
                                // RCLCPP_INFO(logger_, "now_dt:%lf last_time:%lf", (new_armor_timestamp / 1e9), ((*candidate).second.last_timestamp / 1e9));
                            }

                            // spinning_map_.spin_status_map[cnt.first].switch_timestamp = now;
                            // ++spinning_map_.spin_counter_map[cnt.first].switch_gyro_status_counter;
                            
                            // new_tracker->hop_timestamp_ = now;
                            // last_tracker->hop_timestamp_ = now;
                            // ++new_tracker->switch_gyro_status_counter_;

                            if (spinning_map_.spin_score_map.count(cnt.first) == 0)
                            {   //若无该元素则插入新元素，为车辆的陀螺分数赋初值
                                spinning_map_.spin_score_map[cnt.first] = 1000 * spin_movement / abs(spin_movement);
                            }
                            else if (spin_movement * spinning_map_.spin_score_map[cnt.first] < 0)
                            {   //若已有该元素且目前旋转方向与记录不同,则对目前车辆的陀螺分数进行减半惩罚
                                spinning_map_.spin_score_map[cnt.first] *= 0.5;
                            }
                            else
                            {   //若目前旋转方向与记录同向，并且已有该元素则更新元素
                                spinning_map_.spin_score_map[cnt.first] = gyro_params_.anti_spin_max_r_multiple * spinning_map_.spin_score_map[cnt.first];
                            }
                        }
                        // else
                        // {
                        //     // RCLCPP_WARN(logger_, "switch_dx:%.3f", abs(spin_x_dis));
                        //     // --new_tracker->switch_gyro_status_counter_;
                        //     spinning_map_.spin_status_map[cnt.first].switch_timestamp = now;
                        //     --spinning_map_.spin_counter_map[cnt.first].switch_gyro_status_counter;
                        // }
                    }
                }
            }
        }

        // if (isSpinning(trackers_map, now))
        // {
        //     for (auto & tracker : spinning_map_.spin_status_map)
        //     {
        //         // if (tracker.second.switch_timestamp != 0)
        //         // {
        //         //     double dt = (now - tracker.second.switch_timestamp) / 1e6;
        //         //     if (dt > max_hop_period_)
        //         //     {
        //         //         spinning_map_.spin_counter_map[tracker.first].flag = 0;
        //         //         spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter = 0;   
        //         //         spinning_map_.spin_counter_map[tracker.first].switch_gyro_status_counter = 0;   
        //         //         spinning_map_.spin_status_map[tracker.first].switch_timestamp = 0;
        //         //         spinning_map_.spin_status_map[tracker.first].spin_state = UNKNOWN;
        //         //     }
        //         // }

        //         if (abs(spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter) >= 25
        //         || (spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter >= 10
        //         && spinning_map_.spin_counter_map[tracker.first].switch_gyro_status_counter >= 3))
        //         {
        //             SpinHeading spin_status = UNKNOWN;
        //             int count = spinning_map_.spin_counter_map[tracker.first].flag;
        //             if (count >= 25)
        //                 spin_status = COUNTER_CLOCKWISE;
        //             else if (count <= -25)
        //                 spin_status = COUNTER_CLOCKWISE;
        //             // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "flag:%d", count);
        //             spinning_map_.spin_status_map[tracker.first].spin_state = spin_status;
        //             if (spinning_map_.spin_status_map[tracker.first].spin_state == CLOCKWISE)
        //             {
        //                 RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[Spinning]:CLOCKWISE");
        //             }
        //             else 
        //             {
        //                 RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[Spinning]:COUNTER_CLOCKWISE");
        //             }
        //         }
        //         else
        //         {
        //             // tracker.second.spin_status_ = UNKNOWN;
        //             spinning_map_.spin_status_map[tracker.first].switch_timestamp = 0;
        //             spinning_map_.spin_status_map[tracker.first].spin_state = UNKNOWN;
        //         }
        //         // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "Normal_counter:%d Switch_counter:%d", spinning_map_.spin_counter_map[tracker.first].normal_gyro_status_counter, spinning_map_.spin_counter_map[tracker.first].switch_gyro_status_counter);
        //     }
        // }
        return true;
    }
} //namespace detector
