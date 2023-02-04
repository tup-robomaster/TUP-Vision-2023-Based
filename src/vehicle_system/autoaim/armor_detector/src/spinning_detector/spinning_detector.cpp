/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 21:39:01
 * @LastEditTime: 2023-02-05 00:41:17
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
                // fmt::print(fmt::fg(fmt::color::red), "[SpinDetection] Removing {}.\n", (*score).first);
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
    void SpinningDetector::createArmorTracker(std::multimap<std::string, ArmorTracker>& trackers_map, std::vector<Armor>& armors, std::map<std::string, int>& new_armors_cnt_map, double timestamp, int dead_buffer_cnt)
    {
        new_armors_cnt_map.clear();

        //为装甲板分配或新建最佳ArmorTracker(注:将不会为灰色装甲板创建预测器，只会分配给现有的预测器)
        for (auto armor = armors.begin(); armor != armors.end(); ++armor)
        {
            //当装甲板颜色为灰色且当前dead_buffer小于max_dead_buffer
            string tracker_key;
            if ((*armor).color == 2)
            {   
                if (dead_buffer_cnt >= gyro_params_.max_dead_buffer)
                {
                    RCLCPP_INFO(logger_, "dead buffer cnt: %d", dead_buffer_cnt);
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
                double delta_t = timestamp - (*candidate).second.last_timestamp;
                double delta_dist = ((*armor).armor3d_world - (*candidate).second.last_armor.armor3d_world).norm();
                // auto iou = (*candidate).second.last_armor.roi & (*armor);
                // auto velocity = (delta_dist / delta_t) * 1e3;
                
                // 若匹配则使用此ArmorTracker
                if (delta_dist <= gyro_params_.max_delta_dist && delta_t > 0 && (*candidate).second.last_armor.roi.contains((*armor).center2d))
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
                int min_delta_t = 1e9;
                bool is_best_candidate_exist = false;
                std::multimap<string, ArmorTracker>::iterator best_candidate;
                auto candiadates = trackers_map.equal_range(tracker_key);
                
                for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                {   // 遍历所有同Key预测器，匹配速度最小且更新时间最近的ArmorTracker
                    double delta_t = timestamp - (*iter).second.last_timestamp;
                    double delta_dist = ((*armor).armor3d_world - (*iter).second.last_armor.armor3d_world).norm();
                    double velocity = (delta_dist / delta_t) * 1e9;
                    
                    if ((*iter).second.last_armor.roi.contains((*armor).center2d) && delta_t > 0)
                    {   // 若当前预测器中的装甲板的roi包含当前装甲板的中心
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
                    auto velocity = min_delta_dist;
                    auto delta_t = min_delta_t;
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
        if (trackers_map.size() != 0)
        {   //维护预测器Map，删除过久之前的装甲板
            for (auto iter = trackers_map.begin(); iter != trackers_map.end();)
            {   //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                if ((timestamp - (*iter).second.last_timestamp) / 1e6 > gyro_params_.max_delta_t)
                    next = trackers_map.erase(iter);
                else
                    ++next;
                iter = next;
            }       

            for (auto iter = spinning_map_.spinning_x_map.begin(); iter != spinning_map_.spinning_x_map.end();)
            {   
                auto next = iter;
                if ((timestamp - (*iter).second.new_timestamp) / 1e9 > 30)
                    next = spinning_map_.spinning_x_map.erase(iter);
                else
                    ++next;
                iter = next;
            }
        }
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
    bool SpinningDetector::isSpinning(std::multimap<std::string, ArmorTracker>& trackers_map, std::map<std::string, int>& new_armors_cnt_map, double timestamp)
    {
        /**
         * @brief 检测装甲板变化情况，计算各车陀螺分数
        */
        for (auto cnt : new_armors_cnt_map)
        {   //只在该类别新增装甲板数量为1时计算陀螺分数
            // std::cout << 1 << std::endl;
            if (cnt.second == 1)
            {
                int same_armors_cnt = trackers_map.count(cnt.first);
                if (same_armors_cnt == 2)
                {   // 若相同key键的tracker存在两个，一个为新增，一个先前存在，且两个tracker本次都有更新，则视为一次陀螺动作（其实就是对应目标车辆小陀螺时两个装甲板同时出现在视野的情况）
                    // 遍历所有同Key预测器，确定左右侧的Tracker
                    // std::cout << 2 << std::endl;

                    ArmorTracker *new_tracker = nullptr;
                    ArmorTracker *last_tracker = nullptr;
                    double last_armor_center;
                    double last_armor_timestamp;
                    double new_armor_center;
                    double new_armor_timestamp;
                    int best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳

                    //求出重复key键的元素数目，并返回两个map类型迭代器，分别指向该key的起始位置和结束位置的下一个元素位置
                    auto candiadates = trackers_map.equal_range(cnt.first);
                    
                    for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                    {
                        // std::cout << 3 << " last_time:" << (*iter).second.last_timestamp / 1e9 << " cur_time:" << timestamp / 1e9 << std::endl;

                        //若未完成初始化则视为新增tracker
                        if (!(*iter).second.is_initialized && (*iter).second.last_timestamp == timestamp)
                        {
                            // std::cout << 6 << std::endl;

                            new_tracker = &(*iter).second;
                        }
                        else if ((*iter).second.last_timestamp > best_prev_timestamp && (*iter).second.is_initialized)
                        {
                            // std::cout << 7 << std::endl;

                            best_prev_timestamp = (*iter).second.last_timestamp;
                            last_tracker = &(*iter).second;
                        }
                    }
                    if (new_tracker != nullptr && last_tracker != nullptr)
                    {
                        // std::cout << 4 << std::endl;

                        new_armor_center = new_tracker->last_armor.center2d.x;
                        new_armor_timestamp = new_tracker->last_timestamp;
                        last_armor_center = last_tracker->last_armor.center2d.x;
                        last_armor_timestamp = last_tracker->last_timestamp;
                        auto spin_movement = new_armor_center - last_armor_center;
                        auto spin_x_dis = last_tracker->last_armor.armor3d_world[0] - new_tracker->last_armor.armor3d_world[0];

                        // LOG(INFO)<<"[SpinDetection] Candidate Spin Movement Detected : "<<cnt.first<<" : "<<spin_movement;
                        //TODO:to be fixed!!!
                        if (abs(spin_movement) > 10 && abs(spin_x_dis) > 0.15 && new_armor_timestamp == timestamp && last_armor_timestamp == timestamp)
                        {
                            // std::cout << 5 << std::endl;

                            detector_info_.last_add_tracker_timestamp = detector_info_.new_add_tracker_timestamp;
                            detector_info_.new_add_tracker_timestamp = new_armor_timestamp;
                            
                            // 记录新增的装甲板对应的时间戳
                            // auto cnt = spinning_time_map.count(new_tracker->key);
                            // if(cnt == 0)
                            // {   // 若multimap中没有当前key键，则创建
                            //     Timestamp ts;
                            //     ts.new_timestamp = new_add_tracker_timestamp;
                            //     ts.last_timestamp = 0;
                            //     spinning_time_map.insert(make_pair(new_tracker->key, ts));
                            // }
                            // else
                            // {   // 若multimap中存在上次新增装甲板对应的key键，则更新
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
                                gyro_info.new_x_font = last_tracker->last_armor.armor3d_world[0];
                                gyro_info.new_x_back = new_tracker->last_armor.armor3d_world[0];
                                gyro_info.new_x_font_2d = last_tracker->last_armor.center2d.x;
                                gyro_info.new_x_back_2d = new_tracker->last_armor.center2d.x; 
                                gyro_info.new_timestamp = new_armor_timestamp;
                                gyro_info.last_x_back = 0;
                                gyro_info.last_x_back = 0;
                                gyro_info.last_x_back_2d = 0;
                                gyro_info.last_x_font_2d = 0;
                                gyro_info.last_timestamp = 0;
                                spinning_map_.spinning_x_map.insert(make_pair(new_tracker->key, gyro_info));
                            }
                            else
                            {
                                auto candidate = spinning_map_.spinning_x_map.find(new_tracker->key);
                                (*candidate).second.last_x_font = (*candidate).second.new_x_font;
                                (*candidate).second.last_x_back = (*candidate).second.new_x_back;
                                (*candidate).second.last_x_font_2d = (*candidate).second.new_x_font_2d;
                                (*candidate).second.last_x_back_2d = (*candidate).second.new_x_back_2d;
                                (*candidate).second.last_timestamp = (*candidate).second.new_timestamp;

                                (*candidate).second.new_x_font = last_tracker->last_armor.armor3d_world[0];
                                (*candidate).second.new_x_back = new_tracker->last_armor.armor3d_world[0];
                                (*candidate).second.new_x_font_2d = last_tracker->last_armor.center2d.x;
                                (*candidate).second.new_x_back_2d = new_tracker->last_armor.center2d.x;
                                (*candidate).second.new_timestamp = new_armor_timestamp;

                                (*candidate).second.last_rmat = last_tracker->last_armor.rmat;
                                (*candidate).second.new_rmat = new_tracker->last_armor.rmat;

                                RCLCPP_INFO(logger_, "now_dt:%lf last_time:%lf", (new_armor_timestamp / 1e9), ((*candidate).second.last_timestamp / 1e9));
                            }

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
                    }
                }
            }
        }
        return true;
    }

    // detector::ArmorTracker* SpinningDetector::chooseTargetTracker(vector<detector::ArmorTracker*> trackers, int timestamp, int prev_timestamp)
    // {
    //     //TODO:优化打击逻辑
    //     //TODO:本逻辑为哨兵逻辑
    //     float max_score = 0;
    //     int target_idx = 0;
    //     int last_target_idx = -1;
    //     // cout<<trackers.size()<<endl;
    //     for (int i = 0; i < trackers.size(); i++)
    //     {
    //         //计算tracker的切换打击分数,由装甲板旋转角度,距离,面积大小决定
    //         if (trackers[i]->last_timestamp == prev_timestamp)
    //         {
    //             if (trackers[i]->last_selected_timestamp == prev_timestamp && abs(prev_timestamp - timestamp) < 100)
    //                 last_target_idx = i;
    //             if (trackers[i]->hit_score > max_score)
    //             {
    //                 max_score = trackers[i]->hit_score;
    //                 target_idx = i;
    //             }
    //         }
    //     }

    //     //若存在上次存在目标且分数与相差不大，选择该装甲板
    //     if (last_target_idx != -1 && abs(trackers[last_target_idx]->hit_score - max_score) / max_score < 0.1)
    //         target_idx = last_target_idx;
    //     return trackers[target_idx];
    // }

    // int spinning_detector::chooseTargetID(vector<detector::Armor> &armors, int timestamp, int prev_timestamp)
    // {
    //     //TODO:自瞄逻辑修改
    //     bool is_last_id_exists = false;
    //     int target_id;
    //     //该选择逻辑主要存在两层约束:
    //     //英雄约束与上次目标约束
    //     //若检测到危险距离内的英雄直接退出循环
    //     //若检测到存在上次击打目标,时间较短,且该目标运动较小,则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
    //     for (auto armor : armors)
    //     {
    //         //FIXME:该处需根据兵种修改
    //         //若视野中存在英雄且距离小于危险距离，直接选为目标
    //         if (armor.id == 1 && armor.armor3d_world.norm() <= gyro_params_.hero_danger_zone)
    //         {
    //             return armor.id;
    //         }
    //         //若存在上次击打目标,时间较短,且该目标运动较小则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
    //         else if (armor.id == last_armor.id && abs(armor.area - last_armor.area) / (float)armor.area < 0.3  && abs(timestamp - prev_timestamp) < 30)
    //         {
    //             is_last_id_exists = true;
    //             target_id = armor.id;
    //         }
    //     }
    //     //若不存在则返回面积最大的装甲板序号，即队列首元素序号
    //     if (is_last_id_exists)
    //         return target_id;
    //     else
    //         return (*armors.begin()).id;
    // }

} //namespace detector
