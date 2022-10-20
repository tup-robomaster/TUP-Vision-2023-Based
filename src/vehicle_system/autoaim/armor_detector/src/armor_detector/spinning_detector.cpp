/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 21:39:01
 * @LastEditTime: 2022-10-17 13:54:10
 * @FilePath: /tup_2023-10-16/src/vehicle_system/autoaim/armor_detector/src/spinning_detector.cpp
 */
#include "../../include/armor_detector/spinning_detector.hpp"

namespace armor_detector
{
    spinning_detector::spinning_detector()
    {
        this->detect_color = RED;
        this->gyro_params_.max_dead_buffer = 2;
        this->gyro_params_.max_delta_dist = 3;
        this->gyro_params_.max_delta_t = 30;
        this->gyro_params_.hero_danger_zone = 4;
        this->gyro_params_.anti_spin_judge_high_thres = 4e3;
        this->gyro_params_.anti_spin_judge_low_thres = 2e3;
        this->gyro_params_.anti_spin_max_r_multiple = 3;
    }

    spinning_detector::spinning_detector(Color color, gyro_params _gyro_params_)
    {
        this->detect_color = color;
        this->gyro_params_.max_dead_buffer = _gyro_params_.max_dead_buffer;
        this->gyro_params_.max_delta_dist = _gyro_params_.max_delta_dist;
        this->gyro_params_.max_delta_t = _gyro_params_.max_delta_t;
        this->gyro_params_.hero_danger_zone = _gyro_params_.hero_danger_zone;
        this->gyro_params_.anti_spin_judge_high_thres = _gyro_params_.anti_spin_judge_high_thres;
        this->gyro_params_.anti_spin_judge_low_thres = _gyro_params_.anti_spin_judge_low_thres;
        this->gyro_params_.anti_spin_max_r_multiple = _gyro_params_.anti_spin_max_r_multiple;
    }

    spinning_detector::~spinning_detector()
    {
        
    }


    bool spinning_detector::update_spin_score()
    {
        /**
         * @brief 更新陀螺Score，函数关系在MATLAB中测试得出，在程序帧率恒定100fps
         * 的假设下，该函数关系可以在转速为 5rad/s -- 15rad/s 的情况下，
         * 在10到12次装甲板切换后识别出陀螺状态，无切换约0.5s-1s后自动退出陀螺状态
         * 
         * @return true 更新分数成功
         */
        for (auto score = spin_score_map.begin(); score != spin_score_map.end();)
        {
            SpinHeading spin_status;
            //若Status_Map不存在该元素

            if (spin_score_map.count((*score).first) == 0)
                spin_status = UNKNOWN;
            else
                spin_status = spin_status_map[(*score).first];
            // cout<<(*score).first<<"--:"<<(*score).second<<" "<<spin_status<<endl;
            LOG(INFO)<<"[SpinDetection] Current Spin score :"<<(*score).first<<" : "<<(*score).second<<" "<<spin_status;
            // 若分数过低移除此元素
            if (abs((*score).second) <= gyro_params_.anti_spin_judge_low_thres && spin_status != UNKNOWN)
            {
                fmt::print(fmt::fg(fmt::color::red), "[SpinDetection] Removing {}.\n", (*score).first);
                LOG(INFO)<<"[SpinDetection] Removing "<<(*score).first;
                spin_status_map.erase((*score).first);
                score = spin_score_map.erase(score);
                continue;
            }
            // 
            if (spin_status != UNKNOWN)
                (*score).second = 0.978 * (*score).second - 1 * abs((*score).second) / (*score).second;
            else
                (*score).second = 0.997 * (*score).second - 1 * abs((*score).second) / (*score).second;
            //当小于该值时移除该元素
            if (abs((*score).second) < 3 || isnan((*score).second))
            {
                spin_status_map.erase((*score).first);
                score = spin_score_map.erase(score);
                continue;
            }
            else if (abs((*score).second) >= gyro_params_.anti_spin_judge_high_thres)
            {
                (*score).second = gyro_params_.anti_spin_judge_high_thres * abs((*score).second) / (*score).second;
                if ((*score).second > 0)
                    spin_status_map[(*score).first] = CLOCKWISE;
                else if((*score).second < 0)
                    spin_status_map[(*score).first] = COUNTER_CLOCKWISE;
            }
            ++score;
        }
        // cout<<"++++++++++++++++++++++++++"<<endl;
        // for (auto status : spin_status_map)
        // {
        //     cout<<status.first<<" : "<<status.second<<endl;
        // }
        return true;
    }

    void spinning_detector::create_armor_tracker(int timestamp, int dead_buffer_cnt)
    {
        /**
         * @brief 生成/分配ArmorTracker
         * 
        */
        new_armors_cnt_map.clear();

        //为装甲板分配或新建最佳ArmorTracker(注:将不会为灰色装甲板创建预测器，只会分配给现有的预测器)
        for (auto armor = armors.begin(); armor != armors.end(); ++armor)
        {
            //当装甲板颜色为灰色且当前dead_buffer小于max_dead_buffer
            string tracker_key;
            if ((*armor).color == 2)
            {   
                if (dead_buffer_cnt >= gyro_params_.max_dead_buffer)
                    continue;
                
                if (detect_color == RED)
                    tracker_key = "R" + to_string((*armor).id);
                if (detect_color == BLUE)
                    tracker_key = "B" + to_string((*armor).id);
            }
            else
            {
                tracker_key = (*armor).key;
            }

            auto predictors_with_same_key = trackers_map.count(tracker_key);
            //当不存在该类型装甲板ArmorTracker且该装甲板Tracker类型不为灰色装甲板
            if (predictors_with_same_key == 0 && (*armor).color != 2)
            {
                ArmorTracker tracker((*armor), timestamp);
                auto target_predictor = trackers_map.insert(make_pair((*armor).key, tracker));
                new_armors_cnt_map[(*armor).key]++;
            }
            //当存在一个该类型ArmorTracker
            else if (predictors_with_same_key == 1)
            {
                auto candidate = trackers_map.find(tracker_key);
                auto delta_t = timestamp - (*candidate).second.last_timestamp;
                auto delta_dist = ((*armor).center3d_world - (*candidate).second.last_armor.center3d_world).norm();
                // auto iou = (*candidate).second.last_armor.roi & (*armor)
                // auto velocity = (delta_dist / delta_t) * 1e3;
                //若匹配则使用此ArmorTracker
                if (delta_dist <= gyro_params_.max_delta_dist && delta_t > 0 && (*candidate).second.last_armor.roi.contains((*armor).center2d))
                {
                    (*candidate).second.update((*armor), timestamp);
                }
                //若不匹配则创建新ArmorTracker
                else if ((*armor).color != 2)
                {
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
                
                //遍历所有同Key预测器，匹配速度最小且更新时间最近的ArmorTracker
                for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                {
                    auto delta_t = timestamp - (*iter).second.last_timestamp;
                    auto delta_dist = ((*armor).center3d_world - (*iter).second.last_armor.center3d_world).norm();
                    auto velocity = (delta_dist / delta_t) * 1e3;
                    
                    if ((*iter).second.last_armor.roi.contains((*armor).center2d) && delta_t > 0)
                    {
                        if (delta_dist <= gyro_params_.max_delta_dist && delta_dist <= min_delta_dist &&
                        delta_t <= min_delta_t)
                        {
                            min_delta_t = delta_t;
                            min_delta_dist = delta_dist;
                            best_candidate = iter;
                            is_best_candidate_exist = true;
                        }
                    }
                }
                if (is_best_candidate_exist)
                {
                    auto velocity = min_delta_dist;
                    auto delta_t = min_delta_t;
                    (*best_candidate).second.update((*armor), timestamp);
                }
                else if ((*armor).color != 2)
                {
                    ArmorTracker tracker((*armor), timestamp);
                    trackers_map.insert(make_pair((*armor).key, tracker));
                    new_armors_cnt_map[(*armor).key]++;
                }

            }
        }
        if (trackers_map.size() != 0)
        {
            //维护预测器Map，删除过久之前的装甲板
            for (auto iter = trackers_map.begin(); iter != trackers_map.end();)
            {
                //删除元素后迭代器会失效，需先行获取下一元素
                auto next = iter;
                // cout<<(*iter).second.last_timestamp<<"  "<<timestamp<<endl;
                if ((timestamp - (*iter).second.last_timestamp) > gyro_params_.max_delta_t)
                    next = trackers_map.erase(iter);
                else
                    ++next;
                iter = next;
            }
        }
    }

    bool spinning_detector::is_spinning()
    {
        /**
         * @brief 检测装甲板变化情况，计算各车陀螺分数
        */
        for (auto cnt : new_armors_cnt_map)
        {
            //只在该类别新增装甲板时数量为1时计算陀螺分数
            if (cnt.second == 1)
            {
                auto same_armors_cnt = trackers_map.count(cnt.first);
                if (same_armors_cnt == 2)
                {
                    // cout<<"1"<<endl;
                    //遍历所有同Key预测器，确定左右侧的Tracker
                    armor_detector::ArmorTracker *new_tracker = nullptr;
                    armor_detector::ArmorTracker *last_tracker = nullptr;
                    double last_armor_center;
                    double last_armor_timestamp;
                    double new_armor_center;
                    double new_armor_timestamp;
                    int best_prev_timestamp = 0;    //候选ArmorTracker的最近时间戳

                    //求出重复key键的元素数目，并返回两个map类型迭代器，分别指向该key的起始位置和结束位置的下一个元素位置
                    auto candiadates = trackers_map.equal_range(cnt.first);
                    
                    for (auto iter = candiadates.first; iter != candiadates.second; ++iter)
                    {
                        //若未完成初始化则视为新增tracker
                        if (!(*iter).second.is_initialized && (*iter).second.last_timestamp == (*iter).second.prev_timestamp)
                        {
                            new_tracker = &(*iter).second;
                        }
                        else if ((*iter).second.last_timestamp > best_prev_timestamp && (*iter).second.is_initialized)
                        {
                            best_prev_timestamp = (*iter).second.last_timestamp;
                            last_tracker = &(*iter).second;
                        }
                        
                    }
                    if (new_tracker != nullptr && last_tracker != nullptr)
                    {
                        new_armor_center = new_tracker->last_armor.center2d.x;
                        new_armor_timestamp = new_tracker->last_timestamp;
                        last_armor_center = last_tracker->last_armor.center2d.x;
                        last_armor_timestamp = last_tracker->last_timestamp;
                        auto spin_movement = new_armor_center - last_armor_center;
                        // auto delta_t = 
                        LOG(INFO)<<"[SpinDetection] Candidate Spin Movement Detected : "<<cnt.first<<" : "<<spin_movement;
                        if (abs(spin_movement) > 10 && new_armor_timestamp == new_tracker->prev_timestamp && last_armor_timestamp == new_tracker->prev_timestamp)
                        {
                            //若无该元素则插入新元素
                            if (spin_score_map.count(cnt.first) == 0)
                            {
                                spin_score_map[cnt.first] = 1000 * spin_movement / abs(spin_movement);
                            }
                            //若已有该元素且目前旋转方向与记录不同,则对目前分数进行减半惩罚
                            else if (spin_movement * spin_score_map[cnt.first] < 0)
                            {
                                spin_score_map[cnt.first] *= 0.5;
                            }
                            //若已有该元素则更新元素
                            else
                            {
                                spin_score_map[cnt.first] = gyro_params_.anti_spin_max_r_multiple * spin_score_map[cnt.first];
                            }
                        }
                    }
                }
            }
        }
        return true;
    }

    // detector::ArmorTracker* spinning_detector::chooseTargetTracker(vector<detector::ArmorTracker*> trackers, int timestamp, int prev_timestamp)
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
    //         if (armor.id == 1 && armor.center3d_world.norm() <= gyro_params_.hero_danger_zone)
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
