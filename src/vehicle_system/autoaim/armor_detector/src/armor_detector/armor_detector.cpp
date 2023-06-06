/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:26:16
 * @LastEditTime: 2023-06-02 22:05:34
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/armor_detector/armor_detector.cpp
 */
#include "../../include/armor_detector/armor_detector.hpp"

using namespace std;
namespace armor_detector
{
    Detector::Detector(const PathParam& path_param, const DetectorParam& detector_params, const DebugParam& debug_params, const GyroParam& gyro_params)
    : spinning_detector_(detector_params.color, gyro_params), detector_params_(detector_params), debug_params_(debug_params),
    logger_(rclcpp::get_logger("armor_detector"))
    {
        //初始化clear
        lost_cnt_ = 0;
        is_last_target_exists_ = false;
        is_target_switched_ = false;
        last_target_area_ = 0;
        last_bullet_speed_ = 0;
        input_size_ = {720, 720};
        is_init_ = false;
        last_period_ = 0;
        last_last_status_ = last_status_ = cur_status_ = NONE;

        is_save_data_ = debug_params_.save_data; //save distance error data
        save_dataset_ = debug_params_.save_dataset;

        car_id_map_ = 
        {
            {"B0", 0}, {"B1", 1},
            {"B2", 2}, {"B3", 3},
            {"B4", 4}, {"R0", 5},
            {"R1", 6}, {"R2", 7},
            {"R3", 8}, {"R4", 9} 
        };
    }

    Detector::~Detector()
    {
        if(is_save_data_)
        {
            data_save_.close();
        }
    }

    /**
     * @brief 车辆装甲板检测
     * 
     * @param src 图像数据结构体
     * @param is_target_lost 目标是否丢失
     * @return true 
     * @return false 
     */
    bool Detector::armor_detect(TaskData &src, bool& is_target_lost)
    {
        time_start_ = steady_clock_.now();
        last_timestamp_ = now_;
        now_ = src.timestamp;
        auto input = src.img;
        
        rmat_imu_ = src.quat.toRotationMatrix();

        if (debug_params_.use_roi)
        {   //启用roi
            //吊射模式采用固定ROI
            if (src.mode == AUTOAIM_SLING)
            {
                input(Range(600, 1024), Range(432, 848)).copyTo(input);
                roi_offset_ = cv::Point2d((float)432, (float)600);
            }
            else
            {
                roi_offset_ = cropImageByROI(input);
            }
            RCLCPP_INFO_ONCE(logger_, "Using roi...");
        }

        time_crop_ = steady_clock_.now();

        objects_.clear();
        new_armors_.clear();
        if (!armor_detector_.detect(input, objects_))
        {   //若未检测到目标
            if (debug_params_.show_aim_cross)
            {
                drawAimCrossCurve(src.img);
            }

            is_target_lost = true;
            lost_cnt_++;
            is_last_target_exists_ = false;
            last_target_area_ = 0.0;

            return false;
        }
        time_infer_ = steady_clock_.now();
        
        //将对象排序，保留面积较大的对象
        sort(objects_.begin(), objects_.end(), [](ArmorObject& prev, ArmorObject& next)
        {
            return prev.area > next.area;
        });

        //若对象较多保留前按面积排序后的前max_armors个
        if ((int)(objects_.size()) > this->detector_params_.max_armors_cnt)
            objects_.resize(this->detector_params_.max_armors_cnt);
        
        //生成装甲板对象
        for (auto object : objects_)
        {
            //TODO:加入紫色装甲板限制通过条件
            if (detector_params_.color == RED)
            {
                if (object.color == BLUE_SMALL || object.color == BLUE_BIG || object.color == PURPLE_SMALL || object.color == PURPLE_BIG)
                    continue;
            }
            else if (detector_params_.color == BLUE)
            {
                if (object.color == RED_SMALL || object.color == RED_BIG || object.color == PURPLE_SMALL || object.color == PURPLE_BIG)
                    continue;
            }
   
            Armor armor;
            armor.id = object.cls;
            armor.color = object.color;
            armor.conf = object.prob;
            
            TargetType target_type = SMALL;
            if (object.color == BLUE_SMALL)
            {
                target_type = SMALL;
                armor.key = "B" + to_string(object.cls);
            }
            else if (object.color == BLUE_BIG)
            {
                target_type = BIG;
                armor.key = "B" + to_string(object.cls);
            }
            else if (object.color == GRAY_SMALL)
            {
                target_type = SMALL;
                armor.key = "N" + to_string(object.cls);
            }
            else if (object.color == GRAY_BIG)
            {
                target_type = BIG;
                armor.key = "N" + to_string(object.cls);
            }
            else if (object.color == RED_SMALL)
            {
                target_type = SMALL;
                armor.key = "R" + to_string(object.cls);
            }     
            else if (object.color == RED_BIG)
            {
                target_type = BIG;
                armor.key = "R" + to_string(object.cls);
            }   
            else if (object.color == PURPLE_SMALL)
            {
                target_type = SMALL;
                armor.key = "P" + to_string(object.cls);
            }
            else if (object.color == PURPLE_BIG)
            {
                target_type = BIG;
                armor.key = "P" + to_string(object.cls);
            }
            
            memcpy(armor.apex2d, object.apex, 4 * sizeof(cv::Point2f));
            for(int i = 0; i < 4; i++)
                armor.apex2d[i] += cv::Point2f((float)roi_offset_.x,(float)roi_offset_.y);
            cv::Point2f apex_sum;
            for(auto apex : armor.apex2d)
                apex_sum += apex;
            armor.center2d = apex_sum / 4.f;
            //若装甲板置信度小于高阈值，需要相同位置存在过装甲板才放行
            if (armor.conf <= this->detector_params_.armor_conf_high_thresh)
            {
                if (!last_armors_.empty())
                {
                    bool is_this_armor_available = false;
                    for (auto last_armor : last_armors_)
                    {
                        if (last_armor.roi.contains(armor.center2d))
                        {
                            is_this_armor_available = true;
                            break;
                        }
                    }
                    if (!is_this_armor_available)
                    {
                        RCLCPP_WARN_THROTTLE(
                            logger_, 
                            steady_clock_, 
                            500, 
                            "[IGNORE]:armor_key:%s armor_conf:%.2f", 
                            armor.key.c_str(), 
                            armor.conf
                        );
                        continue;
                    }
                }
                else
                {
                    continue;
                }
            }

            //生成装甲板旋转矩形和ROI
            std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
            RotatedRect points_pic_rrect = minAreaRect(points_pic); 
            armor.rrect = points_pic_rrect;
            auto bbox = points_pic_rrect.boundingRect();
            
            auto x = bbox.x - 0.5 * bbox.width * (detector_params_.armor_roi_expand_ratio_width - 1);
            auto y = bbox.y - 0.5 * bbox.height * (detector_params_.armor_roi_expand_ratio_height - 1);
            armor.roi = Rect(
                x, 
                y,
                bbox.width * detector_params_.armor_roi_expand_ratio_width,
                bbox.height * detector_params_.armor_roi_expand_ratio_height
            );

            //进行PnP，目标较少时采取迭代法，较多时采用IPPE
            int pnp_method = ((int)objects_.size() <= 2) ? SOLVEPNP_ITERATIVE : SOLVEPNP_IPPE;

            //计算长宽比,确定装甲板类型
            auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) / min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            if (abs(armor.rrect.angle) > 80.0 || abs(armor.rrect.angle) < 10.0)
            {
                if (apex_wh_ratio >= detector_params_.armor_type_wh_high_thresh)
                {
                    target_type = BIG;
                }
                else if (apex_wh_ratio <= detector_params_.armor_type_wh_high_thresh)
                {
                    target_type = SMALL;
                }
            }
            
            // if (object.cls == 1 || apex_wh_ratio > detector_params_.armor_type_wh_thresh)
            // {   //若大于长宽阈值或为哨兵、英雄装甲板
            //     target_type = BIG;
            // }
            // else if (object.cls == 2 || object.cls == 3 || object.cls == 4 || object.cls == 5 || object.cls == 6)
            // {   //FIXME：若存在平衡步兵需要对此处步兵装甲板类型进行修改
            //     target_type = SMALL;
            // }
            RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 50, "ID: %d target_type: %s", armor.id, target_type == 0 ? "SMALL_ARMOR" : "BIG_ARMOR");

            // 单目PnP
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu_, target_type, pnp_method);
            if (!pnp_result.is_solver_success)
            {
                continue;
            }
            
            //防止装甲板类型出错导致解算问题，首先尝试切换装甲板类型，若仍无效则直接跳过该装甲板
            if (!isPnpSolverValidation(pnp_result.armor_cam))
            {
                target_type = (target_type == SMALL) ? BIG : SMALL;
                pnp_result = coordsolver_.pnp(points_pic, rmat_imu_, target_type, SOLVEPNP_IPPE);
                if (!pnp_result.is_solver_success)
                {
                    continue;
                }
                if (!isPnpSolverValidation(pnp_result.armor_cam))
                {
                    continue;
                }
            }

            armor.armor3d_world = pnp_result.armor_world;
            armor.armor3d_cam = pnp_result.armor_cam;
            armor.euler = pnp_result.euler;
            armor.rmat = pnp_result.rmat;
            armor.area = object.area;
            armor.rangle = pnp_result.rangle;
            new_armors_.emplace_back(armor);
        }

        if (debug_params_.show_crop_img)
        {
            cv::namedWindow("crop_img", cv::WINDOW_AUTOSIZE);
            cv::imshow("crop_img", input);
            cv::waitKey(1);
        }
        
        //若无合适装甲板
        if (new_armors_.empty())
        {
            RCLCPP_WARN_THROTTLE(logger_, this->steady_clock_, 500, "No suitable targets...");
            if(debug_params_.show_aim_cross)
            {
                drawAimCrossCurve(src.img);
            }

            if(debug_params_.show_all_armors)
            {
                RCLCPP_DEBUG_ONCE(logger_, "Show all armors...");
                showArmors(src);
            }

            is_target_lost = true;
            lost_cnt_++;
            is_last_target_exists_ = false;
            last_target_area_ = 0;
            return false;
        }
        else
        {
            if(save_dataset_)
            {
                bool is_init = false;
                for(auto armor : new_armors_)
                {
                    vector<cv::Point2d> cornor_points(armor.apex2d, armor.apex2d + 4);
                    autoLabel(is_init, src.img, file_, path_prefix_, now_,
                        armor.id, armor.color, cornor_points, roi_offset_, input_size_);
                }
            }
            last_armors_ = new_armors_;
        }
        is_target_lost = false;
        return true;
    }

    /**
     * @brief 车辆小陀螺状态检测
     * 
     * @param src 图像数据结构体
     * @param autoaim_msg 目标装甲板message
     * @return true 
     * @return false 
     */
    bool Detector::gyro_detector(TaskData &src, global_interface::msg::Autoaim& autoaim_msg, ObjHPMsg hp, DecisionMsg decision_msg)
    {
        //Choose target vehicle
        //此处首先根据哨兵发来的ID指令进行目标车辆追踪
        int target_id = -1;
        target_id = chooseTargetID(src);

        //Create ArmorTracker for new armors 
        spinning_detector_.createArmorTracker(trackers_map_, new_armors_, new_armors_cnt_map_, now_);
        
        //Detect armors status
        spinning_detector_.isSpinning(trackers_map_, new_armors_cnt_map_, now_);
        
        //未检索到有效车辆ID，直接退出
        if(target_id == -1)
        {
            autoaim_msg.is_target_lost = true;
            autoaim_msg.target_switched = true;
            autoaim_msg.is_spinning = false;
            return false;
        }

        string target_key;
        //TODO:考虑灰色装甲板
        if (detector_params_.color == BLUE)
        {
            target_key = "B" + to_string(target_id);
        }
        else if (detector_params_.color == RED)
        {
            target_key = "R" + to_string(target_id);
        }
        
        RCLCPP_INFO_THROTTLE(
            logger_, 
            this->steady_clock_, 
            500, 
            "Target key: %s", 
            target_key.c_str()
        );

        ///-----------------------------detect whether exists matched tracker------------------------------------------
        if (trackers_map_.count(target_key) == 0)
        {
            if(debug_params_.show_aim_cross)
            {
                drawAimCrossCurve(src.img);
            }

            if(debug_params_.show_all_armors)
            {
                showArmors(src);
            }

            autoaim_msg.is_target_lost = true;
            lost_cnt_++;
            is_last_target_exists_ = false;
            last_target_area_ = 0;
            RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "No available tracker exists!");
            return false;
        }

        auto ID_candiadates = trackers_map_.equal_range(target_key);

        ///---------------------------acqusition final armor's sequences---------------------------------------
        bool is_target_spinning;
        Armor target;
        std::vector<ArmorTracker*> final_trackers;
        std::vector<Armor> final_armors;
        
        // 确定目标运动状态（陀螺模式与机动模式）
        SpinHeading spin_status;
        if (spinning_detector_.spinning_map_.spin_status_map.count(target_key) == 0)
        {   //若未确定打击车辆的陀螺状态
            spin_status = UNKNOWN;
            is_target_spinning = false;
            autoaim_msg.is_spinning = false;
        }
        else
        {   //若确定打击车辆的陀螺状态
            spin_status = spinning_detector_.spinning_map_.spin_status_map[target_key].spin_state;
            if (spin_status != UNKNOWN)
            {
                is_target_spinning = true;
                autoaim_msg.is_spinning = true;
            }
            else
            {
                is_target_spinning = false;
                autoaim_msg.is_spinning = false;
            }
        }

        ///----------------------------------反陀螺击打---------------------------------------
        autoaim_msg.is_spinning = false;
        autoaim_msg.spinning_switched = false;
        if (spin_status != UNKNOWN)
        {
            //------------------------------估计目标旋转周期-----------------------------------
            double w = 0.0;
            double period = 0.0;
            for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
            {
                if (((*iter).second.now / 1e9) == (src.timestamp / 1e9))
                {
                    final_armors.emplace_back((*iter).second.new_armor);
                    final_trackers.emplace_back(&(*iter).second);
                    if((*iter).second.is_initialized)
                    {
                        auto dt = (((*iter).second.now) - ((*iter).second.last_timestamp)) / 1e9;
                        auto rrmat = ((*iter).second.last_armor.rmat.transpose()) * ((*iter).second.new_armor.rmat);
                        auto angle_axisd = Eigen::AngleAxisd(rrmat);
                        auto angle = angle_axisd.angle();
                        w = (angle / dt);
                        period = ((2 * CV_PI) / w);
                        new_period_deq_.emplace_back(period);
                    }
                }
                else
                {
                    continue;
                }
            }

            if ((int)final_armors.size() == 0)
            {
                autoaim_msg.is_target_lost = true;
                lost_cnt_++;
                is_last_target_exists_ = false;
                
                RCLCPP_ERROR(logger_, "Error in dead zone : size is zero...");
                return false;
            }

            auto cnt = spinning_detector_.spinning_map_.spinning_x_map.count(target_key);
            if(cnt == 1)
            {
                auto candidate = spinning_detector_.spinning_map_.spinning_x_map.find(target_key);
                
                if(new_period_deq_.size() > 1)
                {
                    int idx = 0;
                    double per_sum = 0.0;
                    for(auto per : new_period_deq_)
                    {
                        if(!isnan(per) && !isinf(per) && per < 0.6)
                        {
                            per_sum += per;
                            idx++;
                        }
                    }
                    if(idx != 0)
                    {
                        double ave_per = (per_sum / idx);
                        autoaim_msg.spinning_period = ave_per;
                        RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 500, "ave_per:%lfs", ave_per);
                    }
                    else
                        autoaim_msg.spinning_period = period;
                }

                double delta_x_back = abs((*candidate).second.new_x_back - (*candidate).second.last_x_back);
                double delta_x_font = abs((*candidate).second.new_x_font - (*candidate).second.last_x_font);
                double ave_x_3d = (delta_x_back + delta_x_font) / 2;
                
                if(ave_x_3d > spinning_detector_.gyro_params_.delta_x_3d_high_thresh)
                {
                    autoaim_msg.is_spinning = true;
                    RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[SPINNING]: Movement Spinning...");
                } 
                else if(ave_x_3d < spinning_detector_.gyro_params_.delta_x_3d_low_thresh)
                {
                    autoaim_msg.is_spinning = true;
                    RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "[SPINNING]: Still Spinning...");
                }
                else
                {
                    autoaim_msg.is_spinning = true;
                }
            }
            
            //若存在一块装甲板
            if (final_armors.size() == 1)
            {
                autoaim_msg.spinning_switched = false;
                if(spin_status == CLOCKWISE)
                    autoaim_msg.is_clockwise = CLOCKWISE;
                else
                    autoaim_msg.is_clockwise = false;
                
                last_last_status_ = last_status_;
                last_status_ = cur_status_;
                cur_status_ = SINGER;

                final_armors[0].is_front = true;
                target = final_armors.at(0);
            }
            else if (final_armors.size() == 2)
            {   //若存在两块装甲板
                last_last_status_ = last_status_;
                last_status_ = cur_status_;
                cur_status_ = DOUBLE;
                
                // 选择旋转方向上落后的装甲板进行击打
                // 对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                sort(final_armors.begin(),final_armors.end(),[](Armor& prev, Armor& next)
                                    {return prev.armor3d_cam[0] < next.armor3d_cam[0];});
                // 若顺时针旋转选取右侧装甲板更新
                if (spin_status == CLOCKWISE)
                {       
                    autoaim_msg.is_clockwise = CLOCKWISE;
                    final_armors[1].is_front = true;
                    target = final_armors.at(1);
                }
                else if (spin_status == COUNTER_CLOCKWISE)
                {   // 若逆时针旋转选取左侧装甲板更新
                    autoaim_msg.is_clockwise = false;
                    final_armors[0].is_front = true;
                    target = final_armors.at(0);
                }   
            }
            else
            {
                autoaim_msg.is_target_lost = true;
                lost_cnt_++;
                is_last_target_exists_ = false;
                // target = final_armors.front();
                RCLCPP_ERROR(logger_, "Error in dead zone: armor size:%d", (int)final_armors.size());
                return false;
            }

            //判断装甲板是否切换，若切换将变量置1
            if ((target.id != last_armor_.id || !last_armor_.roi.contains((target.center2d))) &&
                !is_last_target_exists_)
                autoaim_msg.spinning_switched = true;
            else
                autoaim_msg.spinning_switched = false;
        }
        else
        {
            cur_ave_period_ = 0;
            last_period_ = 0;
            last_last_status_ = last_status_ = cur_status_ = NONE;
            history_period_.clear();
            new_period_deq_.clear();
            for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
            {
                final_armors.emplace_back((*iter).second.new_armor);
                final_trackers.emplace_back(&(*iter).second);
            }

            //进行目标选择
            auto tracker = chooseTargetTracker(src, final_trackers);
            tracker->last_selected_timestamp = now_;
            tracker->selected_cnt++;
            target = tracker->new_armor;
            
            //判断装甲板是否切换，若切换将变量置1
            double delta_dist = (target.armor3d_world - last_armor_.armor3d_world).norm();
            if (target.id != last_armor_.id || delta_dist >= 1.5)
            {
                RCLCPP_INFO_THROTTLE(
                    logger_,
                    steady_clock_,
                    50,
                    "Target_switched..."
                );
                is_target_switched_ = true;
                autoaim_msg.target_switched = true;
            }
            else
            {
                is_target_switched_ = false;
                autoaim_msg.target_switched = false;
            }

            double x_2d_dis = abs(last_armor_.center2d.x - target.center2d.x);
            if (target_id != last_armor_.id || (!last_armor_.roi.contains(target.center2d) && (x_2d_dis >= 250 || delta_dist >= 0.35)))
            {
                RCLCPP_INFO_THROTTLE(
                    logger_,
                    steady_clock_,
                    50,
                    "Spinning switched..."
                );
                autoaim_msg.spinning_switched = true;
            }
        }

        if ((int)final_armors.size() > 0)
        {
            autoaim_msg.armors.clear();
            ArmorMsg armor_msg;
            armor_msg.key = target.key;
            for (int ii = 0; ii < 4; ii++)
            {
                armor_msg.point2d[ii].x = target.apex2d[ii].x; 
                armor_msg.point2d[ii].y = target.apex2d[ii].y; 
            }
            armor_msg.point3d_cam.x = target.armor3d_cam[0];
            armor_msg.point3d_cam.y = target.armor3d_cam[1];
            armor_msg.point3d_cam.z = target.armor3d_cam[2];
            armor_msg.point3d_world.x = target.armor3d_world[0];
            armor_msg.point3d_world.y = target.armor3d_world[1];
            armor_msg.point3d_world.z = target.armor3d_world[2];
            armor_msg.is_last_exists = false;
            for (auto last_armor : last_armors_)
            {
                if (last_armor.id == target.id && last_armor.roi.contains(target.center2d))
                {
                    armor_msg.is_last_exists = true;
                }
            }
            armor_msg.is_front = target.is_front;
            armor_msg.rangle = target.rangle;
            // armor_msg.rangle = 0.0;
            // cout << "armor_rangle:" << armor_msg.rangle << endl;

            autoaim_msg.armors.emplace_back(armor_msg);

            // for (auto armor : final_armors)
            // {
            //     armor_msg.key = armor.key;
            //     for (int ii = 0; ii < 4; ii++)
            //     {
            //         armor_msg.point2d.x = armor.apex2d[ii].x; 
            //         armor_msg.point2d.y = armor.apex2d[ii].y; 
            //     }
            //     armor_msg.point3d_cam.x = armor.armor3d_cam[0];
            //     armor_msg.point3d_cam.y = armor.armor3d_cam[1];
            //     armor_msg.point3d_cam.z = armor.armor3d_cam[2];
            //     armor_msg.point3d_world.x = armor.armor3d_world[0];
            //     armor_msg.point3d_world.y = armor.armor3d_world[1];
            //     armor_msg.point3d_world.z = armor.armor3d_world[2];
            //     armor_msg.is_last_exists = false;
            //     for (auto last_armor : last_armors_)
            //     {
            //         if (last_armor.id == armor.id && last_armor.roi.contains(armor.center2d))
            //         {
            //             armor_msg.is_last_exists = true;
            //         }
            //     }
            //     armor_msg.is_front = armor.is_front;
            //     // autoaim_msg.armors.emplace_back(armor_msg);
            // }
        }
        else
        {
            autoaim_msg.is_target_lost = true;
            return false;
        }

        if(last_status_ == SINGER && cur_status_ == DOUBLE)
            autoaim_msg.spinning_switched = true;
        
        int target_hp = car_id_map_[target.key];
        autoaim_msg.vehicle_id = target.key;
        autoaim_msg.vehicle_hp = target_hp;
        autoaim_msg.is_target_lost = false;

        RCLCPP_INFO_THROTTLE(
            logger_, 
            steady_clock_, 
            200, 
            "xyz_cam: (%.3f %.3f %.3f) xyz_world: (%.3f %.3f %.3f)", 
            autoaim_msg.armors.front().point3d_cam.x, autoaim_msg.armors.front().point3d_cam.y, autoaim_msg.armors.front().point3d_cam.z,
            autoaim_msg.armors.front().point3d_world.x, autoaim_msg.armors.front().point3d_world.y, autoaim_msg.armors.front().point3d_world.z
        );

        //获取装甲板中心与装甲板面积以下一次ROI截取使用
        // last_roi_center_ = Point2i(512,640);
        last_roi_center_ = target.center2d;
        last_armor_ = target;
        lost_cnt_ = 0;
        last_target_area_ = target.area;
        last_aiming_point_ = target.armor3d_cam;
        is_last_target_exists_ = true;
        last_armors_ = new_armors_;

        if(debug_params_.show_aim_cross)
        {
            drawAimCrossCurve(src.img);
        }

        if(debug_params_.show_all_armors)
        {
            showArmors(src);
            // RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 200, "rotate_angle:%.3f", target.rrect.angle);  
            char ch[10];
            sprintf(ch, "%.2f", target.rrect.angle);
            std::string angle_str = ch;     
            putText(src.img, angle_str, target.apex2d[1] + cv::Point2f{0, 26}, FONT_HERSHEY_SIMPLEX, 1, {255, 125, 125}, 1);
            auto armor_center = coordsolver_.reproject(target.armor3d_cam);
            circle(src.img, armor_center, 4, {0, 0, 255}, 2);
            line(src.img, cv::Point2f(armor_center.x - 25, armor_center.y), cv::Point2f(armor_center.x + 25, armor_center.y), {0, 0, 255}, 1);
            line(src.img, cv::Point2f(armor_center.x, armor_center.y - 30), cv::Point2f(armor_center.x, armor_center.y + 30), {0, 0, 255}, 1);
            line(src.img, cv::Point2f(armor_center.x - 25, armor_center.y), cv::Point2f(armor_center.x + 25, armor_center.y), {0, 0, 255}, 1);
            line(src.img, cv::Point2f(armor_center.x, armor_center.y - 30), cv::Point2f(armor_center.x, armor_center.y + 30), {0, 0, 255}, 1);
        }
        
        auto angle = coordsolver_.getAngle(target.armor3d_cam, rmat_imu_);
        // 若预测出错则直接世界坐标系下坐标作为击打点
        if (!isAngleSolverValidataion(angle))
        {
            angle = coordsolver_.getAngle(target.armor3d_world, rmat_imu_);
            RCLCPP_ERROR(logger_, "Error while solving angle: %.2f %.2f", angle[0], angle[1]);
        }
        
        auto time_predict = steady_clock_.now();
        double dr_crop_ns = (time_crop_ - time_start_).nanoseconds();
        double dr_infer_ns = (time_infer_ - time_crop_).nanoseconds();
        double dr_full_ns = (time_predict - time_start_).nanoseconds();
        if(debug_params_.show_fps)
        {
            char ch[10];
            sprintf(ch, "%.2f", (1e9 / dr_full_ns));
            std::string fps_str = ch;
            putText(src.img, fps_str, {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
        }

        if(debug_params_.print_latency)
        {
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "-----------TIME------------");
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Crop:  %lfms", (dr_crop_ns / 1e6));
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Infer: %lfms", (dr_infer_ns / 1e6));
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Total: %lfms", (dr_full_ns / 1e6));

            if(is_save_data_)
            {
                data_save_ << setprecision(3) << (float)(dr_infer_ns / 1e6) << endl;
            }
        }
        if(debug_params_.print_target_info)
        {
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "-----------INFO------------");
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Yaw: %lf", angle[0]);
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Pitch: %lf", angle[1]);
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Dist: %fm", (float)target.armor3d_cam.norm());
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Target: %s", target.key.c_str());
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Target Type: %s", (char *)(target.type == SMALL ? "SMALL" : "BIG"));
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Is Spinning: %d", (int)(is_target_spinning));
            RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 20, "Is Switched: %d", (int)(is_target_switched_));
        }
        return true;
    }

    /**
     * @brief 显示检测到的装甲板信息
     * 
     * @param src 图像数据结构体
     */
    void Detector::showArmors(TaskData& src)
    {
        for (auto armor : new_armors_)
        {
            char ch[10];
            sprintf(ch, "%.3f", armor.conf);
            std::string conf_str = ch;
            putText(src.img, conf_str, armor.apex2d[3], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

            std::string id_str = to_string(armor.id);
            if (armor.color == BLUE_SMALL)
                putText(src.img, "BS" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
            else if (armor.color == BLUE_BIG)
                putText(src.img, "BB" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
            else if (armor.color == GRAY_SMALL)
                putText(src.img, "NS" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 0, 255}, 2);
            else if (armor.color == GRAY_BIG)
                putText(src.img, "NB" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 255}, 2);
            else if (armor.color == RED_SMALL)
                putText(src.img, "RS" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
            else if (armor.color == RED_BIG)
                putText(src.img, "RB" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {125, 255, 125}, 2);
            else if (armor.color == PURPLE_SMALL)
                putText(src.img, "PS" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 0, 255}, 2);
            else if (armor.color == PURPLE_BIG)
                putText(src.img, "PB" + id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {125, 0, 125}, 2);

            for(int i = 0; i < 4; i++)
            {
                putText(src.img, to_string(i), armor.apex2d[i % 4], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
                line(src.img, armor.apex2d[i % 4], armor.apex2d[(i + 1) % 4], {0,255,0}, 1);
            }
            rectangle(src.img, armor.roi, {255, 0, 255}, 1);
        }
    }

    int Detector::chooseTargetID(TaskData &src, std::vector<Armor>& armors, ObjHPMsg hps, DecisionMsg decision_msg)
    {
        std::vector<Armor> new_armors;
        cv::Point2d img_center = cv::Point2d(src.img.size().width / 2, src.img.size().height / 2);
        if (src.mode == SENTRY_NORMAL)
        {
            for (auto& armor : armors)
            {
                int id = car_id_map_[armor.key];
                if (armor.id == decision_msg.decision_id && armor.armor3d_world.norm() <= detector_params_.fire_zone)
                {
                    return armor.id;
                }
                else if (hps.hp[id] < 50 && id < 10 && armor.armor3d_world.norm() <= detector_params_.fire_zone)
                {   //检测到目标且血量过低，且在开火范围内
                    //TODO:将此目标作为待击打目标
                    // new_armors.emplace_back(armor);
                    return armor.id;
                }
                else if (armor.id == 1 && armor.armor3d_world.norm() <= detector_params_.hero_danger_zone)
                {   //检测到英雄机器人且在危险距离内，直接作为待打击目标
                    return armor.id;
                }
                else if (armor.id == last_armor_.id && (now_ - last_timestamp_) / 1e6 <= 100 && armor.armor3d_world.norm() <= detector_params_.fire_zone)
                {   //若存在上帧目标且目标在开火范围内，则直接返回
                    return armor.id;
                }
                else if (armor.armor3d_world.norm() < detector_params_.fire_zone)
                {   //若以上条件均不满足，则将开火范围内的目标作为候选目标
                    // min_dist = dist;
                    new_armors.emplace_back(armor);
                }
            }

            double min_dist = 1e9; //FIXME:给最大阈值
            int max_area = 0; //FIXME:给最小阈值
            int idx = -1;
            int target_idx = 0;
            bool is_target_exists = false;
            for (auto& armor : new_armors)
            {
                idx++;
                double dist = abs(img_center.x - armor.center2d.x);
                if (dist < min_dist && dist <= detector_params_.fire_zone)
                {
                    min_dist = dist;
                    is_target_exists = true;
                    target_idx = idx;
                }
                else if (dist / min_dist >= 0.6 && armor.area > max_area)
                {
                    min_dist = dist;
                    max_area = armor.area;
                    is_target_exists = true;
                    target_idx = idx;
                }
            }
            if (is_target_exists)
                return new_armors[target_idx].id;
            else
                return -1;
        }
        else if (src.mode == AUTOAIM_TRACKING || src.mode == AUTOAIM_NORMAL || src.mode == AUTOAIM_SLING)
        {
            return chooseTargetID(src, armors);
        }
        else
        {
            return -1;
        }
    }

    /**
     * @brief 图像ROI裁剪
     * 
     * @param img 原图像
     * @return Point2i 裁剪的偏移量
     */
    Point2i Detector::cropImageByROI(Mat &img)
    {
        double area_ratio = last_target_area_ / img.size().area();
        
        //若上次不存在目标
        if (!is_last_target_exists_)
        {
            //当丢失目标帧数过多或lost_cnt为初值
            if (lost_cnt_ > detector_params_.max_lost_cnt || lost_cnt_ == 0 || area_ratio == 0.0)
            {
                return Point2i(0,0);
            }
        }
        
        //若目标大小大于阈值
        if (area_ratio > detector_params_.no_crop_ratio)
        {
            return Point2i(0,0);
        }

        int max_expand = (img.size().height - input_size_.width) / 32;
        double cropped_ratio = (detector_params_.no_crop_ratio / detector_params_.full_crop_ratio) / max_expand;
        int expand_value = ((int)(area_ratio / detector_params_.full_crop_ratio / cropped_ratio)) * 32;
        Size2i cropped_size = input_size_ + Size2i(expand_value, expand_value);
        // Size2i crooped_size = (input_size + (no_crop_thres / max))

        //处理X越界
        if (last_roi_center_.x <= cropped_size.width / 2)
            last_roi_center_.x = cropped_size.width / 2;
        else if (last_roi_center_.x >= (img.size().width - cropped_size.width / 2))
            last_roi_center_.x = img.size().width - cropped_size.width / 2;
        //处理Y越界
        if (last_roi_center_.y <= cropped_size.height / 2)
            last_roi_center_.y = cropped_size.height / 2;
        else if (last_roi_center_.y >= (img.size().height - cropped_size.height / 2))
            last_roi_center_.y = img.size().height - cropped_size.height / 2;
        
        //左上角顶点
        auto offset = last_roi_center_ - Point2i(cropped_size.width / 2, cropped_size.height / 2);
        // auto offset = last_roi_center_ - Point2i(roi_width / 2, roi_height / 2);
        Rect roi_rect = Rect(offset, cropped_size);
        img(roi_rect).copyTo(img);

        return offset;
    }

    /**
     * @brief 选择目标车辆装甲板对应的追踪器Tracker
     * 
     * @param trackers 追踪器队列
     * @param timestamp 当前帧对应的时间戳
     * @return ArmorTracker* 返回目标车辆装甲板对应的追踪器
     */
    ArmorTracker* Detector::chooseTargetTracker(TaskData& src, vector<ArmorTracker*> trackers)
    {
        //TODO:增加对打击后暂时熄灭装甲板的防抖处理
        float max_score = 0;
        int target_idx = 0;
        int last_target_idx = -1;
        for (int i = 0; i < (int)(trackers.size()); i++)
        {
            //计算tracker的切换打击分数,由装甲板旋转角度,距离,面积大小决定
            if ((trackers[i]->now / 1e9) == (now_ / 1e9))
            {
                if (trackers[i]->is_initialized && (trackers[i]->last_selected_timestamp / 1e9) == (last_timestamp_ / 1e9) 
                && (abs(now_ - last_timestamp_) / 1e6) < 100)
                {
                    last_target_idx = i;
                    break;
                }
                else if (trackers[i]->hit_score > max_score)
                {
                    max_score = trackers[i]->hit_score;
                    target_idx = i;
                }
            }
        }
        //若存在上次存在目标且分数与相差不大，选择该装甲板
        // if (last_target_idx != -1 && abs(trackers[last_target_idx]->hit_score - max_score) / max_score < 0.1)
        //     target_idx = last_target_idx;
        if (last_target_idx != -1)
            target_idx = last_target_idx;
        return trackers[target_idx];
    }   

    /**
     * @brief 选择目标车辆
     * 
     * @param armors 当前帧检测到的所有装甲板对象
     * @param timestamp 当前帧对应的时间戳
     * @return int 返回选择的车辆ID
     */
    int Detector::chooseTargetID(TaskData& src)
    {
        /*
        该选择逻辑主要存在四层约束：前哨站旋转模式约束/英雄约束/上次目标约束/距离位置约束
        1.若进入前哨站旋转装甲模式，检测到目标旋转装甲直接退出；
        2.若检测到存在上次击打目标,时间较短,且该目标运动较小,则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID；
        3.若检测到危险距离内的英雄直接退出循环；
        4.若不存在上次打击目标且未检测到危险距离内的英雄目标，则选择当前距离最近，距视野中心较近的目标作为击打目标；
        5.若上面四层约束都不满足，则返回当前视野中面积最大的。
        */
        bool is_last_id_exists = false;
        int target_id = -1;
        // double min_2d_dist = 1e4;
        float min_rrangle = 1e2;
        float max_rrangle = 0.0;
        double min_3d_dist = 1e2;
        for (auto& armor : new_armors_)
        {   
            // RCLCPP_INFO_THROTTLE(logger_, steady_clock_, 500, "armor_area:%d", armor.area);
            // double dist_2d = abs((src.img.size().width / 2.0) - armor.center2d.x);
            float rrangle = armor.rrect.angle;
            double dist_3d = armor.armor3d_world.norm();

            if (armor.id == 6)
            {
                return armor.id;
            }
            else if (armor.id == 1 && armor.armor3d_world.norm() <= detector_params_.hero_danger_zone)
            {
                return armor.id;
            }
            else if ((armor.id == last_armor_.id 
            || last_armor_.roi.contains(armor.center2d)
            )
            && abs(armor.area - last_armor_.area) / (float)armor.area < 0.40
            && abs(now_ - last_timestamp_) / 1e6 <= 100
            && (abs(rrangle) <= 16.0 || abs(rrangle) >= 74.0))
            {
                armor.id = last_armor_.id;
                is_last_id_exists = true;
                target_id = armor.id;
                break;
            }
            else if (abs(rrangle) >= 13.0 && abs(rrangle) <= 77.0)
            {
                continue;
            }
            else if ((abs(rrangle) > 77.0 || abs(rrangle) < 13.0) && (abs(rrangle) > abs(max_rrangle) || abs(rrangle) < abs(min_rrangle)))
            {
                min_rrangle = abs(rrangle) < 13.0 ? rrangle : 1e2;
                max_rrangle = abs(rrangle) > 77.0 ? rrangle : 0.0; 
                is_last_id_exists = true;
                target_id = armor.id;
            }
            // else if (dist_2d < min_2d_dist && dist_3d / min_3d_dist >= 0.6 && dist_3d <= detector_params_.fire_zone)
            // {
            //     is_last_id_exists = true;
            //     target_id = armor.id;
            // }   
            else if (dist_3d < min_3d_dist && dist_3d <= detector_params_.fire_zone)
            {
                is_last_id_exists = true;
                target_id = armor.id;
            }
        }
        
        //若不存在则返回面积最大的装甲板序号，即队列首元素序号
        if (is_last_id_exists)
        {
            return target_id;
        }
        else
        {
            return (*new_armors_.begin()).id;
        }
    }
} //namespace Detector