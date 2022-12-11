/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:26:16
 * @LastEditTime: 2022-12-09 10:08:47
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/armor_detector/detector.cpp
 */
#include "../../include/armor_detector/detector.hpp"

using namespace std;

namespace armor_detector
{
    detector::detector(const std::string& camera_name_, const std::string& camera_param_path_, const std::string& network_path_,
    const detector_params& _detector_params_, const debug_params& _debug_params_, const gyro_params& _gyro_params_) 
    : spinning_detector_(_detector_params_.color, _gyro_params_), detector_params_(_detector_params_), debug_params_(_debug_params_)
    {
        //参数设置
        this->camera_name = camera_name_;
        this->camera_param_path = camera_param_path_;
        this->network_path = network_path_;
        // this->detector_params_.dw = _detector_params_.dw;
        // this->detector_params_.dh = _detector_params_.dh;
        // this->detector_params_.rescale_ratio = _detector_params_.rescale_ratio;
        // this->detector_params_.armor_type_wh_thres = _detector_params_.armor_type_wh_thres;
        // this->detector_params_.max_lost_cnt = _detector_params_.max_lost_cnt;
        // this->detector_params_.max_armors_cnt = _detector_params_.max_armors_cnt;
        // this->detector_params_.max_v = _detector_params_.max_v;
        // this->detector_params_.max_delta_t = _detector_params_.max_delta_t;
        // this->detector_params_.no_crop_thres = _detector_params_.no_crop_thres;
        // this->detector_params_.hero_danger_zone = _detector_params_.hero_danger_zone;
                   
        //网络模型初始化
        
        //默认值
        lost_cnt = 0;
        is_last_target_exists = false;
        is_target_switched = false;
        last_target_area = 0;
        last_bullet_speed = 0;
        input_size = {720, 720};
        is_init = false;

        is_save_data = false; //save distance error data

        //debug
        // this->debug_params_.debug_without_com = _debug_params_.debug_without_com;
        // this->debug_params_.using_imu =  _debug_params_.using_imu;
        // this->debug_params_.using_roi =  _debug_params_.using_roi;
        // this->debug_params_.show_aim_cross = _debug_params_.show_aim_cross;
        // this->debug_params_.show_img = _debug_params_.show_img;
        // this->debug_params_.detect_red = _debug_params_.detect_red;
       
        //gyro_detect_params
        // spinning_detector_ = spinning_detector(_detector_params_.color, _gyro_params_);
        last_period_ = 0;
        last_last_status_ = last_status_ = cur_status_ = NONE;
        save_image_ = false;
    }

    detector::~detector()
    {
        if(is_save_data)
        {
            data_save.close();
        }
    }

    void detector::debugParams(const detector_params& detector_params, const debug_params& debug_params, const gyro_params& gyro_params)
    {
        //detector params
        this->detector_params_.dw = detector_params.dw;
        this->detector_params_.dh = detector_params.dh;
        this->detector_params_.rescale_ratio = detector_params.rescale_ratio;
        this->detector_params_.armor_type_wh_thres = detector_params.armor_type_wh_thres;
        this->detector_params_.max_lost_cnt = detector_params.max_lost_cnt;
        this->detector_params_.max_armors_cnt = detector_params.max_armors_cnt;
        this->detector_params_.max_v = detector_params.max_v;
        this->detector_params_.max_delta_t = detector_params.max_delta_t;
        this->detector_params_.no_crop_thres = detector_params.no_crop_thres;
        this->detector_params_.hero_danger_zone = detector_params.hero_danger_zone;

        //debug
        this->debug_params_.debug_without_com = debug_params.debug_without_com;
        this->debug_params_.using_imu =  debug_params.using_imu;
        this->debug_params_.using_roi =  debug_params.using_roi;
        this->debug_params_.show_aim_cross = debug_params.show_aim_cross;
        this->debug_params_.show_img = debug_params.show_img;
        this->debug_params_.detect_red = debug_params.detect_red;
        this->debug_params_.print_letency = debug_params.print_letency;
        this->debug_params_.print_target_info = debug_params.print_target_info;
    }
    
    bool detector::armor_detect(global_user::TaskData &src)
    {
        if(!is_init)
        {
            detector_.initModel(network_path);
            coordsolver_.loadParam(camera_param_path, camera_name);

            if(is_save_data)
            {
                data_save.open("src/data/dis_info_1.txt", ios::out | ios::trunc);
                data_save << fixed;
            }

            is_init = true;
        }

        time_start = std::chrono::steady_clock::now();
        auto input = src.img;
        timestamp = src.timestamp;
        
        if(!debug_params_.debug_without_com)
        {   //有串口
            //设置弹速,若弹速大于10m/s值,且弹速变化大于0.5m/s则更新
            if (src.bullet_speed > 10 && abs(src.bullet_speed - last_bullet_speed) > 0.5)
            {
                double bullet_speed;
                if (abs(src.bullet_speed - last_bullet_speed) > 0.5)
                    bullet_speed = src.bullet_speed;
                else
                    bullet_speed = (last_bullet_speed + src.bullet_speed) / 2;
                
                coordsolver_.setBulletSpeed(bullet_speed);
                last_bullet_speed = bullet_speed;
            }
        }

        // Eigen::Matrix3d rmat_imu;
        if(!debug_params_.using_imu)
        {   //使用陀螺仪数据
            rmat_imu = src.quat.toRotationMatrix();
        }
        else
        {
            rmat_imu = Eigen::Matrix3d::Identity();
        }

        // std::cout << 2 << std::endl;
        if(debug_params_.using_roi)
        {   //启用roi
            //吊射模式采用固定ROI
            if (src.mode == 2)
            {
                input(Range(600,1024),Range(432,848)).copyTo(input);
                roi_offset = Point2f((float)432, (float)600);
            }
            else
            {
                roi_offset = cropImageByROI(input);
            }
        }

        time_crop = std::chrono::steady_clock::now();

        objects.clear();
        armors.clear();
        
        if(!detector_.detect(input, objects))
        {   // 若未检测到目标
            if(debug_params_.show_aim_cross)
            {
                line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0,255,0}, 1);
                line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255,0}, 1);
            }
            if(debug_params_.show_img)
            {
                namedWindow("dst",0);
                imshow("dst",src.img);
                waitKey(1);
            }

            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0.0;
            return false;
        }

        time_infer = std::chrono::steady_clock::now();
        
        // 将对象排序，保留面积较大的对象
        sort(objects.begin(),objects.end(),[](ArmorObject& prev, ArmorObject& next)
        {
            return prev.area > next.area;
        });

        // 若对象较多保留前按面积排序后的前max_armors个
        if (objects.size() > this->detector_params_.max_armors_cnt)
            objects.resize(this->detector_params_.max_armors_cnt);
        
        // 生成装甲板对象
        for (auto object : objects)
        {
            if(detector_params_.color == RED)
            {
                if (object.color != 1)
                    continue;
            }
            else
            {
                if (object.color != 0)
                    continue;
            }
   
            Armor armor;
            armor.id = object.cls;
            armor.color = object.color;
            armor.conf = object.prob;
            if (object.color == 0)
                armor.key = "B" + to_string(object.cls);
            if (object.color == 1)
                armor.key = "R" + to_string(object.cls);
            if (object.color == 2)
                armor.key = "N" + to_string(object.cls);
            if (object.color == 3)
                armor.key = "P" + to_string(object.cls);
            memcpy(armor.apex2d, object.apex, 4 * sizeof(cv::Point2f));
            for(int i = 0; i < 4; i++)
            {
                armor.apex2d[i] += Point2f((float)roi_offset.x,(float)roi_offset.y);
            }
            Point2f apex_sum;
            for(auto apex : armor.apex2d)
                apex_sum +=apex;
            armor.center2d = apex_sum / 4.f;

            // auto pnp_result = coordsolver.pnp(armor.apex2d, rmat_imu, SOLVEPNP_ITERATIVE);

            // std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
            
            // std::vector<Point2f> tmp;
            // for (auto rrect.)
            // global_user::TargetType target_type = global_user::SMALL;

            // //计算长宽比,确定装甲板类型
            // RotatedRect points_pic_rrect = minAreaRect(points_pic);
            // auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
            //                         min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            
            // //若大于长宽阈值或为哨兵、英雄装甲板
            // if (apex_wh_ratio > this->detector_params_.armor_type_wh_thres || object.cls == 1 || object.cls == 0)
            //     target_type = global_user::BIG;
            // for (auto pic : points_pic)
            //     cout<<pic<<endl;
            // cout<<target_type<<endl;

            //生成装甲板旋转矩形和ROI
            std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
            RotatedRect points_pic_rrect = minAreaRect(points_pic);        
            armor.rrect = points_pic_rrect;
            auto bbox = points_pic_rrect.boundingRect();
            auto x = bbox.x - 0.5 * bbox.width * (detector_params_.armor_roi_expand_ratio_width - 1);
            auto y = bbox.y - 0.5 * bbox.height * (detector_params_.armor_roi_expand_ratio_height - 1);
            armor.roi = Rect(x,
                            y,
                            bbox.width * detector_params_.armor_roi_expand_ratio_width,
                            bbox.height * detector_params_.armor_roi_expand_ratio_height
                            );
            //若装甲板置信度小于高阈值，需要相同位置存在过装甲板才放行
            if (armor.conf < this->detector_params_.armor_conf_high_thres)
            {
                if (last_armors.empty())
                {
                    continue;
                }
                else
                {
                    bool is_this_armor_available = false;
                    for (auto last_armor : last_armors)
                    {
                        if (last_armor.roi.contains(armor.center2d))
                        {
                            is_this_armor_available = true;
                            break;
                        }
                    }
                    if (!is_this_armor_available)
                    {
                        continue;
                        std::cout << "IGN" << std::endl;
                    }
                }
            }
            //进行PnP，目标较少时采取迭代法，较多时采用IPPE
            int pnp_method;
            if (objects.size() <= 2)
                pnp_method = SOLVEPNP_ITERATIVE;
            else
                pnp_method = SOLVEPNP_IPPE;
            global_user::TargetType target_type = global_user::SMALL;

            //计算长宽比,确定装甲板类型
            auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
                                    min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            //若大于长宽阈值或为哨兵、英雄装甲板
            if (object.cls == 1 || object.cls == 0)
                target_type = global_user::BIG;

            //FIXME：若存在平衡步兵需要对此处步兵装甲板类型进行修改
            else if (object.cls == 2 || object.cls == 3 || object.cls == 4 || object.cls == 5 || object.cls == 6)
                target_type = global_user::SMALL;
            else if(apex_wh_ratio > detector_params_.armor_type_wh_thres)
                target_type = global_user::BIG;

            //单目PnP
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_ITERATIVE);
            // auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
            
            //防止装甲板类型出错导致解算问题，首先尝试切换装甲板类型，若仍无效则直接跳过该装甲板
            if (pnp_result.armor_cam.norm() > 10 ||
                isnan(pnp_result.armor_cam[0]) ||
                isnan(pnp_result.armor_cam[1]) ||
                isnan(pnp_result.armor_cam[2]))
            {
                if (target_type == global_user::SMALL)
                    target_type = global_user::BIG;
                else if (target_type == global_user::BIG)
                    target_type = global_user::SMALL;
                pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_ITERATIVE);
                // pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
                if (pnp_result.armor_cam.norm() > 10 ||
                    isnan(pnp_result.armor_cam[0]) ||
                    isnan(pnp_result.armor_cam[1]) ||
                    isnan(pnp_result.armor_cam[2]))
                    {
                        continue;
                    }
            }

            // double yy = pnp_result.armor_cam[2];
            // pnp_result.armor_cam[2] = pnp_result.armor_cam[1];
            // pnp_result.armor_cam[1] = yy;
            
            // std::cout << "x:" << pnp_result.armor_cam[0] << " y:" << pnp_result.armor_cam[1] << " z:" << pnp_result.armor_cam[2] << std::endl;

            armor.center3d_world = pnp_result.armor_world;
            armor.center3d_cam = pnp_result.armor_cam;
            armor.euler = pnp_result.euler;
            armor.area = object.area;
            armors.push_back(armor);
        }
        
        //若无合适装甲板
        if (armors.empty())
        {
            // std::cout << "No suitable targets..." << std::endl;

            if(debug_params_.show_aim_cross)
            {
                line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
                line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
            }
            if(debug_params_.show_img)
            {
                namedWindow("dst",0);
                imshow("dst",src.img);
                waitKey(1);
            }

            //更新陀螺分数
            spinning_detector_.update_spin_score();

            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0;
            return false;
        }
        else
        {
            last_armors = armors;
        }

        return true;
    }

    bool detector::gyro_detector(global_user::TaskData &src, global_interface::msg::Target& target_info, TargetInfoPtr& target_ptr)
    {
        /**
         * @brief 车辆小陀螺状态检测
        */

        //Create ArmorTracker for new armors 
        spinning_detector_.create_armor_tracker(trackers_map, armors, new_armors_cnt_map, timestamp, dead_buffer_cnt);

        //Detect armors status
        spinning_detector_.is_spinning(trackers_map, new_armors_cnt_map, timestamp);

        //Update spinning score
        spinning_detector_.update_spin_score();
        // std::cout << 9 << std::endl;

        //Choose target vehicle
        auto target_id = chooseTargetID(armors, timestamp);
        // std::cout << 10 << std::endl;

        string target_key;
        if (detector_params_.color == BLUE)
            target_key = "B" + to_string(target_id);
        else if (detector_params_.color == RED)
            target_key = "R" + to_string(target_id);

        ///-----------------------------detect whether exists matched tracker------------------------------------------
        if (trackers_map.count(target_key) == 0)
        {
            if(debug_params_.show_aim_cross)
            {
                line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
                line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
            }
            if(debug_params_.show_img)
            {
                namedWindow("dst",0);
                imshow("dst",src.img);
                waitKey(1);
            }

            lost_cnt++;
            is_last_target_exists = false;
            // LOG(WARNING) <<"[AUTOAIM] No available tracker exists!";
            return false;
        }
        auto ID_candiadates = trackers_map.equal_range(target_key);
        // std::cout << 14 << std::endl;

        ///---------------------------acqusition final armor's sequences---------------------------------------
        bool is_target_spinning;
        Armor target;
        std::vector<ArmorTracker*> final_trackers;
        std::vector<Armor> final_armors;
        //TODO:反陀螺防抖(增加陀螺模式与常规模式)
        //若目标处于陀螺状态，预先瞄准目标中心，待预测值与该点距离较近时开始击打
        SpinHeading spin_status;
        if (spinning_detector_.spin_status_map.count(target_key) == 0)
        {
            spin_status = UNKNOWN;
            is_target_spinning = false;
            target_info.is_spinning = false;
        }
        else
        {
            spin_status = spinning_detector_.spin_status_map[target_key];
            if (spin_status != UNKNOWN)
            {
                is_target_spinning = true;
                target_info.is_spinning = true;
            }
            else
            {
                is_target_spinning = false;
                target_info.is_spinning = false;
            }
        }


        ///----------------------------------反陀螺击打---------------------------------------
        target_ptr->is_spinning = false;

        target_info.is_spinning = false;
        target_info.is_still_spinning = false;
        if (spin_status != UNKNOWN)
        {
            //------------------------------尝试确定旋转中心-----------------------------------
            auto available_candidates_cnt = 0;
            for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
            {
                // std::cout << 15 << std::endl;
                if ((*iter).second.last_timestamp == src.timestamp)
                {
                    final_armors.push_back((*iter).second.last_armor);
                    final_trackers.push_back(&(*iter).second);
                }
                else
                {
                    continue;
                }
            }
            // std::cout << 16 << std::endl;
            
            target_ptr->is_spinning = false;
            target_info.is_spinning = false;
            target_info.is_still_spinning = false;
            
            auto cnt = spinning_detector_.spinning_x_map.count(target_key);
            // std::cout << 17 << std::endl;
            
            // if(target_info.spinning_switched)
            // {
            //     cur_ave_period_ = 0;
            //     last_period_ = 0;
            //     history_period_.clear();
            // }

            if(cnt == 1)
            {
                auto candidate = spinning_detector_.spinning_x_map.find(target_key);

                cur_period_ = ((*candidate).second.new_timestamp - (*candidate).second.last_timestamp) / 1e3;
                double w = (2 * M_PI) / (4 * cur_period_);
                
                if(last_period_ == 0)
                {
                    last_period_ = cur_period_;
                }

                if(history_period_.size() < 3)
                {
                    history_period_.push_back(cur_period_);
                }
                // std::cout << std::endl;
                // std::cout << "period:" << cur_period_ << std::endl;
                double per_sum = 0;

                last_ave_period_ = cur_ave_period_;
                if(history_period_.size() >= 3)
                {
                    for(auto &per : history_period_)
                    {
                        per_sum += per;
                    }
                    
                    cur_ave_period_ = (per_sum / history_period_.size());
                    
                    // std::cout << std::endl;
                    // std::cout << " T:" << cur_ave_period_ << std::endl;
                    // std::cout << std::endl;

                    if(abs(cur_ave_period_ - cur_period_) < 0.05)
                    {
                        last_period_ = cur_period_;
                        if(history_period_.size() < 9)
                        {
                            history_period_.push_back(cur_period_);
                        }
                        else
                        {
                            history_period_.pop_front();
                        }
                    }
                }

                if(last_ave_period_ == cur_ave_period_) 
                    target_info.period = last_ave_period_;
                else if(last_ave_period_ != cur_ave_period_ != 0)
                    target_info.period = cur_ave_period_;

                // std::cout << "period: " << target_info.period << std::endl;
                
                target_info.w = w;
                
                double delta_x_back = abs((*candidate).second.new_x_back - (*candidate).second.last_x_back);
                double delta_x_font = abs((*candidate).second.new_x_font - (*candidate).second.last_x_font);
                int delta_x_back_2d = abs((*candidate).second.new_x_back_2d - (*candidate).second.last_x_back_2d);
                int delta_x_font_2d = abs((*candidate).second.new_x_font_2d - (*candidate).second.last_x_font_2d);
                double ave_x_3d = (delta_x_back + delta_x_font) / 2;
                double ave_x_2d = (delta_x_back_2d + delta_x_font_2d) / 2;

                // std::cout << std::endl;
                // std::cout << "delta_x_back: " << delta_x_back << std::endl;
                // std::cout << "delta_x_font: " << delta_x_font << std::endl;
                // std::cout << "x_ave: " << (delta_x_back + delta_x_font) / 2 << std::endl;
                // std::cout << std::endl;
                
                // std::cout << "delta_x_back_2d: " << delta_x_back_2d << std::endl; 
                // std::cout << "delta_x_font_2d: " << delta_x_font_2d << std::endl;
                // std::cout << "x_2d_ave: " << (delta_x_back_2d + delta_x_font_2d) / 2 << std::endl;
                // std::cout << std::endl;
                
                if((ave_x_2d > spinning_detector_.gyro_params_.delta_x_2d_higher_thresh || ave_x_3d > spinning_detector_.gyro_params_.delta_x_3d_higher_thresh) 
                || (ave_x_2d > spinning_detector_.gyro_params_.delta_x_2d_high_thresh && ave_x_3d > spinning_detector_.gyro_params_.delta_x_3d_high_thresh))
                // if(ave_x_2d > 51)
                {
                    target_ptr->is_spinning = true;
                    target_ptr->spinning_status = MOVEMENT_SPINNING;
                    
                    target_info.is_spinning = true;
                    target_info.is_still_spinning = false;
                }
                else if((ave_x_2d < spinning_detector_.gyro_params_.delta_x_2d_lower_thresh) 
                || (ave_x_2d < spinning_detector_.gyro_params_.delta_x_2d_low_thresh && ave_x_3d < spinning_detector_.gyro_params_.delta_x_3d_low_thresh))
                // else if(ave_x_2d < 48)
                {
                    target_ptr->is_spinning = true;
                    target_ptr->spinning_status = STILL_SPINNING;

                    target_info.is_still_spinning = true;
                    target_info.is_spinning = false;
                }
                else
                {
                    target_ptr->is_spinning = true;
                }
            }

            //若存在一块装甲板
            if (final_armors.size() == 1)
            {
                if(spin_status == CLOCKWISE)
                {
                    target_info.clockwise = CLOCKWISE;
                    target_ptr->is_clockwise = CLOCKWISE;
                }
                else
                {
                    target_info.clockwise = false;
                    target_ptr->is_clockwise = false;
                }
                
                if(save_image_)
                {
                    if(!cur_frame_.empty())
                    {
                        cur_frame_.copyTo(last_frame_);
                        src.img.copyTo(cur_frame_);
                    }
                    else
                    {
                        src.img.copyTo(cur_frame_);
                    }
                }

                last_last_status_ = last_status_;
                last_status_ = cur_status_;
                cur_status_ = SINGER;

                // std::cout << "one..." << std::endl;
                target = final_armors.at(0);
            }
            //若存在两块装甲板
            else if (final_armors.size() == 2)
            {
                if(save_image_)
                {
                    if(!cur_frame_.empty())
                    {
                        cur_frame_.copyTo(last_frame_);
                        src.img.copyTo(cur_frame_);
                    }
                    else
                    {
                        src.img.copyTo(cur_frame_);
                    }
                }

                last_last_status_ = last_status_;
                last_status_ = cur_status_;
                cur_status_ = DOUBLE;

                //对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                sort(final_armors.begin(),final_armors.end(),[](Armor& prev, Armor& next)
                                    {return prev.center3d_cam[0] < next.center3d_cam[0];});
                //若顺时针旋转选取右侧装甲板更新
                if (spin_status == CLOCKWISE)
                {
                    target_info.clockwise = CLOCKWISE;
                    target_ptr->is_clockwise = CLOCKWISE;
                    // std::cout << "right..." << std::endl;
                    target = final_armors.at(1);
                }
                //若逆时针旋转选取左侧装甲板更新
                else if (spin_status == COUNTER_CLOCKWISE)
                {
                    target_info.clockwise = false;
                    target_ptr->is_clockwise = false;
                    // std::cout << "left..." << std::endl;
                    target = final_armors.at(0);
                }
            }

            //判断装甲板是否切换，若切换将变量置1
            auto delta_t = src.timestamp - prev_timestamp;
            auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
            auto velocity = (delta_dist / delta_t) * 1e3;
            if ((target.id != last_armor.id || !last_armor.roi.contains((target.center2d))) &&
                !is_last_target_exists)
            {
                is_target_switched = true;
                target_info.target_switched = true;
            }
            else
            {
                is_target_switched = false;
                target_info.target_switched = false;
            }

            target_info.point2d[0].x = target.apex2d[0].x;
            target_info.point2d[0].y = target.apex2d[0].y;
            target_info.point2d[1].x = target.apex2d[1].x;
            target_info.point2d[1].y = target.apex2d[1].y;
            target_info.point2d[2].x = target.apex2d[2].x;
            target_info.point2d[2].y = target.apex2d[2].y;
            target_info.point2d[3].x = target.apex2d[3].x;
            target_info.point2d[3].y = target.apex2d[3].y;
            target_info.aiming_point.x = target.center3d_cam[0];
            target_info.aiming_point.y = target.center3d_cam[1];
            target_info.aiming_point.z = target.center3d_cam[2];
        }
        else
        {
            cur_ave_period_ = 0;
            last_period_ = 0;
            last_last_status_ = last_status_ = cur_status_ = NONE;
            history_period_.clear();

            // std::cout << 20 << std::endl;
            for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
            {
                // final_armors.push_back((*iter).second.last_armor);
                final_trackers.push_back(&(*iter).second);
            }
            //进行目标选择
            auto tracker = chooseTargetTracker(final_trackers, src.timestamp);
            tracker->last_selected_timestamp = src.timestamp;
            tracker->selected_cnt++;
            target = tracker->last_armor;
            //判断装甲板是否切换，若切换将变量置1
            auto delta_t = src.timestamp - prev_timestamp;
            auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
            auto velocity = (delta_dist / delta_t) * 1e3;
            // cout<<(delta_dist >= max_delta_dist)<<" "<<!last_armor.roi.contains(target.center2d)<<endl;
            if ((target.id != last_armor.id || !last_armor.roi.contains((target.center2d))) && !is_last_target_exists)
            {
                is_target_switched = true;
                target_info.target_switched = true;
            }
            else
            {
                is_target_switched = false;
                target_info.target_switched = false;
            }

            target_info.point2d[0].x = target.apex2d[0].x;
            target_info.point2d[0].y = target.apex2d[0].y;
            target_info.point2d[1].x = target.apex2d[1].x;
            target_info.point2d[1].y = target.apex2d[1].y;
            target_info.point2d[2].x = target.apex2d[2].x;
            target_info.point2d[2].y = target.apex2d[2].y;
            target_info.point2d[3].x = target.apex2d[3].x;
            target_info.point2d[3].y = target.apex2d[3].y;

            target_info.aiming_point.x = target.center3d_cam[0];
            target_info.aiming_point.y = target.center3d_cam[1];
            target_info.aiming_point.z = target.center3d_cam[2];

            // std::cout << 21 << std::endl;

        }

        // std::cout << "period: " << target_info.period << std::endl;

        target_info.spinning_switched = false;
        if(last_status_ == SINGER && cur_status_ == DOUBLE)
        {
            // std::cout << 1 << std::endl;
            target_info.spinning_switched = true;
            // std::cout << target_info.target_switched << std::endl;
        }

        // std::cout << std::endl;
        // std::cout << "Target_switched: " << target_info.target_switched << std::endl;
        // std::cout << std::endl;

        // std::cout << "Spinning_status: " << spin_status << std::endl;
 
        if(save_image_)
        {
            // if(last_last_status_ == DOUBLE && last_status_ == SINGER && cur_status_ == DOUBLE)
            // {
                // char now[64];
                // std::time_t tt;
                // struct tm *ttime;
                // tt = time(nullptr);
                // ttime = localtime(&tt);
                // strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  // 以时间为名字
                // std::string now_string(now);
                // const std::string &storage_location = "src/vehicle_system/autoaim/armor_detector/image/";
                // std::string path(std::string(storage_location + now_string).append(".png"));
                
                std::string img_name = path_prefix + to_string(src.timestamp) + ".jpg";
                cv::imwrite(img_name, src.img);

                std::string label_name = path_prefix + to_string(src.timestamp) + ".txt";
                std::string content;

                int cls = 0;
                if(target.id == 7)
                    cls = 9 * target.color - 1;
                if(target.id != 7)
                    cls = target.id + target.color * 9;
                
                content.append(to_string(cls) + " ");
                for(auto apex : target.apex2d)
                {
                    content.append(to_string((apex.x - roi_offset.x) / input_size.width));
                    content.append(" ");
                    content.append(to_string((apex.y - roi_offset.y) / input_size.height));
                    content.append(" ");
                }
                content.pop_back();
                file.open(label_name, std::ofstream::app);
                file << content;
                file.close();
            // }
        }

        if (target.color == 2)
            dead_buffer_cnt++;
        else
            dead_buffer_cnt = 0;

        target_info.timestamp = src.timestamp;

        //获取装甲板中心与装甲板面积以下一次ROI截取使用
        last_roi_center = target.center2d;
        // last_roi_center = Point2i(512,640);
        last_armor = target;
        lost_cnt = 0;
        prev_timestamp = src.timestamp;
        last_target_area = target.area;
        last_aiming_point[0] = target_info.aiming_point.x;
        last_aiming_point[1] = target_info.aiming_point.y;
        last_aiming_point[2] = target_info.aiming_point.z;
        is_last_target_exists = true;
        last_armors.clear();
        last_armors = armors;

        // std::cout << 22 << std::endl;

        if(debug_params_.show_aim_cross)
        {
            line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0,255,0}, 1);
            line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255,0}, 1);
        }

        if(debug_params_.show_all_armors)
        {
            for (auto armor : armors)
            {
                putText(src.img, fmt::format("{:.2f}", armor.conf),armor.apex2d[3],FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);
                if (armor.color == 0)
                    putText(src.img, fmt::format("B{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
                if (armor.color == 1)
                    putText(src.img, fmt::format("R{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
                if (armor.color == 2)
                    putText(src.img, fmt::format("N{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
                if (armor.color == 3)
                    putText(src.img, fmt::format("P{}",armor.id),armor.apex2d[0],FONT_HERSHEY_SIMPLEX, 1, {255, 100, 255}, 2);
                for(int i = 0; i < 4; i++)
                    line(src.img, armor.apex2d[i % 4], armor.apex2d[(i + 1) % 4], {0,255,0}, 1);
                rectangle(src.img, armor.roi, {255, 0, 255}, 1);
                auto armor_center = coordsolver_.reproject(armor.center3d_cam);
                circle(src.img, armor_center, 4, {0, 0, 255}, 2);
            }
        }
        
        auto angle = coordsolver_.getAngle(last_aiming_point, rmat_imu);
        //若预测出错则直接世界坐标系下坐标作为击打点
        if (isnan(angle[0]) || isnan(angle[1]))
            angle = coordsolver_.getAngle(last_aiming_point, rmat_imu);
        auto time_predict = std::chrono::steady_clock::now();

        double dr_crop_ms = std::chrono::duration<double,std::milli>(time_crop - time_start).count();
        double dr_infer_ms = std::chrono::duration<double,std::milli>(time_infer - time_crop).count();
        // double dr_predict_ms = std::chrono::duration<double,std::milli>(time_predict - time_infer).count();
        double dr_full_ms = std::chrono::duration<double,std::milli>(time_predict - time_start).count();

        if(debug_params_.show_fps)
        {
            putText(src.img, fmt::format("FPS: {}", int(1000 / dr_full_ms)), {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
        }

        if(debug_params_.print_letency)
        {
            //降低输出频率，避免影响帧率
            if (count % 5 == 0)
            {
                fmt::print(fmt::fg(fmt::color::gray), "-----------TIME------------\n");
                fmt::print(fmt::fg(fmt::color::blue_violet), "Crop: {} ms\n"   ,dr_crop_ms);
                fmt::print(fmt::fg(fmt::color::golden_rod), "Infer: {} ms\n",dr_infer_ms);
                // fmt::print(fmt::fg(fmt::color::green_yellow), "Predict: {} ms\n",dr_predict_ms);
                fmt::print(fmt::fg(fmt::color::orange_red), "Total: {} ms\n",dr_full_ms);
            }
        }
        // cout<<target.center3d_world<<endl;
        // cout<<endl;
    
        if(debug_params_.print_target_info)
        {
            if (count % 5 == 0)
            {
                fmt::print(fmt::fg(fmt::color::gray), "-----------INFO------------\n");
                fmt::print(fmt::fg(fmt::color::blue_violet), "Yaw: {} \n",angle[0]);
                fmt::print(fmt::fg(fmt::color::golden_rod), "Pitch: {} \n",angle[1]);
                fmt::print(fmt::fg(fmt::color::green_yellow), "Dist: {} m\n",(float)target.center3d_cam.norm());
                fmt::print(fmt::fg(fmt::color::white), "Target: {} \n",target.key);
                fmt::print(fmt::fg(fmt::color::white), "Target Type: {} \n",target.type == global_user::SMALL ? "SMALL" : "BIG");
                fmt::print(fmt::fg(fmt::color::orange_red), "Is Spinning: {} \n",is_target_spinning);
                fmt::print(fmt::fg(fmt::color::orange_red), "Is Switched: {} \n",is_target_switched);

                if(is_save_data)
                {
                    data_save << setprecision(3) << (float)target.center3d_cam.norm() << endl;
                }

                count = 0;
            }
            else
            {
                count++;
            }
        }

        //若预测出错取消本次数据发送
        if (isnan(angle[0]) || isnan(angle[1]))
        {
            // LOG(ERROR)<<"NAN Detected! Data Transmit Aborted!";
            // std::cout << "3..." << std::endl;
            return false;
        }

        if(debug_params_.show_img)
        {
            namedWindow("dst",0);
            imshow("dst",src.img);
            waitKey(1);
        }
        // std::cout << 23 << std::endl;

        return true;
    }

    Point2i detector::cropImageByROI(Mat &img)
    {
        // if (!is_last_target_exists)
        // {
        //     //当丢失目标帧数过多或lost_cnt为初值
        //     if (lost_cnt > this->detector_params_.max_lost_cnt || lost_cnt == 0)
        //     {
        //         return Point2i(0,0);
        //     }
        // }

        // //若目标大小大于阈值
        // if ((last_target_area / img.size().area()) > this->detector_params_.no_crop_thres)
        // {
        //     return Point2i(0,0);
        // }
        // //处理X越界
        
        // // 计算上一帧roi中心在原图像中的坐标
        // Point2i last_armor_center = Point2i(last_roi_center.x - this->detector_params_.dw, last_roi_center.y - this->detector_params_.dh) * (1 / this->detector_params_.rescale_ratio);

        // float armor_h = global_user::calcDistance(last_armor.apex2d[0], last_armor.apex2d[1]);
        // float armor_w = global_user::calcDistance(last_armor.apex2d[1], last_armor.apex2d[2]);
        // int roi_width = MAX(armor_h, armor_w) * (1 / this->detector_params_.rescale_ratio);
        // int roi_height = MIN(armor_h, armor_w) * (1 / this->detector_params_.rescale_ratio);

        // //根据丢失帧数逐渐扩大ROI大小
        // if(lost_cnt == 2)
        // {   //丢失2帧ROI扩大3.2倍
        //     roi_width *= 1.2;
        //     roi_height *= 1.2;
        // }
        // else if(lost_cnt == 3)
        // {   //丢失3帧ROI扩大4.5倍
        //     roi_width *= 1.8;
        //     roi_height *= 1.8;
        // }
        // else if(lost_cnt == 4)
        // {   //丢失4帧ROI扩大6倍
        //     roi_width *= 2.2;
        //     roi_height *= 2.2;
        // }
        // else if(lost_cnt == 5)
        // {   //返回原图像
        //     return Point2i(0,0);
        // }

        // //防止roi越界
        // if(last_armor_center.x > img.size().width || last_armor_center.y > img.size().height)
        // {
        //     return Point2f(0, 0);
        // }
        // if(last_armor_center.x < 0 || last_armor_center.y < 0)
        // {
        //     return Point2f(0, 0);
        // }

        // if((last_armor_center.x + roi_width / 2)  > img.size().width)
        //     roi_width = (img.size().width - last_armor_center.x) * 2;
        
        // if(last_armor_center.y + roi_height / 2 > img.size().height)
        //     roi_height = (img.size().height - last_armor_center.y) * 2;

        // Point2f roi_left_top = Point2i(last_armor_center.x - roi_width / 2, last_armor_center.y - roi_height / 2);
        // Rect roi_rect = Rect(roi_left_top.x, roi_left_top.y, roi_width, roi_height);
        // img(roi_rect).copyTo(img);

        // return roi_left_top;

        // if (last_roi_center.x <= input_size.width / 2)
        //     last_roi_center.x = input_size.width / 2;
        // else if (last_roi_center.x > (img.size().width - input_size.width / 2))
        //     last_roi_center.x = img.size().width - input_size.width / 2;
        // //处理Y越界
        // if (last_roi_center.y <= input_size.height / 2)
        //     last_roi_center.y = input_size.height / 2;
        // else if (last_roi_center.y > (img.size().height - input_size.height / 2))
        //     last_roi_center.y = img.size().height - input_size.height / 2;

        // //左上角顶点
        // auto offset = last_roi_center - Point2i(input_size.width / 2, input_size.height / 2);
        // Rect roi_rect = Rect(offset, input_size);
        // img(roi_rect).copyTo(img);

        // return offset;

         //若上次不存在目标
        if (!is_last_target_exists)
        {
            //当丢失目标帧数过多或lost_cnt为初值
            if (lost_cnt > detector_params_.max_lost_cnt || lost_cnt == 0)
            {
                return Point2i(0,0);
            }
        }
        //若目标大小大于阈值
        auto area_ratio = last_target_area / img.size().area();
        int max_expand = (img.size().height - input_size.width) / 32;
        double cropped_ratio = (detector_params_.no_crop_ratio / detector_params_.full_crop_ratio) / max_expand;
        int expand_value = ((int)(area_ratio / detector_params_.full_crop_ratio / cropped_ratio)) * 32;

        Size2i cropped_size = input_size + Size2i(expand_value, expand_value);
        // cout<<cropped_size<<endl;
        // Size2i crooped_size = (input_size + (no_crop_thres / max))
        if (area_ratio > detector_params_.no_crop_ratio)
        {
            return Point2i(0,0);
        }

        //处理X越界
        if (last_roi_center.x <= cropped_size.width / 2)
            last_roi_center.x = cropped_size.width / 2;
        else if (last_roi_center.x > (img.size().width - cropped_size.width / 2))
            last_roi_center.x = img.size().width - cropped_size.width / 2;
        //处理Y越界
        if (last_roi_center.y <= cropped_size.height / 2)
            last_roi_center.y = cropped_size.height / 2;
        else if (last_roi_center.y > (img.size().height - cropped_size.height / 2))
            last_roi_center.y = img.size().height - cropped_size.height / 2;
        
        //左上角顶点
        auto offset = last_roi_center - Point2i(cropped_size.width / 2, cropped_size.height / 2);
        // auto offset = last_roi_center - Point2i(roi_width / 2, roi_height / 2);
        Rect roi_rect = Rect(offset, cropped_size);
        img(roi_rect).copyTo(img);

        return offset;
    }
    
    ArmorTracker* detector::chooseTargetTracker(vector<ArmorTracker*> trackers, int timestamp)
    {
        //TODO:优化打击逻辑
        //TODO:本逻辑为哨兵逻辑
        float max_area = 0;
        float min_horizonal_dist = 0;
        int target_idx = 0;

        //若存在上次tracker则直接返回,若不存在则装甲板面积最大的Tracker
        for (int i = 0; i < trackers.size(); i++)
        {
            auto horizonal_dist_to_center = abs(trackers[i]->last_armor.center2d.x - 640);
            if (trackers[i]->last_timestamp == timestamp)
            {
                //若该Tracker为上次Tracker且本次仍在更新,则直接使用该装甲板
                if (trackers[i]->last_selected_timestamp == last_timestamp)
                    return trackers[i];
                //若该装甲板面积较大且与目前最大面积差距较大，列为候选并记录该装甲板数据
                else if (trackers[i]->last_armor.area >= max_area && (trackers[i]->last_armor.area / max_area > 1.4 || trackers[i]->last_armor.area / max_area < 0.6))
                {
                    max_area = trackers[i]->last_armor.area;
                    min_horizonal_dist = horizonal_dist_to_center;
                    target_idx = i;
                }
                //若该装甲板面积较大且与目前最大面积差距较小，判断该装甲板与图像中心点的二维水平距离
                else if ((trackers[i]->last_armor.area / max_area < 1.4 || trackers[i]->last_armor.area / max_area > 0.6) && horizonal_dist_to_center < min_horizonal_dist)
                {
                    min_horizonal_dist = horizonal_dist_to_center;
                    target_idx = i;
                }
            }
        }
        return trackers[target_idx];
    }   

    int detector::chooseTargetID(vector<Armor> &armors, int timestamp)
    {
        //TODO:自瞄逻辑修改
        bool is_last_id_exists = false;
        int target_id;
        //该选择逻辑主要存在两层约束:
        //英雄约束与上次目标约束
        //若检测到危险距离内的英雄直接退出循环
        //若检测到存在上次击打目标,时间较短,且该目标运动较小,则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
        for (auto armor : armors)
        {
            //FIXME:该处需根据兵种修改
            //若视野中存在英雄且距离小于危险距离，直接选为目标
            if (armor.id == 1 && armor.center3d_world.norm() <= detector_params_.hero_danger_zone)
            {
                return armor.id;
            }
            //若存在上次击打目标,时间较短,且该目标运动较小则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
            else if (armor.id == last_armor.id && abs(armor.area - last_armor.area) / (float)armor.area < 0.3 && abs(timestamp - prev_timestamp) < 30)
            {
                is_last_id_exists = true;
                target_id = armor.id;
            }
        }
        //若不存在则返回面积最大的装甲板序号，即队列首元素序号
        if (is_last_id_exists)
            return target_id;
        else
            return (*armors.begin()).id;
    }
} //namespace detector