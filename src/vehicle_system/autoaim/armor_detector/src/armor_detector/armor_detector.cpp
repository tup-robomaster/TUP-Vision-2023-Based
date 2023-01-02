/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:26:16
 * @LastEditTime: 2023-01-03 01:01:03
 * @FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/src/armor_detector/armor_detector.cpp
 */
#include "../../include/armor_detector/armor_detector.hpp"

using namespace std;
namespace armor_detector
{
    Detector::Detector(const std::string& camera_name_, const std::string& camera_param_path_, const std::string& network_path_,
    const DetectorParam& detector_params_, const DebugParam& _debug_params_, const GyroParam& _gyro_params_) 
    : spinning_detector_(detector_params_.color, _gyro_params_), detector_params_(detector_params_), debug_params_(_debug_params_),
    logger_(rclcpp::get_logger("armor_detector"))
    {
        //参数设置
        this->camera_name = camera_name_;
        this->camera_param_path = camera_param_path_;
        this->network_path = network_path_;
        // this->armor_detector_params_.dw = _armor_detector_params_.dw;
        // this->armor_detector_params_.dh = _armor_detector_params_.dh;
        // this->armor_detector_params_.rescale_ratio = _armor_detector_params_.rescale_ratio;
        // this->armor_detector_params_.armor_type_wh_thres = _armor_detector_params_.armor_type_wh_thres;
        // this->armor_detector_params_.max_lost_cnt = _armor_detector_params_.max_lost_cnt;
        // this->armor_detector_params_.max_armors_cnt = _armor_detector_params_.max_armors_cnt;
        // this->armor_detector_params_.max_v = _armor_detector_params_.max_v;
        // this->armor_detector_params_.max_delta_t = _armor_detector_params_.max_delta_t;
        // this->armor_detector_params_.no_crop_thres = _armor_detector_params_.no_crop_thres;
        // this->armor_detector_params_.hero_danger_zone = _armor_detector_params_.hero_danger_zone;
                   
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
        // spinning_armor_detector_ = spinning_detector(_armor_detector_params_.color, _gyro_params_);
    }

    Detector::~Detector()
    {
        if(is_save_data)
        {
            data_save.close();
        }
    }

    void Detector::debugParams(const DetectorParam& detector_params, const DebugParam& debug_params, const GyroParam& gyro_params) 
    {
        //Detector params
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
        this->debug_params_.using_imu = debug_params.using_imu;
        this->debug_params_.using_roi = debug_params.using_roi;
        this->debug_params_.show_aim_cross = debug_params.show_aim_cross;
        this->debug_params_.show_img = debug_params.show_img;
        this->debug_params_.detect_red = debug_params.detect_red;
        this->debug_params_.print_letency = debug_params.print_letency;
        this->debug_params_.print_target_info = debug_params.print_target_info;
    }
    
    bool Detector::armor_detect(TaskData &src)
    {
        if(!is_init)
        {
            armor_detector_.initModel(network_path);
            coordsolver_.loadParam(camera_param_path, camera_name);
            if(is_save_data)
            {
                data_save.open("src/data/dis_info_1.txt", ios::out | ios::trunc);
                data_save << fixed;
            }
            is_init = true;
        }

        time_start = steady_clock_.now();

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
        if(debug_params_.using_imu)
        {   //使用陀螺仪数据
            rmat_imu = src.quat.toRotationMatrix();
            RCLCPP_INFO(logger_, "Using imu...");
        }
        else
        {
            rmat_imu = Eigen::Matrix3d::Identity();
            RCLCPP_INFO(logger_, "No imu...");
        }

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
            RCLCPP_INFO(logger_, "Using roi...");
        }

        time_crop = steady_clock_.now();

        objects.clear();
        armors.clear();
        
        if(!armor_detector_.detect(input, objects))
        {   //若未检测到目标
            if(debug_params_.show_aim_cross)
            {
                line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0,255,0}, 1);
                line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255,0}, 1);
            }
            // if(debug_params_.show_img)
            // {
            //     namedWindow("dst",0);
            //     imshow("dst",src.img);
            //     waitKey(1);
            // }

            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0.0;
            return false;
        }

        time_infer = steady_clock_.now();
        
        //将对象排序，保留面积较大的对象
        sort(objects.begin(),objects.end(),[](ArmorObject& prev, ArmorObject& next)
        {
            return prev.area > next.area;
        });

        //若对象较多保留前按面积排序后的前max_armors个
        if (objects.size() > this->detector_params_.max_armors_cnt)
            objects.resize(this->detector_params_.max_armors_cnt);
        
        //生成装甲板对象
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
                // std::cout << "x:" << armor.apex2d[i].x << " y:" << armor.apex2d[i].y << std::endl;
            }
            Point2f apex_sum;
            for(auto apex : armor.apex2d)
                apex_sum +=apex;
            armor.center2d = apex_sum / 4.f;

            // auto pnp_result = coordsolver.pnp(armor.apex2d, rmat_imu, SOLVEPNP_ITERATIVE);

            // std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
            
            // std::vector<Point2f> tmp;
            // for (auto rrect.)
            // TargetType target_type = SMALL;

            // //计算长宽比,确定装甲板类型
            // RotatedRect points_pic_rrect = minAreaRect(points_pic);
            // auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
            //                         min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            
            // //若大于长宽阈值或为哨兵、英雄装甲板
            // if (apex_wh_ratio > this->armor_detector_params_.armor_type_wh_thres || object.cls == 1 || object.cls == 0)
            //     target_type = BIG;
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
                        cout << "IGN" << endl;
                    }
                }
            }
            //进行PnP，目标较少时采取迭代法，较多时采用IPPE
            int pnp_method;
            if (objects.size() <= 2)
                pnp_method = SOLVEPNP_ITERATIVE;
            else
                pnp_method = SOLVEPNP_IPPE;
            TargetType target_type = SMALL;

            //计算长宽比,确定装甲板类型
            auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
                                    min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            //若大于长宽阈值或为哨兵、英雄装甲板
            if (object.cls == 1 || object.cls == 0)
                target_type = BIG;
            //FIXME：若存在平衡步兵需要对此处步兵装甲板类型进行修改
            else if (object.cls == 2 || object.cls == 3 || object.cls == 4 || object.cls == 5 || object.cls == 6)
                target_type = SMALL;
            else if(apex_wh_ratio > detector_params_.armor_type_wh_thres)
                target_type = BIG;

            //单目PnP
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_ITERATIVE);
            // auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
            
            //防止装甲板类型出错导致解算问题，首先尝试切换装甲板类型，若仍无效则直接跳过该装甲板
            if (pnp_result.armor_cam.norm() > 10 ||
                isnan(pnp_result.armor_cam[0]) ||
                isnan(pnp_result.armor_cam[1]) ||
                isnan(pnp_result.armor_cam[2]))
            {
                if (target_type == SMALL)
                    target_type = BIG;
                else if (target_type == BIG)
                    target_type = SMALL;
                pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
                if (pnp_result.armor_cam.norm() > 10 ||
                    isnan(pnp_result.armor_cam[0]) ||
                    isnan(pnp_result.armor_cam[1]) ||
                    isnan(pnp_result.armor_cam[2]))
                    {
                        continue;
                    }
            }

            armor.armor3d_world = pnp_result.armor_world;
            armor.armor3d_cam = pnp_result.armor_cam;
            armor.euler = pnp_result.euler;
            armor.area = object.area;
            armors.push_back(armor);

            // std::cout << std::endl;
            // std::cout << "armor3d_world: x" << armor.armor3d_world[0] << " y:" << armor.armor3d_world[1] << " z:" << armor.armor3d_world[2] << std::endl;
            // std::cout << "armor3d_cam: x" << armor.armor3d_cam[0] << " y:" << armor.armor3d_cam[1] << " z:" << armor.armor3d_world[2] << std::endl;
            // std::cout << std::endl;
        }
        
        //若无合适装甲板
        if (armors.empty())
        {
            RCLCPP_WARN(logger_, "No suitable targets...");
            if(debug_params_.show_aim_cross)
            {
                line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
                line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
            }
            // if(debug_params_.show_img)
            // {
            //     namedWindow("dst",0);
            //     imshow("dst",src.img);
            //     waitKey(1);
            // }

            if(debug_params_.show_all_armors)
            {
                showArmors(src);
            }

            //更新陀螺分数
            spinning_detector_.updateSpinScore();

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

    bool Detector::gyro_detector(TaskData &src, global_interface::msg::Target& target_info)
    {
        /**
         * @brief 车辆小陀螺状态检测
        */

        //Create ArmorTracker for new armors 
        spinning_detector_.createArmorTracker(trackers_map, armors, new_armors_cnt_map, timestamp, dead_buffer_cnt);

        //Detect armors status
        spinning_detector_.isSpinning(trackers_map, new_armors_cnt_map);

        //Update spinning score
        spinning_detector_.updateSpinScore();

        //Choose target vehicle
        auto target_id = chooseTargetID(armors, timestamp);

        string target_key;
        if (detector_params_.color == BLUE)
            target_key = "B" + to_string(target_id);
        else if (detector_params_.color == RED)
            target_key = "R" + to_string(target_id);

        RCLCPP_INFO(logger_, "Target key: %s", target_key.c_str());

        ///-----------------------------detect whether exists matched tracker------------------------------------------
        if (trackers_map.count(target_key) == 0)
        {
            if(debug_params_.show_aim_cross)
            {
                line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), Scalar(0,255,0), 1);
                line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), Scalar(0,255,0), 1);
            }
            // if(debug_params_.show_img)
            // {
            //     namedWindow("dst",0);
            //     imshow("dst",src.img);
            //     waitKey(1);
            // }

            if(debug_params_.show_all_armors)
            {
                showArmors(src);
            }

            lost_cnt++;
            is_last_target_exists = false;
            RCLCPP_WARN(logger_, "No available tracker exists!");
            return false;
        }

        auto ID_candiadates = trackers_map.equal_range(target_key);

        ///---------------------------acqusition final armor's sequences---------------------------------------
        bool is_target_spinning;
        Armor target;
        std::vector<ArmorTracker*> final_trackers;
        std::vector<Armor> final_armors;
        //TODO:反陀螺防抖(增加陀螺模式与常规模式)
        //若目标处于陀螺状态，预先瞄准目标中心，待预测值与该点距离较近时开始击打
        SpinHeading spin_status;
        if (spinning_detector_.spin_status_map.count(target_key) == 0)
        {   //若未确定打击车辆的陀螺状态
            spin_status = UNKNOWN;
            is_target_spinning = false;
            target_info.is_spinning = false;
        }
        else
        {   //若确定打击车辆的陀螺状态
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
        if (spin_status != UNKNOWN)
        {
            //------------------------------尝试确定旋转中心-----------------------------------
            auto available_candidates_cnt = 0;
            for (auto iter = ID_candiadates.first; iter != ID_candiadates.second; ++iter)
            {
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
            
            auto cnt = spinning_detector_.spinning_x_map.count(target_key);
            if(cnt == 1)
            {
                auto candidate = spinning_detector_.spinning_x_map.find(target_key);

                auto t = ((*candidate).second.new_timestamp - (*candidate).second.last_timestamp) / 1e9;
                auto w = (2 * M_PI) / (4 * t);
                target_info.w = w;
                RCLCPP_INFO(logger_, "Target spinning period: %lf", w);
                
                if(((*candidate).second.new_x_back - (*candidate).second.last_x_back) > 0.15 && ((*candidate).second.new_x_font - (*candidate).second.last_x_font) > 0.15)
                {
                    target_info.is_spinning = true;
                    target_info.is_still_spinning = false;
                }
                else if((*candidate).second.new_x_back - (*candidate).second.last_x_back < 0.09 && ((*candidate).second.new_x_font - (*candidate).second.last_x_font) < 0.09)
                {
                    target_info.is_still_spinning = true;
                    target_info.is_spinning = false;
                }
            }
            
            //若存在一块装甲板
            if (final_armors.size() == 1)
            {
                target = final_armors.at(0);
            }
            //若存在两块装甲板
            else if (final_armors.size() == 2)
            {   // 选择旋转方向上落后的装甲板进行击打
                // 对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                sort(final_armors.begin(),final_armors.end(),[](Armor& prev, Armor& next)
                                    {return prev.armor3d_cam[0] < next.armor3d_cam[0];});
                // 若顺时针旋转选取右侧装甲板更新
                if (spin_status == CLOCKWISE)
                    target = final_armors.at(1);
                // 若逆时针旋转选取左侧装甲板更新
                else if (spin_status == COUNTER_CLOCKWISE)
                    target = final_armors.at(0);
            }

            //判断装甲板是否切换，若切换将变量置1
            auto delta_t = src.timestamp - prev_timestamp;
            auto delta_dist = (target.armor3d_world - last_armor.armor3d_world).norm();
            auto velocity = (delta_dist / delta_t) * 1e9;
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

            // target_info.point2d[0].x = target.apex2d[0].x;
            // target_info.point2d[0].y = target.apex2d[0].y;
            // target_info.point2d[1].x = target.apex2d[1].x;
            // target_info.point2d[1].y = target.apex2d[1].y;
            // target_info.point2d[2].x = target.apex2d[2].x;
            // target_info.point2d[2].y = target.apex2d[2].y;
            // target_info.point2d[3].x = target.apex2d[3].x;
            // target_info.point2d[3].y = target.apex2d[3].y;
            // target_info.aiming_point.x = target.armor3d_cam[0];
            // target_info.aiming_point.y = target.armor3d_cam[1];
            // target_info.aiming_point.z = target.armor3d_cam[2];
        }
        else
        {
            target_info.is_spinning = false;
            target_info.is_still_spinning = false;

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
            auto delta_dist = (target.armor3d_world - last_armor.armor3d_world).norm();
            auto velocity = (delta_dist / delta_t) * 1e9;
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
        }
        target_info.point2d[0].x = target.apex2d[0].x;
        target_info.point2d[0].y = target.apex2d[0].y;
        target_info.point2d[1].x = target.apex2d[1].x;
        target_info.point2d[1].y = target.apex2d[1].y;
        target_info.point2d[2].x = target.apex2d[2].x;
        target_info.point2d[2].y = target.apex2d[2].y;
        target_info.point2d[3].x = target.apex2d[3].x;
        target_info.point2d[3].y = target.apex2d[3].y;
        target_info.aiming_point_world.x = target.armor3d_world[0];
        target_info.aiming_point_world.y = target.armor3d_world[1];
        target_info.aiming_point_world.z = target.armor3d_world[2];
        target_info.aiming_point_cam.x = target.armor3d_cam[0];
        target_info.aiming_point_cam.y = target.armor3d_cam[1];
        target_info.aiming_point_cam.z = target.armor3d_cam[2];

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
        last_aiming_point = target.armor3d_cam;
        is_last_target_exists = true;
        last_armors.clear();
        last_armors = armors;

        if(debug_params_.show_aim_cross)
        {
            line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0,255,0}, 1);
            line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255,0}, 1);
        }

        if(debug_params_.show_all_armors)
        {
            showArmors(src);
        }
        
        auto angle = coordsolver_.getAngle(target.armor3d_cam, rmat_imu);
        // 若预测出错则直接世界坐标系下坐标作为击打点
        if (isnan(angle[0]) || isnan(angle[1]))
            angle = coordsolver_.getAngle(target.armor3d_world, rmat_imu);
        
        auto time_predict = steady_clock_.now();
        double dr_crop_ns = (time_crop - time_start).nanoseconds();
        double dr_infer_ns = (time_infer - time_crop).nanoseconds();
        double dr_full_ns = (time_predict - time_start).nanoseconds();

        if(debug_params_.show_fps)
        {
            char ch[10];
            sprintf(ch, "%.2f", (1e9 / dr_full_ns));
            std::string fps_str = ch;
            putText(src.img, fps_str, {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0,255,0});
        }

        if(debug_params_.print_letency)
        {
            //降低输出频率，避免影响帧率
            if (count % 5 == 0)
            {
                RCLCPP_INFO(logger_, "-----------TIME------------");
                RCLCPP_INFO(logger_, "Crop:  %lfms\n", (dr_crop_ns / 1e6));
                RCLCPP_INFO(logger_, "Infer: %lfms\n", (dr_infer_ns / 1e6));
                RCLCPP_INFO(logger_, "Total: %lfms\n", (dr_full_ns / 1e6));
            }
        }
        // cout<<target.armor3d_world<<endl;
        // cout<<endl;
    
        if(debug_params_.print_target_info)
        {
            if (count % 5 == 0)
            {
                RCLCPP_INFO(logger_, "-----------INFO------------");
                RCLCPP_INFO(logger_, "Yaw: %lf", angle[0]);
                RCLCPP_INFO(logger_, "Pitch: %lf", angle[1]);
                RCLCPP_INFO(logger_, "Dist: %fm", (float)target.armor3d_cam.norm());
                RCLCPP_INFO(logger_, "Target: %s", target.key.c_str());
                RCLCPP_INFO(logger_, "Target Type: %s", (char *)(target.type == SMALL ? "SMALL" : "BIG"));
                RCLCPP_INFO(logger_, "Is Spinning: %d", (int)(is_target_spinning));
                RCLCPP_INFO(logger_, "Is Switched: %d", (int)(is_target_switched));

                if(is_save_data)
                {
                    data_save << setprecision(3) << (float)target.armor3d_cam.norm() << endl;
                }
                count = 0;
            }
            else
            {
                count++;
            }
        }

        //若预测出错取消本次数据发送
        // if (isnan(angle[0]) || isnan(angle[1]))
        //     return false;

        // if(debug_params_.show_img)
        // {
        //     namedWindow("dst",0);
        //     imshow("dst",src.img);
        //     waitKey(1);
        // }

        return true;
    }

    void Detector::showArmors(TaskData& src)
    {
        for (auto armor : armors)
        {
            char ch[10];
            sprintf(ch, "%.3f", armor.conf);
            std::string conf_str = ch;
            putText(src.img, conf_str, armor.apex2d[3], FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0}, 2);

            char ch1[10];
            std::string id_str = "";
            if (armor.color == 0)
            {
                sprintf(ch1, "B%d", armor.id);
                id_str = ch1;
                putText(src.img, id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
            }
            if (armor.color == 1)
            {
                sprintf(ch1, "R%d", armor.id);
                id_str = ch1;
                putText(src.img, id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);
            }
            if (armor.color == 2)
            {
                sprintf(ch1, "N%d", armor.id);
                id_str = ch1;
                putText(src.img, id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 255, 255}, 2);
            }
            if (armor.color == 3)
            {
                sprintf(ch1, "P%d", armor.id);
                id_str = ch1;
                putText(src.img, id_str, armor.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 255}, 2);
            }
            for(int i = 0; i < 4; i++)
                line(src.img, armor.apex2d[i % 4], armor.apex2d[(i + 1) % 4], {0,255,0}, 1);
            rectangle(src.img, armor.roi, {255, 0, 255}, 1);
            auto armor_center = coordsolver_.reproject(armor.armor3d_cam);
            circle(src.img, armor_center, 4, {0, 0, 255}, 2);
        }
    }

    Point2i Detector::cropImageByROI(Mat &img)
    {
        // if (!is_last_target_exists)
        // {
        //     //当丢失目标帧数过多或lost_cnt为初值
        //     if (lost_cnt > this->armor_detector_params_.max_lost_cnt || lost_cnt == 0)
        //     {
        //         return Point2i(0,0);
        //     }
        // }

        // //若目标大小大于阈值
        // if ((last_target_area / img.size().area()) > this->armor_detector_params_.no_crop_thres)
        // {
        //     return Point2i(0,0);
        // }
        // //处理X越界
        
        // // 计算上一帧roi中心在原图像中的坐标
        // Point2i last_armor_center = Point2i(last_roi_center.x - this->armor_detector_params_.dw, last_roi_center.y - this->armor_detector_params_.dh) * (1 / this->armor_detector_params_.rescale_ratio);

        // float armor_h = calcDistance(last_armor.apex2d[0], last_armor.apex2d[1]);
        // float armor_w = calcDistance(last_armor.apex2d[1], last_armor.apex2d[2]);
        // int roi_width = MAX(armor_h, armor_w) * (1 / this->armor_detector_params_.rescale_ratio);
        // int roi_height = MIN(armor_h, armor_w) * (1 / this->armor_detector_params_.rescale_ratio);

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
    
    ArmorTracker* Detector::chooseTargetTracker(vector<ArmorTracker*> trackers, double timestamp)
    {
        //TODO:优化打击逻辑
        //TODO:本逻辑为哨兵逻辑
        float max_area = 0;
        float min_horizonal_dist = 0;
        int target_idx = 0;

        //若存在上次tracker则直接返回,若不存在则装甲板面积最大的Tracker
        for (int i = 0; i < (int)(trackers.size()); i++)
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

    int Detector::chooseTargetID(vector<Armor> &armors, double timestamp)
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
            if (armor.id == 1 && armor.armor3d_world.norm() <= detector_params_.hero_danger_zone)
            {
                return armor.id;
            }
            //若存在上次击打目标,时间较短,且该目标运动较小则将其选为候选目标,若遍历结束未发现危险距离内的英雄则将其ID选为目标ID.
            else if (armor.id == last_armor.id && abs(armor.area - last_armor.area) / (float)armor.area < 0.3 && abs(timestamp - prev_timestamp) / 1e6 < 30)
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

    void Detector::setDetectorParam(const double& param, int idx)
    {
        switch (idx)
        {
        case 1:
            detector_params_.armor_conf_high_thres = param;
            break;
        case 2:
            detector_params_.armor_roi_expand_ratio_height = param;
            break;
        case 3:
            detector_params_.armor_roi_expand_ratio_width = param;
            break;
        case 4:
            detector_params_.armor_type_wh_thres = param;
            break;
        case 5:
            detector_params_.color = (Color)(param);
            break;
        case 6:
            detector_params_.dh = param;
            break;
        case 7:
            detector_params_.dw = param;
            break;
        case 8:
            detector_params_.full_crop_ratio = param;
            break;
        case 9:
            detector_params_.hero_danger_zone = param;
            break;
        case 10:
            detector_params_.max_armors_cnt = param;
            break;
        case 11:
            detector_params_.max_delta_dist = param;
            break;
        case 12:
            detector_params_.max_delta_t = param;
            break;
        default:
            break;
        }
    }

    void Detector::setDebugParam(const bool& param, int idx)
    {
        switch (idx)
        {
        case 1:
            debug_params_.debug_without_com = param;
            break;
        case 2:
            debug_params_.detect_red = param;
            break;
        case 3:
            debug_params_.print_letency = param;
            break;
        case 4:
            debug_params_.print_target_info = param;
            break;
        case 5:
            debug_params_.show_aim_cross = param;
            break;
        case 6:
            debug_params_.show_fps = param;
            break;
        case 7:
            debug_params_.show_img = param;
            break;
        case 8:
            debug_params_.using_imu = param;
            break;
        case 9:
            debug_params_.using_roi = param;
            break;
        default:
            break;
        }
    }

    bool Detector::getDebugParam(int idx)
    {
        return debug_params_.using_imu;
    }
} //namespace Detector