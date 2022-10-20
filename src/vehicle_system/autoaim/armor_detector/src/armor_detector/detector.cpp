/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-13 23:26:16
 * @LastEditTime: 2022-10-17 14:01:51
 * @FilePath: /tup_2023-10-16/src/vehicle_system/autoaim/armor_detector/src/detector.cpp
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

        //debug
        // this->debug_params_.debug_without_com = _debug_params_.debug_without_com;
        // this->debug_params_.using_imu =  _debug_params_.using_imu;
        // this->debug_params_.using_roi =  _debug_params_.using_roi;
        // this->debug_params_.show_aim_cross = _debug_params_.show_aim_cross;
        // this->debug_params_.show_img = _debug_params_.show_img;
        // this->debug_params_.detect_red = _debug_params_.detect_red;
       
        //gyro_detect_params
        // spinning_detector_ = spinning_detector(_detector_params_.color, _gyro_params_);
    }

    detector::~detector()
    {
        
    }
    
    bool detector::armor_detect(global_user::TaskData &src)
    {
        if(!is_init)
        {
            // printf("1");
            // RCLCPP(this->get_logger(), "1...");
            cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
            cv::imshow("image", src.img);
            cv::waitKey(0);

            detector_.initModel(network_path);

            cv::namedWindow("image_1", cv::WINDOW_AUTOSIZE);
            cv::imshow("image_1", src.img);
            cv::waitKey(0);

            // printf("2");
            coordsolver_.loadParam(camera_param_path, camera_name);

            is_init = true;
        }

        auto time_start = std::chrono::steady_clock::now();
        auto input = src.img;
        timestamp = src.timestamp;
        
        if(!debug_params_.debug_without_com)
        {   //有串口
            //设置弹速,若弹速大于10m/s值,且弹速变化大于0.5m/s则更新
            if (src.bullet_speed > 10 && abs(src.bullet_speed - last_bullet_speed) > 0.5)
            {
                last_bullet_speed = src.bullet_speed;
            }
        }

        Eigen::Matrix3d rmat_imu;
        if(!debug_params_.using_imu)
        {   //使用陀螺仪数据
            rmat_imu = src.quat.toRotationMatrix();
        }
        else
        {
            rmat_imu = Eigen::Matrix3d::Identity();
        }

        if(!debug_params_.using_roi)
        {   //启用roi
            roi_offset = cropImageByROI(input);
        }

        auto time_crop = std::chrono::steady_clock::now();

        objects.clear();
        armors.clear();
        if(!detector_.detect(input, objects, this->detector_params_.dw, this->detector_params_.dh, this->detector_params_.rescale_ratio))
        {   //若未检测到目标
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

        auto inference_time = std::chrono::steady_clock::now();
        
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
            }
            Point2f apex_sum;
            for(auto apex : armor.apex2d)
                apex_sum +=apex;
            armor.center2d = apex_sum / 4.f;

            // auto pnp_result = coordsolver.pnp(armor.apex2d, rmat_imu, SOLVEPNP_ITERATIVE);

            std::vector<Point2f> points_pic(armor.apex2d, armor.apex2d + 4);
            // std::vector<Point2f> tmp;
            // for (auto rrect.)
            global_user::TargetType target_type = global_user::SMALL;

            //计算长宽比,确定装甲板类型
            RotatedRect points_pic_rrect = minAreaRect(points_pic);
            auto apex_wh_ratio = max(points_pic_rrect.size.height, points_pic_rrect.size.width) /
                                    min(points_pic_rrect.size.height, points_pic_rrect.size.width);
            
            //若大于长宽阈值或为哨兵、英雄装甲板
            if (apex_wh_ratio > this->detector_params_.armor_type_wh_thres || object.cls == 1 || object.cls == 0)
                target_type = global_user::BIG;
            // for (auto pic : points_pic)
            //     cout<<pic<<endl;
            // cout<<target_type<<endl;

            //单目PnP
            auto pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
            
            //防止装甲板类型出错导致解算问题，首先尝试切换装甲板类型，若仍无效则直接跳过该装甲板
            if (pnp_result.armor_cam.norm() > 10)
            {
                if (target_type == global_user::SMALL)
                    target_type = global_user::BIG;
                else if (target_type == global_user::BIG)
                    target_type = global_user::SMALL;
                pnp_result = coordsolver_.pnp(points_pic, rmat_imu, target_type, SOLVEPNP_IPPE);
                if (pnp_result.armor_cam.norm() > 10)
                    continue;
            }

            armor.center3d_world = pnp_result.armor_world;
            armor.center3d_cam = pnp_result.armor_cam;
            armor.euler = pnp_result.euler;
            armor.area = object.area;
            armors.push_back(armor);
        }

        //若无合适装甲板
        if (armors.empty())
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

            //更新陀螺分数
            spinning_detector_.update_spin_score();

            lost_cnt++;
            is_last_target_exists = false;
            last_target_area = 0;
            return false;
        }
    }

    bool detector::gyro_detector(global_user::TaskData &src, Eigen::Vector3d& aiming_point)
    {
        /**
         * @brief 车辆小陀螺状态检测
        */
        spinning_detector_.create_armor_tracker(timestamp, dead_buffer_cnt);

        spinning_detector_.is_spinning();

        spinning_detector_.update_spin_score();

        //判断击打车辆
        auto target_id = chooseTargetID(armors, timestamp);
        string target_key;
        if (detector_params_.color == BLUE)
            target_key = "B" + to_string(target_id);
        else if (detector_params_.color == RED)
            target_key = "R" + to_string(target_id);

        ///-----------------------------判断该装甲板是否有可用Tracker------------------------------------------
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

        ///---------------------------获取最终装甲板序列---------------------------------------
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
        }
        else
        {
            spin_status = spinning_detector_.spin_status_map[target_key];
            if (spin_status != UNKNOWN)
                is_target_spinning = true;
            else
                is_target_spinning = false;
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
            //若存在一块装甲板
            if (final_armors.size() == 1)
            {
                target = final_armors.at(0);
            }
            //若存在两块装甲板
            else if (final_armors.size() == 2)
            {
                //对最终装甲板进行排序，选取与旋转方向相同的装甲板进行更新
                sort(final_armors.begin(),final_armors.end(),[](Armor& prev, Armor& next)
                                    {return prev.center3d_cam[0] < next.center3d_cam[0];});
                //若顺时针旋转选取右侧装甲板更新
                if (spin_status == CLOCKWISE)
                    target = final_armors.at(1);
                //若逆时针旋转选取左侧装甲板更新
                else if (spin_status == COUNTER_CLOCKWISE)
                    target = final_armors.at(0);
            }

            //判断装甲板是否切换，若切换将变量置1
            auto delta_t = src.timestamp - prev_timestamp;
            auto delta_dist = (target.center3d_world - last_armor.center3d_world).norm();
            auto velocity = (delta_dist / delta_t) * 1e3;
            if ((target.id != last_armor.id || !last_armor.roi.contains((target.center2d))) &&
                is_last_target_exists)
                is_target_switched = true;
            else
                is_target_switched = false;
            
            aiming_point = target.center3d_cam;
        }
        else
        {
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
            if ((target.id != last_armor.id || !last_armor.roi.contains((target.center2d))) && is_last_target_exists)
                is_target_switched = true;
            else
                is_target_switched = false;

            aiming_point = target.center3d_cam;
        }

        if (target.color == 2)
            dead_buffer_cnt++;
        else
            dead_buffer_cnt = 0;

        //获取装甲板中心与装甲板面积以下一次ROI截取使用
        last_roi_center = target.center2d;
        // last_roi_center = Point2i(512,640);
        last_armor = target;
        lost_cnt = 0;
        prev_timestamp = src.timestamp;
        last_target_area = target.area;
        last_aiming_point = aiming_point;
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
            for (auto armor :armors)
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

        if(debug_params_.show_img)
        {
            namedWindow("dst",0);
            imshow("dst",src.img);
            waitKey(1);
        }

        return true;
    }

    Point2i detector::cropImageByROI(Mat &img)
    {
        if (!is_last_target_exists)
        {
            //当丢失目标帧数过多或lost_cnt为初值
            if (lost_cnt > this->detector_params_.max_lost_cnt || lost_cnt == 0)
            {
                return Point2i(0,0);
            }
        }

        //若目标大小大于阈值
        if ((last_target_area / img.size().area()) > this->detector_params_.no_crop_thres)
        {
            return Point2i(0,0);
        }
        //处理X越界
        
        // 计算上一帧roi中心在原图像中的坐标
        Point2i last_armor_center = Point2i(last_roi_center.x - this->detector_params_.dw, last_roi_center.y - this->detector_params_.dh) * (1 / this->detector_params_.rescale_ratio);

        float armor_h = global_user::calcDistance(last_armor.apex2d[0], last_armor.apex2d[1]);
        float armor_w = global_user::calcDistance(last_armor.apex2d[1], last_armor.apex2d[2]);
        int roi_width = MAX(armor_h, armor_w) * (1 / this->detector_params_.rescale_ratio);
        int roi_height = MIN(armor_h, armor_w) * (1 / this->detector_params_.rescale_ratio);

        //根据丢失帧数逐渐扩大ROI大小
        if(lost_cnt == 2)
        {   //丢失2帧ROI扩大3.2倍
            roi_width *= 1.2;
            roi_height *= 1.2;
        }
        else if(lost_cnt == 3)
        {   //丢失3帧ROI扩大4.5倍
            roi_width *= 1.8;
            roi_height *= 1.8;
        }
        else if(lost_cnt == 4)
        {   //丢失4帧ROI扩大6倍
            roi_width *= 2.2;
            roi_height *= 2.2;
        }
        else if(lost_cnt == 5)
        {   //返回原图像
            return Point2i(0,0);
        }

        //防止roi越界
        if(last_armor_center.x > img.size().width || last_armor_center.y > img.size().height)
        {
            return Point2f(0, 0);
        }
        if(last_armor_center.x < 0 || last_armor_center.y < 0)
        {
            return Point2f(0, 0);
        }

        if((last_armor_center.x + roi_width / 2)  > img.size().width)
            roi_width = (img.size().width - last_armor_center.x) * 2;
        
        if(last_armor_center.y + roi_height / 2 > img.size().height)
            roi_height = (img.size().height - last_armor_center.y) * 2;

        Point2f roi_left_top = Point2i(last_armor_center.x - roi_width / 2, last_armor_center.y - roi_height / 2);
        Rect roi_rect = Rect(roi_left_top.x, roi_left_top.y, roi_width, roi_height);
        img(roi_rect).copyTo(img);

        return roi_left_top;

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