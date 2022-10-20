// Copyright 2020 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMOSS_AUTO_AIM__ARMOR_DETECTOR_HPP_
#define RMOSS_AUTO_AIM__ARMOR_DETECTOR_HPP_

#include <vector>
#include <opencv2/opencv.hpp>
#include "inference.h"

const string network_path = "/home/dishierweidu/Desktop/TUP-InfantryVision-2022-main/model/nano_0411.xml";

// 装甲板识别检测算法部分，提取相关特征点。

/* 本代码参考了其他robomaster2018各大学开源代码，主要有如下：
 *   大连交大robomaster
 *    https://github.com/Ponkux/RM2018-DJTU-VisionOpenSource
 *   东南大学robomaster :
 *    https://github.com/SEU-SuperNova-CVRA/Robomaster2018-SEU-OpenSource
 *   东林robomaster :
 *    https://github.com/moxiaochong/NEFU-ARES-Vison-Robomaster2018
*/

namespace rmoss_auto_aim
{
typedef struct _ArmorDescriptor
{
    int id;
    int color;
    int area;
    string key;
    cv::Point2f points[4];
    cv::Point2f centerPoint;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    Eigen::Vector3d predict;
} ArmorDescriptor;

class ArmorDetector
{
public:
  ArmorDetector();
  ~ArmorDetector();

public:
  void set_target_color(bool is_red);
  int process(cv::Mat img);
  std::vector<ArmorDescriptor> getArmorVector();
  // 调试
  void printArmorDescriptor(ArmorDescriptor armor);

private:
  bool target_is_red_;

  std::vector<ArmorDescriptor> mArmors;
  
  // inference
  bool is_last_target_exists;
  int lost_cnt;
  int last_timestamp;
  double last_target_area;
  Point2i last_roi_center;
  Eigen::Vector3d last_aiming_point;
  Point2i roi_offset;
  Size2d input_size;


  std::map<string,int> new_armors_cnt_map;    //装甲板计数map，记录新增装甲板数

  std::map<string,double> spin_score_map;     //反小陀螺，记录各装甲板小陀螺可能性分数，大于0为逆时针旋转，小于0为顺时针旋转

  const int max_lost_cnt = 5;                 //最大丢失目标帧数
  const int max_armors = 8;                   //视野中最多装甲板数
  const int max_v = 8;                        //两次预测间最大速度(m/s)
  const int max_delta_t = 100;                //使用同一预测器的最大时间间隔(ms)

  int anti_spin_judge_high_thres = 2e4;//大于该阈值认为该车已开启陀螺
  int anti_spin_judge_low_thres = 1e4;//小于该阈值认为该车已关闭陀螺
  int anti_spin_max_r_multiple = 2;

  const double no_crop_thres = 2e-3;      //禁用ROI裁剪的装甲板占图像面积最大面积比值

  const int hero_danger_zone = 400;       //英雄危险距离阈值，检测到有小于该距离的英雄直接开始攻击

  // Armor last_armor;
  Detector detector;
  // Predictor predictor_param_loader;
  // Predictor predictor;
  // CoordSolver coordsolver;
  // string chooseTargetID(vector<ArmorDescriptor> &armors);
  // ArmorDescriptor chooseTargetArmor(vector<ArmorDescriptor> armors);
  Point2i cropImageByROI(Mat &img);
};

}  // namespace rmoss_auto_aim

#endif  // RMOSS_AUTO_AIM__ARMOR_DETECTOR_HPP_
