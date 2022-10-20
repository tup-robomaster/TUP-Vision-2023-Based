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
#include "rmoss_power_rune2019/rune_detector.hpp"

#include "rmoss_util/debug.hpp"
#include "rmoss_util/image_utils.hpp"

using namespace std;
using namespace cv;
using namespace rmoss_power_rune2019;

RuneDetector::RuneDetector() {target_is_red_ = true;}

RuneDetector::~RuneDetector() {}


void RuneDetector::setTargetColor(ArmorColor color)
{
  if (color != ArmorColor::red) {
    target_is_red_ = true;
  }
}

int RuneDetector::preImg(Mat & src, Mat & dst)
{
  Mat bgr[3];
  Mat img_b, img_g, img_r;
  split(src, bgr);    //将三个通道的像素值分离
  img_b = bgr[0];
  img_g = bgr[1];
  img_r = bgr[2];
  // RMOSS_DEBUG(imshow("b",img_b));
  // RMOSS_DEBUG(imshow("g",img_g));
  // RMOSS_DEBUG(imshow("r",img_r));
  //图像预处理
  Mat imgTargetGray, img_Blur, imgTargetThre;
  Mat img_color, img_bright;
  // taget is red
  if (target_is_red_) {
    imgTargetGray = img_r - 0.3 * img_g - 0.5 * img_b;
    img_bright = img_r.clone();
  } else {
    imgTargetGray = img_b - 0.5 * img_r - 0.3 * img_g;
    img_bright = img_b.clone();
  }
  //目标颜色区域
  Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
  dilate(imgTargetGray, img_color, element);
  threshold(img_color, img_color, 100, 255, THRESH_BINARY);
  RMOSS_DEBUG(imshow("color_dialte_thre", img_color));
  //亮度区域

  Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(7, 7));
  morphologyEx(
    img_bright, img_bright, MORPH_CLOSE,
    element);               //先膨胀，再腐蚀，连接，抗锯齿，光滑
  GaussianBlur(img_bright, img_bright, Size(5, 5), 1);
  threshold(img_bright, img_bright, 128, 255, THRESH_BINARY);
  RMOSS_DEBUG(imshow("bright_thre", img_bright));
  //逻辑与，求交集
  bitwise_and(img_color, img_bright, dst);
  RMOSS_DEBUG(imshow("dst", dst));
  RMOSS_DEBUG(waitKey(1));
  return 0;
}

int RuneDetector::process(cv::Mat & img)
{
  //////////////预处理
  Mat thresh_img;
  preImg(img, thresh_img);
  //////////////提取装甲板
  armors_.clear();
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  //检测所有轮廓，保存轮廓上的所有点
  findContours(thresh_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
  for (size_t i = 0; i < contours.size(); i++) {
    //轮廓层级判断
    if (hierarchy[i][2] != -1) {
      continue;        //如果有子轮廓，否定
    }
    if (hierarchy[i][3] == -1) {
      continue;        //如果没有父轮廓，否定
    } else {
      Vec4i parent_hierarchy = hierarchy[hierarchy[i][3]];
      if (parent_hierarchy[3] != -1) {
        continue;          //如果有父轮廓，但是父轮廓不是一级轮廓，否定
      }
    }
    if ((contours[i].size() > 50) && (contours[i].size() < 300)) {
      //满足轮廓点数
      RotatedRect armorRect = minAreaRect(Mat(contours[i]));        //椭圆拟合
      ArmorDescriptor armor;
      int ret = 0;
      ret = getArmorDescriptor(armorRect, armor);
      if (ret == 0) {
        armors_.push_back(armor);
        // printArmorDescriptor(armor);
      }
      // cout<<"arm ret:"<<ret<<endl;
    }
  }
  // cout<<"armor num:"<<armors_.size()<<endl;
  //////////////赋予id
  for (size_t i = 0; i < armors_.size(); i++) {
    armors_[i].id = i;
  }
  if (armors_.size() == 0) {
    return 1;      //识别失败
  }
  return 0;
}

int RuneDetector::getArmorDescriptor(RotatedRect r, ArmorDescriptor & armor)
{
  armor.r = r;
  armor.center = r.center;
  Point2f rect_points[4];
  r.points(rect_points);
  // line1:point0-point1
  float linelen1 = cv::norm(rect_points[0] - rect_points[1]);
  // line2:point1-point2
  float linelen2 = cv::norm(rect_points[1] - rect_points[2]);
  bool line1_max_flag = false;
  if (linelen1 > linelen2) {
    // line1为最长边;
    armor.armor_hight = linelen2;
    armor.armor_width = linelen1;
    line1_max_flag = true;
  } else {
    // line2为最长边，作为上边
    armor.armor_hight = linelen1;
    armor.armor_width = linelen2;
  }
  armor.armor_area = armor.armor_hight * armor.armor_width;
  armor.armor_ratio_wh = armor.armor_width / armor.armor_hight;
  //条件1 ：
  if (armor.armor_width < 45 || armor.armor_width > 65) {
    return 1;
  }
  //条件2
  if (armor.armor_hight < 20 || armor.armor_hight > 40) {
    return 2;      // heigt of armor is short,because it is too far
  }
  //条件3
  if (armor.armor_ratio_wh < 1.4 || armor.armor_ratio_wh > 2) {
    return 3;      //宽高比不符合，否定
  }
  //装甲板四个点
  Point2f vec_long_line2d, vec_short_line2d;
  if (line1_max_flag) {
    vec_long_line2d = rect_points[0] - rect_points[1];
    vec_short_line2d = rect_points[2] - rect_points[1];
  } else {
    vec_long_line2d = rect_points[2] - rect_points[1];
    vec_short_line2d = rect_points[0] - rect_points[1];
  }
  //向量积,1为顶点,long_vec*short_vec
  float z = vec_long_line2d.x * vec_short_line2d.y -
    vec_short_line2d.x * vec_long_line2d.y;
  if (z > 0) {
    // 1做起点
    if (line1_max_flag) {
      armor.points[0] = rect_points[1];
      armor.points[1] = rect_points[0];
      armor.points[2] = rect_points[3];
      armor.points[3] = rect_points[2];
    } else {
      armor.points[0] = rect_points[1];
      armor.points[1] = rect_points[2];
      armor.points[2] = rect_points[3];
      armor.points[3] = rect_points[0];
    }
  } else {
    // 1不能做起点
    if (line1_max_flag) {
      // 0做起点
      armor.points[0] = rect_points[0];
      armor.points[1] = rect_points[1];
      armor.points[2] = rect_points[2];
      armor.points[3] = rect_points[3];
    } else {
      // 2做起点
      armor.points[0] = rect_points[2];
      armor.points[1] = rect_points[1];
      armor.points[2] = rect_points[0];
      armor.points[3] = rect_points[3];
    }
  }
  return 0;
}

void RuneDetector::printArmorDescriptor(ArmorDescriptor & armor)
{
  cout << "-----------------------------------" << endl;
  cout << "armor_area    :" << armor.armor_area << endl;
  cout << "armor_hight   :" << armor.armor_hight << endl;
  cout << "armor_width   :" << armor.armor_width << endl;
  cout << "armor_ratio_wh:" << armor.armor_ratio_wh << endl;
  cout << "-----------------------------------" << endl;
}

std::vector<ArmorDescriptor> RuneDetector::getAllArmors() {return armors_;}
