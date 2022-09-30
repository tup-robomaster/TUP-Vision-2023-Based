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

#include "rmoss_power_rune2019/power_rune_node.hpp"
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"
#include "rmoss_util/debug.hpp"

using namespace cv;
using namespace std;
using namespace rmoss_power_rune2019;


PowerRuneNode::PowerRuneNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("simple_auto_aim", options);
  std::string camera_name = "camera";
  std::string robot_color = "red";
  bool auto_start = false;
  bool debug = false;
  // parameters
  node_->declare_parameter("robot_color", robot_color);
  node_->declare_parameter("debug", debug);
  node_->declare_parameter("camera_name", camera_name);
  node_->declare_parameter("auto_start", auto_start);
  node_->get_parameter("robot_color", robot_color);
  node_->get_parameter("debug", debug);
  node_->get_parameter("camera_name", camera_name);
  node_->get_parameter("auto_start", auto_start);
  bool is_red = (robot_color == "red");
  rmoss_util::set_debug(debug);
  // create pub,sub,srv
  using namespace std::placeholders;
  gimbal_ctrl_pub_ = node_->create_publisher<rmoss_interfaces::msg::GimbalCmd>(
    "robot_base/gimbal_cmd", 10);
  shoot_pub_ = node_->create_publisher<rmoss_interfaces::msg::ShootCmd>("robot_base/shoot_cmd", 10);
  gimbal_state_sub_ = node_->create_subscription<rmoss_interfaces::msg::Gimbal>(
    "robot_base/gimbal_state", 10,
    std::bind(&PowerRuneNode::gimbalStateCallback, this, _1));
  set_mode_srv_ = node_->create_service<rmoss_interfaces::srv::SetMode>(
    "task_power_rune/set_mode",
    std::bind(&PowerRuneNode::setModeCallBack, this, _1, _2));
  // create camera client
  cam_client_ = std::make_shared<rmoss_cam::CamClient>(
    node_);
  // wait camera parameters camera_k,camera_d,camera_p
  sensor_msgs::msg::CameraInfo info;
  if (!cam_client_->get_camera_info(info)) {
    RCLCPP_ERROR(node_->get_logger(), "get camera info failed!");
    return;
  }
  std::ostringstream oss;
  oss << "k:";
  for (auto & x : info.k) {
    oss << x <<" ";
  }
  oss << ",d:";
  for (auto & x : info.d) {
    oss << x <<" ";
  }
  RCLCPP_INFO(node_->get_logger(), "get camera info: %s", oss.str().c_str());
  std::vector<double> camera_k(9, 0);
  std::copy_n(info.k.begin(), 9, camera_k.begin());
  // init algo tool
  auto mono_location_tool = std::make_shared<rmoss_util::MonoMeasureTool>();
  mono_location_tool->set_camera_info(camera_k, info.d);
  power_rune_algo_ = std::make_shared<SimplePowerRuneAlgo>(mono_location_tool);
  if (is_red) {
    power_rune_algo_->setTargetColor(ArmorColor::red);
  }else{
    power_rune_algo_->setTargetColor(ArmorColor::blue);
  }
  // init projectile tool
  auto projectile_model = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(25);
  gimbal_tansformoss_tool_ = std::make_shared<rmoss_projectile_motion::GimbalTransformTool>();
  gimbal_tansformoss_tool_->set_projectile_solver(projectile_model);
  RCLCPP_INFO(node_->get_logger(), "init successfully!");
  if (auto_start) {
    current_mode_ = TaskMode::large;
    run_flag_ = true;
    RCLCPP_INFO(node_->get_logger(), "auto start!");
  }
}

void PowerRuneNode::process_image(const cv::Mat & img, const rclcpp::Time & /*stamp*/)
{
  if (!run_flag_) {
    return;
  }
  // RCLCPP_INFO(node_->get_logger(), "stamp:%f",img_stamp);
  if (is_need_reshoot_) {
    power_rune_algo_->setReShoot();
    is_need_reshoot_ = false;
  }
  if (current_mode_ == TaskMode::small) {
    if (is_need_clear_) {
      power_rune_algo_->clear4small();
      is_need_clear_ = false;
    }
    int ret = power_rune_algo_->process4small(img);
    if (ret == 0) {
      //射击
      Point3f position = power_rune_algo_->getShootTarget();
      std::cout << "small shoot:" << position << std::endl;
    }
  } else if (current_mode_ == TaskMode::large) {
    if (is_need_clear_) {
      power_rune_algo_->clear4large();
      is_need_clear_ = false;
    }
    int ret = power_rune_algo_->process4large(img);
    if (ret == 0) {
      //射击
      Point3f position = power_rune_algo_->getShootTarget();
      std::cout << "large shoot:" << position << std::endl;
    }
  }
}

bool PowerRuneNode::setModeCallBack(
  const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response)
{
  // 0x00 停止，0x01,运行小能量机关，0x02运行大能量机关,0x03重置发射
  response->success = true;
  if (request->mode == 0x00) {
    run_flag_ = false;
    current_mode_ = TaskMode::idle;
  } else if (request->mode == 0x01) {
    run_flag_ = true;
    is_need_clear_ = true;
    current_mode_ = TaskMode::small;
  } else if (request->mode == 0x02) {
    run_flag_ = true;
    is_need_clear_ = true;
    current_mode_ = TaskMode::large;
  } else if (request->mode == 0x03) {
    is_need_reshoot_ = true;
  } else if (request->mode == 0x10) {
    //red color config
    if (current_mode_ == TaskMode::idle) {
      power_rune_algo_->setTargetColor(ArmorColor::red);
    } else {
      response->success = false;
    }
  } else if (request->mode == 0x11) {
    //blue color config,
    if (current_mode_ == TaskMode::idle) {
      power_rune_algo_->setTargetColor(ArmorColor::blue);
    } else {
      response->success = false;
    }
  } else {
    response->success = false;
  }
  return true;
}

void PowerRuneNode::gimbalStateCallback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg)
{
  current_pitch_ = msg->pitch;
}
