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
#ifndef RMOSS_POWER_RUNE2019__POWER_RUNE_NODE_HPP_
#define RMOSS_POWER_RUNE2019__POWER_RUNE_NODE_HPP_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "rmoss_interfaces/srv/set_mode.hpp"

#include "rmoss_power_rune2019/simple_power_rune_algo.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"
#include "rmoss_cam/cam_client.hpp"


namespace rmoss_power_rune2019
{
enum class TaskMode { idle,
  small,
  large, };
class PowerRuneNode
{
public:
  explicit PowerRuneNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void process_image(const cv::Mat & img, const rclcpp::Time & stamp);
  bool setModeCallBack(
    const std::shared_ptr<rmoss_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rmoss_interfaces::srv::SetMode::Response> response);
  void gimbalStateCallback(const rmoss_interfaces::msg::Gimbal::SharedPtr msg);
  void taskImageProcess(cv::Mat & img, double img_stamp);

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rmoss_cam::CamClient> cam_client_;
  //
  rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_ctrl_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_pub_;
  rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
  rclcpp::Service<rmoss_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  // tool
  std::shared_ptr<SimplePowerRuneAlgo> power_rune_algo_;
  std::shared_ptr<rmoss_projectile_motion::GimbalTransformTool> gimbal_tansformoss_tool_;
  // data
  // offset of cam-gimbal
  cv::Point3f offset_trans_;    // 云台相对于相机的坐标（直接测量获得），单位为cm
  float offset_pitch_;          // pitch轴偏移量
  float offset_yaw_;            // yaw轴偏移量
  // tmp data
  float current_pitch_;
  bool is_need_clear_{false};
  bool is_need_reshoot_{false};
  TaskMode current_mode_{TaskMode::idle};
  bool run_flag_{false};
};
}  // namespace rmoss_power_rune2019

#endif  // RMOSS_POWER_RUNE2019__POWER_RUNE_NODE_HPP_
