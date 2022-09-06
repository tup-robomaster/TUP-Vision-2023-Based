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

#ifndef RMOSS_AUTO_AIM__SIMPLE_AUTO_AIM_NODE_HPP_
#define RMOSS_AUTO_AIM__SIMPLE_AUTO_AIM_NODE_HPP_

#include <memory>
#include <string>

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "rmoss_cam/cam_client.hpp"
#include "rmoss_auto_aim/simple_auto_aim_algo.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "rmoss_interfaces/msg/gimbal.hpp"
#include "rmoss_interfaces/msg/gimbal_cmd.hpp"
#include "rmoss_interfaces/msg/shoot_cmd.hpp"
#include "rmoss_interfaces/srv/set_color.hpp"
#include "rmoss_util/task_manager.hpp"

namespace rmoss_auto_aim
{
class SimpleAutoAimNode
{
public:
  explicit SimpleAutoAimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  void init();
  void process_image(const cv::Mat & img, const rclcpp::Time & stamp);
  void set_color(bool is_red);
  rmoss_util::TaskStatus get_task_status_cb();
  bool control_task_cb(rmoss_util::TaskCmd cmd);

private:
  rclcpp::Node::SharedPtr node_;
  // 相机客户端
  std::shared_ptr<rmoss_cam::CamClient> cam_client_;
  // 自瞄算法类
  std::shared_ptr<SimpleAutoAimAlgo> auto_aim_algo_;
  // 坐标变换工具类
  std::shared_ptr<rmoss_projectile_motion::GimbalTransformTool> gimbal_tansformoss_tool_;
  rmoss_util::TaskManager::SharedPtr task_manager_;
  // ros pub, sub, srv
  rclcpp::Publisher<rmoss_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<rmoss_interfaces::msg::ShootCmd>::SharedPtr shoot_cmd_pub_;
  rclcpp::Subscription<rmoss_interfaces::msg::Gimbal>::SharedPtr gimbal_state_sub_;
  rclcpp::Service<rmoss_interfaces::srv::SetColor>::SharedPtr set_color_srv_;
  // tf and message filter
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // data
  std::string camera_name_;
  std::string target_color_{"red"};
  bool debug_{false};
  bool run_flag_{false};
  tf2::Duration transform_tolerance_;
  // tmp data
  bool gimbal_ctrl_flag_{true};
  bool shoot_ctrl_flag_{true};
};
}  // namespace rmoss_auto_aim

#endif  // RMOSS_AUTO_AIM__SIMPLE_AUTO_AIM_NODE_HPP_
