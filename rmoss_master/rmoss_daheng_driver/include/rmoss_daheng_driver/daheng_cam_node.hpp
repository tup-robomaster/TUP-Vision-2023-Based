// Copyright 2022 robomaster-oss.
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

#ifndef RM_CAM__DAHENG_CAM_NODE_HPP
#define RM_CAM__DAHENG_CAM_NODE_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "rmoss_cam/cam_server.hpp"
#include "rmoss_daheng_driver/daheng_cam.hpp"

namespace rmoss_entity_cam
{
    // Node warpper for DaHengCamera
    class DaHengCamNode
    {
    public:
        explicit DaHengCamNode(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
        {
            return node_->get_node_base_interface();
        }

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<DaHengCam> cam_dev_;
        std::shared_ptr<rmoss_cam::CamServer> cam_server_;
    };
}

#endif // RM_CAM__DAHENG_CAM_NODE_HPP