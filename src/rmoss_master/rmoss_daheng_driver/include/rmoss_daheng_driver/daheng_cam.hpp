/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 00:50:25
 * @LastEditTime: 2022-09-06 14:58:23
 * @FilePath: /tup_2023/src/rmoss_master/rmoss_daheng_driver/include/rmoss_daheng_driver/daheng_cam.hpp
 */
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

/*
 * @Author: holakk
 * @Date: 2021-11-05 17:24:21
 * @LastEditors: holakk
 * @LastEditTime: 2021-11-10 17:11:47
 * @Description: file content
 */
#ifndef RM_CAM__DAHENG_CAM_HPP
#define RM_CAM__DAHENG_CAM_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"

#include "GxIAPI.h"
#include "DxImageProc.h"

#include "opencv2/opencv.hpp"
#include "rmoss_cam/cam_interface.hpp"

#define GXError(status, ...)           \
    if (status != GX_STATUS_SUCCESS)   \
    {                                  \
        RCLCPP_FATAL(                  \
            this->_node->get_logger(), \
            __VA_ARGS__);              \
        return false;                  \
    }
#define DXError(status, ...)           \
    if (status != DX_OK)               \
    {                                  \
        RCLCPP_FATAL(                  \
            this->_node->get_logger(), \
            __VA_ARGS__);              \
        return false;                  \
    }
namespace rmoss_entity_cam
{
    // MindVision相机类
    class DaHengCam : public rmoss_cam::CamInterface
    {
    public:
        /** Construct
         * @param camera_sn 相机SN号
         * @param node ROS节点指针
         * @param config_path 相机配置文件路径
         **/
        explicit DaHengCam(
            const std::string &camera_sn,
            rclcpp::Node::SharedPtr node,
            const std::string &config_path = "",
            const std::string &lut_config = "",
            const std::vector<double> &lut_detail = {0, 2., 0.});
        ~DaHengCam();
        bool open() override;
        bool close() override;
        bool is_open() override;
        bool grab_image(cv::Mat &image) override;
        bool set_parameter(rmoss_cam::CamParamType type, int value) override;
        bool get_parameter(rmoss_cam::CamParamType type, int & value) override;
        std::string error_message() override;

    private:
        bool load_config(const std::string &path);
        bool load_lut(const std::string &path);

    private:
        std::string _camera_sn;        // 相机sn号
        rclcpp::Node::SharedPtr _node; // 相机节点
        std::string _config_path;      // 相机配置文件路径
        std::string _lut_path;         // 相机lut文件路径

        GX_DEV_HANDLE _hDevice;            //相机对象
        GX_DEVICE_BASE_INFO _cameraInfo;   // 相机特征信息
        COLOR_IMG_PROCESS _stClrImageProc; // 图像转换处理配置, 需要销毁内存
        PGX_FRAME_BUFFER _pFrameBuffer;    // RAW图像Buffer
        std::vector<double> _lut_detail;    // lut参数
        bool _is_open;                     // 相机状态

        std::unordered_map<rmoss_cam::CamParamType, int> _param; // 摄像头参数
    };
} // namespace rmoss_entity_cam

#endif // RM_CAM__MINDVISION_CAM_HPP