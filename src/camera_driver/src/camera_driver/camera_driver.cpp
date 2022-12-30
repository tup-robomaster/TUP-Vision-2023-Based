/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-12-30 21:16:12
 * @LastEditTime: 2022-12-30 21:43:28
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/camera_driver/camera_driver.cpp
 */
#include "../../include/camera_driver/camera_driver.hpp"

namespace camera_driver
{
    template<class T>
    CameraBaseNode<T>::CameraBaseNode()
    {

    }

    template<class T>
    CameraBaseNode<T>::~CameraBaseNode()
    {

    }

    template<class T>
    void CameraBaseNode<T>::image_callback()
    {

    }

    template<class T>
    std::unique_ptr<ImageMsg> CameraBaseNode<T>::convert_frame_to_msg(cv::Mat frame)
    {
        
    }

    template<class T>
    bool CameraBaseNode<T>::setParam(rclcpp::Parameter param)
    {

    }

    template<class T>
    rcl_interfaces::msg::SetParametersResult CameraBaseNode<T>::paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {

    }
} //namespace camera_driver