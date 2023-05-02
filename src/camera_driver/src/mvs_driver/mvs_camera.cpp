#include "../../include/mvs_driver/mvs_camera.hpp"

namespace camera_driver
{
    MvsCamera::MvsCamera()
    : logger_(rclcpp::get_logger("mvs_driver"))
    {
        auto is_init = init();
        if (!is_init)
        {
            RCLCPP_FATAL(logger_, "Camera initializing failed...");
        }
    }

    MvsCamera::MvsCamera(const CameraParam& cam_params)
    : logger_(rclcpp::get_logger("mvs_driver"))
    {
        // Params set.
        this->cam_param_ = cam_params;
        auto is_init = init();
        if (!is_init)
        {
            RCLCPP_FATAL(logger_, "Camera initializing failed...");
        }
    }

    MvsCamera::~MvsCamera()
    {
        auto is_release = close();
        if(!is_release)
        {
            RCLCPP_FATAL(logger_, "Camera initializing failed...");
        }
    }

    bool MvsCamera::init()
    {
        RCLCPP_FATAL(logger_, "Camera init called...");
        CameraSdkInit(1);
        // CameraSetImageResolution(hCamera, );
        CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
        CameraInit(&tCameraEnumList, -1, -1, &hCamera);
        // if (status == CAMERA_STATUS_SUCCESS) {
        //     RCLCPP_INFO(logger_, "Camera init SUCCESS...");
        // } else {
        //     RCLCPP_FATAL(logger_, "Camera init FAILED...");
        // }
        CameraGetCapability(hCamera, &tCapability);
        g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

        is_camera_initialized_ = true;

        return is_camera_initialized_;
    }

    bool MvsCamera::open()
    {
        RCLCPP_FATAL(logger_, "Camera open called...");

        // CameraSetTriggerMode(hCamera, 0);
        setResolution(this->cam_param_.image_width, this->cam_param_.image_height);
        // TODO : 更新时间戳，设置时间戳偏移量
        // updateTimestamp(time_start_);
        
        // 设置曝光事件
        setExposureTime(12000);
        // TODO : 设置增益
        setGain(3, this->cam_param_.exposure_gain);
        // 是否启用自动白平衡
        bool usingAutoWb = true;
        CameraSetWbMode(hCamera, usingAutoWb);
        
        // CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500);
        CameraPlay(hCamera);

        return true;
    }

    bool MvsCamera::setResolution(int width, int height)
    {
        // TODO: Set Resolution
        CameraSetImageResolution(hCamera, &tCapability.pImageSizeDesc[0]); // 设置图像分辨率

        return true;
    }

    bool MvsCamera::close()
    {
        CameraStop(hCamera);
        CameraUnInit(hCamera);

        return true;
    }

    bool MvsCamera::deviceReset()
    {
        return true;
    }

    bool MvsCamera::isOpen()
    {
        return is_open_;
    }

    bool MvsCamera::setExposureTime(float ExposureTime)
    {
        CameraSdkStatus status;

        bool usingAe = false;
        CameraSetAeState(hCamera, usingAe);

        //设置曝光时间
        status = CameraSetExposureTime(hCamera, ExposureTime);
        if(status != CAMERA_STATUS_SUCCESS)
            RCLCPP_WARN(logger_, "Set exposure time failed! nRet [%x]", status);
        return true;
    }

    bool MvsCamera::setBalance(int value, unsigned int value_num)
    {
        return true;
    }

    bool MvsCamera::setGain(int value, int exp_gain)
    {
        // CameraSetGain(hCamera, gainRValue, gainGValue, gainBValue);
        CameraSetAnalogGain(hCamera, exp_gain);
        return true;
    }

    bool MvsCamera::getImage(::cv::Mat &Src, sensor_msgs::msg::Image& image_msg)
    {
        // RCLCPP_FATAL(logger_, "Camera getImage called...");

        if (is_camera_initialized_) {
            CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 200);
            if (CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo) == CAMERA_STATUS_SUCCESS) {
                Src = cv::Mat(tCapability.pImageSizeDesc->iHeight, tCapability.pImageSizeDesc->iWidth, CV_8UC3);
                // if (pbyBuffer == nullptr) {
                //     cout << "buffer is null" << endl;
                // }
                memcpy(Src.data, g_pRgbBuffer, tCapability.pImageSizeDesc->iWidth * tCapability.pImageSizeDesc->iHeight * 3 * sizeof(unsigned char));
                // cvtColor(Src, Src, COLOR_BayerBG2RGB);
                
                image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(Src.step);
                image_msg.is_bigendian = false;
                image_msg.data.assign(Src.datastart, Src.dataend);

                CameraReleaseImageBuffer(hCamera, pbyBuffer);
                // if (status == CAMERA_STATUS_SUCCESS) {
                //     cout << "free success" << endl;
                // }

                return true;
            } else {
                return false;
            }
        }
    }

    bool MvsCamera::updateTimestamp(rclcpp::Time time_start)
    {   //计算时间戳偏移
        rclcpp::Time time_end = steady_clock_.now();
        double time_span = (time_end - time_start).nanoseconds();
        timestamp_offset_ = time_span / 1e9;
        return true;
    }
}