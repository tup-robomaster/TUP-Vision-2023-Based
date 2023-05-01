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
        g_nPayloadSize = 0;
        return true;
    }

    bool MvsCamera::open()
    {
        CameraSdkInit(1);
        CameraSetImageResolution(hCamera, );
        CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
        status = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
        CameraGetCapability(hCamera, &tCapability);
        CameraSetImageResolution(hCamera, &tCapability.pImageSizeDesc[0]); // 设置图像分辨率

        is_open_ = true;

        return true;
    }

    bool MvsCamera::setResolution(int width, int height)
    {
        // TODO: Set Resolution
        // 是否使用自动曝光
        bool usingAe = false;
        CameraSetAeState(hCamera, usingAe);
        // 设置曝光事件
        setExposureTime(this->cam_param_.exposure_time);
        // TODO: 设置增益
        // CameraSetGain(hCamera, gainRValue, gainGValue, gainBValue);
        // CameraSetSharpness(hCamera, sharpnessValue);
        // 是否启用自动白平衡
        bool usingAutoWb = true;
        CameraSetWbMode(hCamera, usingAutoWb);
        // CameraSetSaturation(hCamera, saturationValue);

        CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500);
        CameraPlay(hCamera);

        return true
    }

    bool MvsCamera::close()
    {
        CameraStop(hCamera);
        CameraUnInit(hCamera);

        destroyAllWindows();

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

        //设置曝光时间
        status = CameraSetExposureTime(hCamera, ExposureTime);
        if(status != CAMERA_STATUS_SUCCESS)
            RCLCPP_WARN(logger_, "Set exposure time failed! nRet [%x]", status);
        return true;
    }

    bool HikCamera::setBalance(int value, unsigned int value_number)
    {   //手动白平衡（具有记忆功能））
        //关闭自动白平衡
        this->nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF);
        if(nRet != MV_OK)
            RCLCPP_WARN(logger_, "Close auto balance failed! nRet [%x]", nRet);

        //设置RGB三通道白平衡值
        if(value == 0)
        {
            this->nRet = MV_CC_SetBalanceRatioRed(handle, value_number);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Set R_Balance failed！ nRet [%x]", nRet);
        }
        else if(value == 1)
        {
            this->nRet = MV_CC_SetBalanceRatioGreen(handle, value_number);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Set G_Balance failed！ nRet [%x]", nRet);
        }
        else if(value == 2)
        {
            this->nRet = MV_CC_SetBalanceRatioBlue(handle, value_number);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Set B_Balance failed！ nRet [%x]", nRet);
        }
        return true;
    }

    bool MvsCamera::getImage(::cv::Mat &Src, sensor_msgs::msg::Image& image_msg)
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
            frame.create(sFrameInfo.iHeight, sFrameInfo.iWidth, CV_8UC1);
            memcpy(frame.data, pbyBuffer, sFrameInfo.iWidth * sFrameInfo.iHeight * sizeof(unsigned char));
            cvtColor(frame, frame, COLOR_BayerBG2RGB);
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
            image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(Src.step);  
            image_msg.is_bigendian = false;
            image_msg.data.assign(Src.datastart, Src.dataend);

            return true;
        } else {
            return false;
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