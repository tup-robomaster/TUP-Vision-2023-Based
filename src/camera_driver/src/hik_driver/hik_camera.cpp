/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:13:23
 * @LastEditTime: 2023-04-14 02:41:15
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/hik_driver/hik_camera.cpp
 */
#include "../../include/hik_driver/hik_camera.hpp"

namespace camera_driver 
{
    HikCamera::HikCamera()
    : logger_(rclcpp::get_logger("hik_driver"))
    {
        auto is_init = init();
        if (!is_init)
        {
            RCLCPP_FATAL(logger_, "Camera initializing failed...");
        }
    }

    HikCamera::HikCamera(const CameraParam& cam_params)
    : logger_(rclcpp::get_logger("hik_driver"))
    {
        // Params set.
        this->cam_param_ = cam_params;
        auto is_init = init();
        if (!is_init)
        {
            RCLCPP_FATAL(logger_, "Camera initializing failed...");
        }
    }

    HikCamera::~HikCamera()
    {
        auto is_release = close();
        if(!is_release)
        {
            RCLCPP_FATAL(logger_, "Camera initializing failed...");
        }
    } 

    bool HikCamera::init()
    {
        g_nPayloadSize = 0;
        return true;
    }

    bool HikCamera::open()
    {
        //TODO:
        setDigitalIoControl();
        setTriggerMode();
        startDevice(this->cam_param_.cam_id);
        // 设置分辨率
        setResolution(this->cam_param_.image_width, this->cam_param_.image_height);
        // 更新时间戳，设置时间戳偏移量
        updateTimestamp(time_start_);
        // 开始采集帧
        setStreamOn();
        // 设置曝光事件
        setExposureTime(this->cam_param_.exposure_time);
        // 设置增益
        // SetGAIN(0, hik_cam_params.exposure_gain_b);
        // SetGAIN(1, hik_cam_params.exposure_gain_g);
        // SetGAIN(2, hik_cam_params.exposure_gain_r;
        setGain(3, this->cam_param_.exposure_gain);
        // 是否启用自动白平衡7
        // Set_BALANCE_AUTO(0);
        // manual白平衡 BGR->012
        setBalance(0, this->cam_param_.balance_b);
        setBalance(1, this->cam_param_.balance_g);
        setBalance(2, this->cam_param_.balance_r);

        return true;
    }

    bool HikCamera::close() 
    {
        nRet = MV_CC_FreeImageBuffer(handle, (&pFrame));
        if(nRet != MV_OK)
        {
            RCLCPP_ERROR(logger_, "Free image buffer failed!");
            return false;
        }
        return true;
    }

    bool HikCamera::deviceReset()
    {
        return true;
    }

    bool HikCamera::isOpen()
    {
        return is_open_;
    }

    void HikCamera::startDevice(int serial_number)
    {   //打开设备
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        try{
            //枚举设备
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            if (MV_OK != nRet)
            {
                RCLCPP_ERROR(logger_, "MV_CC_EnumDevices fail! nRet [%x]", nRet);
                exit(-1);
            }
        }
        catch(const std::exception &ex)
        {
            RCLCPP_ERROR(logger_, "Error while enum devices:%s", ex.what());
        }

        if(stDeviceList.nDeviceNum > 0) //设备数量不为0
        {
            // for(int i = 0; i < stDeviceList.nDeviceNum; i++)
            // {
            //     printf("[device %d]:\n", i);
            //     MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
            //     if(NULL == pDeviceInfo)
            //     {
            //         printf("pDeviceInfo is NULL!\n");
            //         return -1;
            //     }
            //     HikCamera::PrintDeviceInfo(pDeviceInfo);
            // }

            //打开设备
            //选择设备并创建句柄
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[serial_number]);
            if(nRet != MV_OK)
                RCLCPP_ERROR(logger_, "CreateHandle failed! nRet [%x]", nRet);

            //打开设备
            nRet = MV_CC_OpenDevice(handle);
            if(nRet != MV_OK)
                RCLCPP_ERROR(logger_, "Open device failed! nRet [%x]", nRet);
        }
        else
        {
            RCLCPP_WARN(logger_, "Find No Devices!");
        }

        return ;
    }

    bool HikCamera::setStreamOn()
    {   //开始采集
        //设置触发模式为off
        this->nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if(nRet != MV_OK)
            RCLCPP_ERROR(logger_, "SetEnumValue TriggerMode failed! nRet [%x]", nRet);

        //设置采集模式为连续采集
        this->nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
        if(nRet != MV_OK)
            RCLCPP_ERROR(logger_, "SetEnumValue AcquisitionMode failed! nRet [%x]", nRet);

        // //获取数据包大小
        // MVCC_INTVALUE stParam;
        // memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        // nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        // if (MV_OK != nRet)
        // {
        //     printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
        //     return false;
        // }
        // g_nPayloadSize = stParam.nCurValue;

        //开始取流
        nRet = MV_CC_StartGrabbing(handle);

        return true;

        // nRetError(nRet, "[Camera] StartGrabbing failed! nRet [%x]\n", nRet);

        //创建相机图像采集线程
        // pthread_t tid;
        // nRet = pthread_create(&tid, NULL, HikCamera::WorkThread, handle);
        // if(MV_OK == nRet)
        // {
        //     printf("pthread_create succeed!\n");
        //     return true;
        // }
        // else
        // {
        //     printf("pthread_create failed! nRet [%d]\n", nRet);
        //     return false;
        // }
    }

    bool HikCamera::setResolution(int width, int height)
    {   //TODO:分辨率根据相机采集上限设置，目前设置为1280*1024
        nRet = MV_OK;

        //设置像素格式
        nRet = MV_CC_SetPixelFormat(handle, PixelType_Gvsp_BGR8_Packed);
        if(nRet != MV_OK)
        {
            RCLCPP_ERROR(logger_, "setPixelFormat failed! nRet [%x]", nRet);
        }

        nRet = MV_CC_SetIntValue(this->handle, "Width", width);
        if(nRet != MV_OK)
            RCLCPP_ERROR(logger_, "setResolution width failed! nRet [%x]", nRet);

        nRet = MV_CC_SetIntValue(this->handle, "Height", height);
        if(nRet != MV_OK)
            RCLCPP_ERROR(logger_, " setResolution height failed! nRet [%x]", nRet);
        return true;
    }

    bool HikCamera::setExposureTime(float ExposureTime)
    {   //设置曝光时间
        nRet = MV_CC_SetFloatValue(this->handle, "ExposureTime", ExposureTime);
        if(nRet != MV_OK)
            RCLCPP_WARN(logger_, "Set exposure time failed! nRet [%x]", nRet);
        return true;
    }

    bool HikCamera::setGain(int value, int ExpGain)
    {   //曝光增益
        if(value == 0)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainMode", R_CHANNEL);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "set exposure gain R_channel failed! nRet [%x]", nRet);
        }
        else if(value == 1)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainMode", G_CHANNEL);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "set exposure gain G_channel failed! nRet [%x]", nRet);
        }
        else if(value == 2)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainMode", B_CHANNEL);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "set exposure gain B_channel failed! nRet [%x]", nRet);
        }
        else
        {
            nRet = MV_CC_SetFloatValue(handle, "Gain", ExpGain);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "set exposure gain failed! nRet [%x]\n", nRet);
        }
        nRet = MV_CC_SetFloatValue(handle, "Gain", ExpGain);
        if(nRet != MV_OK)
            RCLCPP_WARN(logger_, "set exposure gain failed! nRet [%x]\n", nRet);
        return true;
    }

    bool HikCamera::setAutoBalance()
    {   //自动白平衡（具有记忆功能）
        this->nRet = MV_CC_SetEnumValue(this->handle, "BalanceWhiteAuto", 1);
        if(nRet != MV_OK)
            RCLCPP_WARN(logger_, "Set auto balance failed! nRet [%x]", nRet);
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

    bool HikCamera::setGamma(bool set_status, double dGammaParam)
    {   //设置Gamma值
        if(set_status == true)
        {
            nRet = MV_CC_SetEnumValue(handle, "Gamma", dGammaParam);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Set B_Balance failed！ nRet [%x]", nRet);
            return false;
        }
        else
        {
            nRet = MV_CC_SetEnumValue(handle, "Gamma", dGammaParam);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Close gamma failed！ nRet [%x]", nRet);
            return false;
        }
        return true;
    }

    bool HikCamera::colorCorrect(bool value)
    {   //设置色彩校正
        if(value == true)
        {
            nRet = MV_CC_SetEnumValue(handle, "ColorCorrection", 1);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Set color correction failed！ nRet [%x]\n", nRet);
            return false;
        }
        else
        {
            nRet = MV_CC_SetEnumValue(handle, "ColorCorrection", 0);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Close color correction failed！ nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    bool HikCamera::setContrast(bool set_status,int dContrastParam)
    {   //设置对比度
        if(set_status == true)
        {
            nRet = MV_CC_SetEnumValue(handle, "Contrast", dContrastParam);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "set contrast failed！ nRet [%x]\n", nRet);
            return false;
        }
        else
        {
            nRet = MV_CC_SetEnumValue(handle, "Contrast", dContrastParam);
            if(nRet != MV_OK)
                RCLCPP_WARN(logger_, "Close contrast failed！ nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    bool HikCamera::updateTimestamp(rclcpp::Time time_start)
    {   //计算时间戳偏移
        rclcpp::Time time_end = steady_clock_.now();
        double time_span = (time_end - time_start).nanoseconds();
        timestamp_offset_ = time_span / 1e9;
        return true;
    }

    int HikCamera::getTimestamp()
    {   //获取时间戳
        std::chrono::_V2::steady_clock::time_point time_start = std::chrono::_V2::steady_clock::now();

        nRet = MV_CC_SetCommandValue(handle, "GevTimestampControlLatch");
        if(MV_OK != nRet)
            RCLCPP_WARN(logger_, "获取时间戳失败...");
        
        MVCC_INTVALUE timestamp;
        nRet = MV_CC_GetIntValue(handle, "GevTimestampValue", &timestamp);
        if(MV_OK != nRet)
            RCLCPP_WARN(logger_, "获取时间戳失败...");

        return ((int)time_start.time_since_epoch().count() - timestamp_offset_);
    }

    bool HikCamera::getImage(::cv::Mat &Src, sensor_msgs::msg::Image& image_msg)
    {
        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if(nRet != MV_OK)
        {
            RCLCPP_ERROR(logger_, "Get PayloadSize fail! nRet [%x]\n", nRet);
            return false;
        }

        MV_FRAME_OUT_INFO_EX stImageInfo;
        memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
        unsigned char * pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
        if (NULL == pData)
        {
            return NULL;
        }
        unsigned int nDataSize = stParam.nCurValue;
        
        //从缓存区读取图像
        nRet = MV_CC_GetOneFrameTimeout(handle, pData, nDataSize, &stImageInfo, 1000);
        if(nRet != MV_OK)
        {
            RCLCPP_ERROR(logger_, "No image data! nRet [%x]", nRet);
            return false;
        }

        // fps
        // nRet = MV_CC_GetFrameRate(handle, &frame_rate);
        // printf("FPS:%f\n", frame_rate.fCurValue);

        // memset(&stFrameInfo, 0, sizeof(MV_IMAGE_BASIC_INFO));

        // MV_CC_GetImageInfo(handle, &stFrameInfo);
        
        // printf("fps:%f fps_max:%f fps_min:%f\n", stFrameInfo.fFrameRateValue,
        // stFrameInfo.fFrameRateMax, stFrameInfo.fFrameRateMin);

        // cv::Mat src(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
        Src = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3);
        memcpy(Src.data, pData, stImageInfo.nWidth * stImageInfo.nHeight * 3);
        // src.copyTo(Src);
        image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(Src.step);  
        image_msg.is_bigendian = false;
        image_msg.data.assign(Src.datastart, Src.dataend);

        if(pData)
        {
            delete[] pData;
            pData = NULL;
        }
        return true;
    }

    bool HikCamera::setTriggerMode(TriggerSetting trigger_setting)
    {
        // 设置采集模式
        nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", trigger_setting.acquisition_mode);
        if(MV_OK != nRet)
        {
            RCLCPP_WARN(logger_, "设置采集模式失败...");
            return false;
        }
        
        // 设置采集一次的出图数
        nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", trigger_setting.acquisition_burst_frame_count);
        if(MV_OK != nRet)
        {
            RCLCPP_WARN(logger_, "设置采集一次的出图数失败...");
            return false;
        }

        //设置触发模式
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_setting.trigger_mode);
        if(MV_OK != nRet)
        {
            RCLCPP_WARN(logger_, "设置触发模式失败...");
            return false;
        }
        else
        {
            RCLCPP_INFO(logger_, "设置触发模式为: %d", trigger_setting.trigger_mode);
        }

        //设置交叠曝光

        //设置触发源
        // nRet = MV_CC_SetEnumValue(handle, "TriggerSource", trigger_source_line);
        // if(!nRet)
        // {
        //     printf("设置触发源为line2失败...\n");
        //     return false;
        // }
        // else
        // {
        //     printf("设置触发源为: %d\n", trigger_source_line);
        // }
        
        //设置触发激活方式
        // nRet = MV_CC_SetEnumValue(handle, "TriggerActivation", trigger_activation);
        // if(!nRet)
        // {
        //     printf("设置触发激活方式为上升沿失败...\n");
        //     return false;
        // }
        // else
        // {
        //     printf("设置触发激活方式为: %d\n", trigger_activation);
        // }
        
        //设置触发延时
        // nRet = MV_CC_SetEnumValue(handle, "TriggerDelay", trigger_delay);
        // if(!nRet)
        // {
        //     printf("设置触发延时失败...\n");
        //     return false;
        // }
        // else
        // {
        //     printf("设置触发延时为： %d\n", trigger_delay);
        // }

        return true;
    }

    bool HikCamera::setDigitalIoControl(IoControlSetting io_control_setting)
    {
        //设置反馈信号输出线序
        nRet = MV_CC_SetEnumValue(handle, "LineSelector", io_control_setting.line_selector);
        if(MV_OK != nRet)
        {
            RCLCPP_WARN(logger_, "设置反馈信号输出线序失败...");
            return false;
        }
        else
        {
            RCLCPP_INFO(logger_, "设置反馈信号输出线序为：%d", io_control_setting.line_selector);
        }

        //设置反馈信号种类
        nRet = MV_CC_SetEnumValue(handle, "LineMode", io_control_setting.line_mode);
        if(MV_OK != nRet)
        {
            RCLCPP_WARN(logger_, "设置反馈信号种类失败: [%x]", nRet);
            return false;
        }
        else
        {
            RCLCPP_INFO(logger_, "设置反馈信号种类为： %d", io_control_setting.line_mode);
        }

        //信号线连接状态
        nRet = MV_CC_GetBoolValue(handle, "LineStatus", &io_control_setting.line_status);
        if(!io_control_setting.line_status)
        {
            RCLCPP_ERROR(logger_, "反馈信号线通信失败: [%x]", nRet);
            return false;
        }
        else
        {
            RCLCPP_INFO(logger_, "反馈信号线通信成功...");
        }
        //反馈信号输出使能

        return true;
    }
} // namespace camera_driver


