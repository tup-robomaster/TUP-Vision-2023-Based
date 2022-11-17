/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:13:23
 * @LastEditTime: 2022-11-14 10:49:42
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/src/hik_driver/hik_camera.cpp
 */
#include "../../include/hik_driver/hik_camera.hpp"

namespace camera_driver 
{
    HikCamera::HikCamera(const HikCamParam& cam_params)
    {
        g_nPayloadSize = 0;

        //params set
        this->hik_cam_params_ = cam_params;
    }

    HikCamera::~HikCamera()
    {
        nRet = MV_CC_FreeImageBuffer(handle, (&pFrame));
        if(nRet != MV_OK)
        {
            // fmt::print(fmt::fg(fmt::color::red), "[Camera] free image buffer failed!\n");
        }
    } 

    bool HikCamera::open()
    {
        //TODO:
        set_digital_io_control();

        set_trigger_mode();
        
        start_device(this->hik_cam_params_.hik_cam_id);
        // printf("9\n");
        // 设置分辨率
        set_resolution(this->hik_cam_params_.image_width, this->hik_cam_params_.image_height);
        
        //更新时间戳，设置时间戳偏移量
        update_timestamp(time_start);
        
        // 开始采集帧
        set_stream_on();

        // 设置曝光事件
        set_exposure_time(this->hik_cam_params_.exposure_time);

        // 设置1
        // SetGAIN(0, hik_cam_params.exposure_gain_b);
        // SetGAIN(1, hik_cam_params.exposure_gain_g);
        // SetGAIN(2, hik_cam_params.exposure_gain_r;
        set_gain(3, this->hik_cam_params_.exposure_gain);
        // 是否启用自动白平衡7
        // Set_BALANCE_AUTO(0);
        // manual白平衡 BGR->012
        set_balance(0, this->hik_cam_params_.balance_b);
        set_balance(1, this->hik_cam_params_.balance_g);
        set_balance(2, this->hik_cam_params_.balance_r);

        // _is_open = get_frame(frame);

        return true;
    }

    bool HikCamera::close() 
    {
        return false;
    }

    bool HikCamera::is_open()
    {
        return _is_open;
    }

    void HikCamera::start_device(int serial_number)
    {   //打开设备

        // printf("7\n");
        
        MV_CC_DEVICE_INFO_LIST stDeviceList;

        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

        // printf("5\n");
        try{
            //枚举设备
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            if (MV_OK != nRet)
            {
                printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
                exit(-1);
            }
            // nRetError(nRet, "EnumDevices failed! nRet [%x]\n", nRet);
        }
        catch(const std::exception &ex)
        {
            std::cout << ex.what() << std::endl;
        }
        // printf("a\n");

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
            // nRetError(nRet, "CreateHandle failed! nRet [%x]\n", nRet);
            // printf("b\n");

            
            //打开设备
            nRet = MV_CC_OpenDevice(handle);
            // nRetError(nRet, "[Camera] open failed! nRet [%x]\n", nRet);
            // printf("c\n");

        }
        else
        {
            // printf("4\n");
            // fmt::print(fmt::fg(fmt::color::red), "[Camera] Find No Devices!\n");
            return ;
        }

        return ;
    }

    bool HikCamera::set_stream_on()
    {   //开始采集
        //设置触发模式为off
        this->nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        // nRetError(nRet, "[Camera] SetEnumValue TriggerMode failed! nRet [%x]\n", nRet);

        //设置采集模式为连续采集
        this->nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", 2);
        // nRetError(nRet, "[Camera] SetEnumValue AcquisitionMode failed! nRet [%x]\n", nRet);

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

    bool HikCamera::set_resolution(int width, int height)
    {   //TODO:分辨率根据相机采集上限设置，目前设置为1280*1024
        nRet = MV_OK;

        //设置像素格式
        nRet = MV_CC_SetPixelFormat(handle, PixelType_Gvsp_BGR8_Packed);
        // nRetError(nRet, "[Camera] setPixelFormat failed! nRet [%x]\n", nRet);

        nRet = MV_CC_SetIntValue(this->handle, "Width", width);
        // nRetError(nRet, "[Camera] setResolution width failed! nRet [%x]\n", nRet);

        nRet = MV_CC_SetIntValue(this->handle, "Height", height);
        // nRetError(nRet, "[Camera] setResolution height failed! nRet [%x]\n", nRet);

        return true;
    }

    bool HikCamera::set_exposure_time(float ExposureTime)
    {   //设置曝光时间
        nRet = MV_CC_SetFloatValue(this->handle, "ExposureTime", ExposureTime);
        return true;
        // nRetError(nRet, "[CAMERA] set exposure time failed! nRet [%x]\n", nRet);
    }

    bool HikCamera::set_gain(int value, int ExpGain)
    {   //曝光增益
        if(value == 0)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainMode", R_CHANNEL);
            // nRetError(nRet, "[CAMERA] set exposure gain R_channel failed! nRet [%x]\n", nRet);
        }
        else if(value == 1)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainMode", G_CHANNEL);
            // nRetError(nRet, "[CAMERA] set exposure gain G_channel failed! nRet [%x]\n", nRet);
        }
        else if(value == 2)
        {
            nRet = MV_CC_SetEnumValue(handle, "GainMode", B_CHANNEL);
            // nRetError(nRet, "[CAMERA] set exposure gain B_channel failed! nRet [%x]\n", nRet);
        }
        else
        {
            nRet = MV_CC_SetFloatValue(handle, "Gain", ExpGain);
            // nRetError(nRet, "[CAMERA] set exposure gain failed! nRet [%x]\n", nRet);
        }

        nRet = MV_CC_SetFloatValue(handle, "Gain", ExpGain);
        return true;
        // nRetError(nRet, "[CAMERA] set exposure gain failed! nRet [%x]\n", nRet);
    }

    bool HikCamera::set_auto_balance()
    {   //自动白平衡（具有记忆功能）
        this->nRet = MV_CC_SetEnumValue(this->handle, "BalanceWhiteAuto", 1);
        return true;
        // nRetError(nRet, "[CAMERA] set auto balance failed! nRet [%x]\n", nRet);
    }

    bool HikCamera::set_balance(int value, unsigned int value_number)
    {   //手动白平衡（具有记忆功能））
        //关闭自动白平衡
        this->nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF);
        // nRetError(nRet, "[CAMERA] close auto balance failed! nRet [%x]\n", nRet);

        //设置RGB三通道白平衡值
        if(value == 0)
        {
            this->nRet = MV_CC_SetBalanceRatioRed(handle, value_number);
            // nRetError(nRet, "[CAMERA] set R_Balance failed！ nRet [%x]\n", nRet);
        }
        else if(value == 1)
        {
            this->nRet = MV_CC_SetBalanceRatioGreen(handle, value_number);
            // nRetError(nRet, "[CAMERA] set G_Balance failed！ nRet [%x]\n", nRet);
        }
        else if(value == 2)
        {
            this->nRet = MV_CC_SetBalanceRatioBlue(handle, value_number);
            // nRetError(nRet, "[CAMERA] set B_Balance failed！ nRet [%x]\n", nRet);
        }
        return true;
    }

    bool HikCamera::set_gamma(bool set_status, double dGammaParam)
    {   //设置Gamma值
        if(set_status == true)
        {
            nRet = MV_CC_SetEnumValue(handle, "Gamma", dGammaParam);
            // nRetError(nRet, "[CAMERA] set gamma failed！ nRet [%x]\n", nRet);
            return false;
        }
        else
        {
            nRet = MV_CC_SetEnumValue(handle, "Gamma", dGammaParam);
            // nRetError(nRet, "[CAMERA] close gamma failed！ nRet [%x]\n", nRet);
            return false;
        }

        return true;
    }

    bool HikCamera::color_correct(bool value)
    {   //设置色彩校正
        if(value == true)
        {
            nRet = MV_CC_SetEnumValue(handle, "ColorCorrection", 1);
            // nRetError(nRet, "[CAMERA] set color correction failed！ nRet [%x]\n", nRet);
            return false;
        }
        else
        {
            nRet = MV_CC_SetEnumValue(handle, "ColorCorrection", 0);
            // nRetError(nRet, "[CAMERA] close color correction failed！ nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    bool HikCamera::set_contrast(bool set_status,int dContrastParam)
    {   //设置对比度
        if(set_status == true)
        {
            nRet = MV_CC_SetEnumValue(handle, "Contrast", dContrastParam);
            // nRetError(nRet, "[CAMERA] set contrast failed！ nRet [%x]\n", nRet);
            return false;
        }
        else
        {
            nRet = MV_CC_SetEnumValue(handle, "Contrast", dContrastParam);
            // nRetError(nRet, "[CAMERA] close contrast failed！ nRet [%x]\n", nRet);
            return false;
        }
        return true;
    }

    bool HikCamera::update_timestamp(std::chrono::_V2::steady_clock::time_point time_start)
    {   //计算时间戳偏移
        std::chrono::_V2::steady_clock::time_point time_end = std::chrono::_V2::steady_clock::now();
        std::chrono::duration<double> time_span = time_end - time_start;
        timestamp_offset = time_span.count() * 1000;
        return true;
    }

    int HikCamera::get_timestamp()
    {   //获取时间戳
        std::chrono::_V2::steady_clock::time_point time_start = std::chrono::_V2::steady_clock::now();

        nRet = MV_CC_SetCommandValue(handle, "GevTimestampControlLatch");
        if(MV_OK != nRet)
        {
            printf("获取时间戳失败...\n");
        }
        
        MVCC_INTVALUE timestamp;
        nRet = MV_CC_GetIntValue(handle, "GevTimestampValue", &timestamp);
        if(MV_OK != nRet)
        {
            printf("获取时间戳值失败...\n");
        }

        return ((int)time_start.time_since_epoch().count() - timestamp_offset);
    }

    bool HikCamera::get_frame(::cv::Mat &Src)
    {
        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        // nRetError(nRet, "[CAMERA] Get PayloadSize fail! nRet [%x]\n", nRet);

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
        // nRetError(nRet, "[CAMERA] No image data! nRet [%x]\n", nRet);

        //fps
        // nRet = MV_CC_GetFrameRate(handle, &frame_rate);
        // printf("FPS:%f\n", frame_rate.fCurValue);

        // memset(&stFrameInfo, 0, sizeof(MV_IMAGE_BASIC_INFO));

        // MV_CC_GetImageInfo(handle, &stFrameInfo);
        
        // printf("fps:%f fps_max:%f fps_min:%f\n", stFrameInfo.fFrameRateValue,
        // stFrameInfo.fFrameRateMax, stFrameInfo.fFrameRateMin);

        cv::Mat src = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3);
        memcpy(src.data, pData, stImageInfo.nWidth * stImageInfo.nHeight * 3);
        src.copyTo(Src);
        // cv::Mat src(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
        // src.copyTo(Src);

        if(pData)
        {
            delete[] pData;
            pData = NULL;
        }
        return true;
    }

    bool HikCamera::set_trigger_mode(unsigned int acquisition_mode,
        const char* acquisition_start,
        const char* acquisition_stop,
        unsigned int acquisition_burst_frame_count,
        unsigned int trigger_selector,
        unsigned int trigger_mode, 
        unsigned int trigger_source_line,
        unsigned int trigger_activation,
        unsigned int trigger_delay)
    {
        // 设置采集模式
        nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", acquisition_mode);
        if(MV_OK != nRet)
        {
            printf("设置采集模式失败...\n");
            return false;
        }
        
        // 设置采集一次的出图数
        nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", acquisition_burst_frame_count);
        if(MV_OK != nRet)
        {
            printf("设置采集一次的出图数失败...\n");
        }

        //设置触发模式
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_mode);
        if(!nRet)
        {
            printf("设置触发模式失败...\n");
            return false;
        }
        else
        {
            printf("设置触发模式为: %d\n", trigger_mode);
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

    bool HikCamera::set_digital_io_control(unsigned int line_selector,
        unsigned int line_mode,
        bool line_status,
        unsigned int trigger_selector)
    {
        //设置反馈信号输出线序
        nRet = MV_CC_SetEnumValue(handle, "LineSelector", line_selector);
        if(MV_OK != nRet)
        {
            printf("设置反馈信号输出线序失败...\n");
        }
        else
        {
            printf("设置反馈信号输出线序为： %d\n", line_selector);
        }

        //设置反馈信号种类
        nRet = MV_CC_SetEnumValue(handle, "LineMode", line_mode);
        if(MV_OK != nRet)
        {
            printf("设置反馈信号种类失败...\n");
        }
        else
        {
            printf("设置反馈信号种类为： %d\n", line_mode);
        }

        // 信号线连接状态
        nRet = MV_CC_GetBoolValue(handle, "LineStatus", &line_status);
        if(!line_status)
        {
            printf("反馈信号线通信失败...\n");
        }
        else
        {
            printf("反馈信号线通信成功...\n");
        }

        //反馈信号输出使能
    }
} // namespace camera_driver


