/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:13:23
 * @LastEditTime: 2022-09-30 10:25:54
 * @FilePath: /tup_2023/src/camera_driver/src/hik_driver/hik_camera.cpp
 */
#include "../../include/hik_driver/hik_camera.hpp"

namespace camera_driver
{
    hik_camera::hik_camera()
    {
        g_nPayloadSize = 0;
    }

    hik_camera::~hik_camera()
    {
        nRet = MV_CC_FreeImageBuffer(handle, (&pFrame));
        if(nRet != MV_OK)
        {
            // fmt::print(fmt::fg(fmt::color::red), "[Camera] free image buffer failed!\n");
        }
    } 

    bool hik_camera::open()
    {
        //TODO:

        printf("6\n");
        start_device(0);
        printf("9\n");
        // 设置分辨率
        set_resolution(1440, 1080);
        
        //更新时间戳，设置时间戳偏移量
        update_timestamp(time_start);
        
        // 开始采集帧
        set_stream_on();

        // 设置曝光事件
        set_exposure_time(4000);

        // 设置1
        // SetGAIN(0, 16);
        // SetGAIN(1, 8);
        // SetGAIN(2, 8);
        set_gain(3, 16);
        // 是否启用自动白平衡7
        // Set_BALANCE_AUTO(0);
        // manual白平衡 BGR->012
        set_balance(0, 1690);
        set_balance(1, 1024);
        set_balance(2, 2022);

        // _is_open = get_frame(frame);

        return true;
    }

    bool hik_camera::close() 
    {
        return false;
    }

    bool hik_camera::is_open()
    {
        return _is_open;
    }

    void hik_camera::start_device(int serial_number)
    {   //打开设备

        printf("7\n");
        
        MV_CC_DEVICE_INFO_LIST stDeviceList;

        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));


        printf("5\n");
        try{
            //枚举设备
            nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
            // nRetError(nRet, "EnumDevices failed! nRet [%x]\n", nRet);
        }
        catch(const std::exception &ex)
        {
            std::cout << ex.what() << std::endl;
        }
        printf("a\n");

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
            //     hik_camera::PrintDeviceInfo(pDeviceInfo);
            // }

            //打开设备
            //选择设备并创建句柄
            nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[serial_number]);
            // nRetError(nRet, "CreateHandle failed! nRet [%x]\n", nRet);
            printf("b\n");

            
            //打开设备
            nRet = MV_CC_OpenDevice(handle);
            // nRetError(nRet, "[Camera] open failed! nRet [%x]\n", nRet);
            printf("c\n");

        }
        else
        {
            printf("4\n");
            // fmt::print(fmt::fg(fmt::color::red), "[Camera] Find No Devices!\n");
            return ;
        }

        return ;
    }

    bool hik_camera::set_stream_on()
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
        // nRet = pthread_create(&tid, NULL, hik_camera::WorkThread, handle);
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

    bool hik_camera::set_resolution(int width, int height)
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

    bool hik_camera::set_exposure_time(float ExposureTime)
    {   //设置曝光时间
        nRet = MV_CC_SetFloatValue(this->handle, "ExposureTime", ExposureTime);
        return true;
        // nRetError(nRet, "[CAMERA] set exposure time failed! nRet [%x]\n", nRet);
    }

    bool hik_camera::set_gain(int value, int ExpGain)
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

    bool hik_camera::set_auto_balance()
    {   //自动白平衡（具有记忆功能）
        this->nRet = MV_CC_SetEnumValue(this->handle, "BalanceWhiteAuto", 1);
        return true;
        // nRetError(nRet, "[CAMERA] set auto balance failed! nRet [%x]\n", nRet);
    }

    bool hik_camera::set_balance(int value, unsigned int value_number)
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

    bool hik_camera::set_gamma(bool set_status, double dGammaParam)
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

    bool hik_camera::color_correct(bool value)
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

    bool hik_camera::set_contrast(bool set_status,int dContrastParam)
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

    bool hik_camera::update_timestamp(std::chrono::_V2::steady_clock::time_point time_start)
    {   //计算时间戳偏移
        std::chrono::_V2::steady_clock::time_point time_end = std::chrono::_V2::steady_clock::now();
        std::chrono::duration<double> time_span = time_end - time_start;
        timestamp_offset = time_span.count() * 1000;
        return true;
    }

    int hik_camera::get_timestamp()
    {   //获取时间戳
        std::chrono::_V2::steady_clock::time_point time_start = std::chrono::_V2::steady_clock::now();
        return ((int)time_start.time_since_epoch().count() - timestamp_offset);
    }

    bool hik_camera::get_frame(::cv::Mat &Src)
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
} // namespace camera_driver


