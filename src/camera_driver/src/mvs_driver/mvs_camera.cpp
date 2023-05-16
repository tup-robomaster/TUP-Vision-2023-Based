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
        RCLCPP_INFO_ONCE(logger_, "Camera init called...");
        status = CameraSdkInit(1);
        status = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
        if (status != CAMERA_STATUS_SUCCESS){
            RCLCPP_ERROR(logger_, "No camera was found! Error code:%d", status);
            return false;
        }

        BOOL is_opened = 0;
        status = CameraIsOpened(&tCameraEnumList, &is_opened);
        if (is_opened) {
            RCLCPP_ERROR(logger_, "Camera is opened in other process ...");
        } 

        status = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
        if (status == CAMERA_STATUS_SUCCESS) {
            RCLCPP_INFO(logger_, "Camera init SUCCESS...");
        } else {
            RCLCPP_FATAL(logger_, "Camera init FAILED! Error code:%d", status);
        }
        
        status = CameraGetCapability(hCamera, &tCapability);
        // 计算RGB buffer所需的大小，这里直接按照相机的最大分辨率来分配
        UINT FrameBufferSize = tCapability.sResolutionRange.iWidthMax * tCapability.sResolutionRange.iHeightMax * 3;
        // g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
        g_pRgbBuffer = (BYTE *)CameraAlignMalloc(FrameBufferSize, 16);

        is_camera_initialized_ = true;
        return is_camera_initialized_;
    }

    bool MvsCamera::open()
    {
        RCLCPP_INFO_ONCE(logger_, "Camera openning...");

        status = CameraConnectTest(hCamera);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_ERROR(logger_, "Camera Offline! Error code: [%d]", status);
            status = CameraReConnect(hCamera);
            if (status != CAMERA_STATUS_SUCCESS)
            {
                RCLCPP_FATAL(logger_, "Camera ReConnect Failed! Error code: [%d]", status);
            }
        }

        // 加载参数配置文件
        char config_path_str[this->cam_param_.config_path.length()];
        strcpy(config_path_str, this->cam_param_.config_path.data());
        status = CameraReadParameterFromFile(hCamera, config_path_str);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Read Cam Param failed! Error code: [%d]", status);
        }

        // INT media_type = CAMERA_MEDIA_TYPE_BGR8;
        // status = CameraGetMediaType(hCamera, &media_type);
        // if (status == CAMERA_STATUS_SUCCESS) {
        //     RCLCPP_INFO(logger_, "Camera MediaType Get SUCCESS...");
        // } else {
        //     RCLCPP_FATAL(logger_, "Camera MediaType Get FAILED! Error code:%d", status);
        // }
        // RCLCPP_INFO(logger_, "media_type:%x", media_type);

        // status = CameraSetMediaType(hCamera, media_type);
        // if (status == CAMERA_STATUS_SUCCESS) {
        //     RCLCPP_INFO(logger_, "Camera MediaType Set SUCCESS...");
        // } else {
        //     RCLCPP_FATAL(logger_, "Camera MediaType Set FAILED! Error code:%d", status);
        // }

        // 相机模式切换成连续采集
        // Switch camera mode to continuous acquisition
        // 一般情况，0表示连续采集模式；1表示软件触发模式；2表示硬件触发模式。
        // status = CameraSetTriggerMode(hCamera, 0);

        // CameraSetTriggerMode(hCamera, 0);
        // setResolution(this->cam_param_.image_width, this->cam_param_.image_height);
        
        // TODO : 更新时间戳，设置时间戳偏移量
        // updateTimestamp(time_start_);
        
        // 设置曝光事件
        // setExposureTime(this->cam_param_.exposure_time);
        
        // TODO : 设置增益
        // setGain(3, this->cam_param_.exposure_gain);
        
        // 是否启用自动白平衡
        // bool usingAutoWb = false;
        // status = CameraSetWbMode(hCamera, usingAutoWb);
        // setBalance(0, 1.0);
        // setBalance(1, 1.0);
        // setBalance(2, 1.0);

        // INT iIspProcessor = 0;
        // INT *piAlgorithmSel = nullptr;
        // status = CameraGetBayerDecAlgorithm(hCamera, iIspProcessor, piAlgorithmSel);

        // CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 500);
        
        // 让SDK内部取图线程开始工作
        status = CameraPlay(hCamera);

        return true;
    }

    bool MvsCamera::close()
    {
        // 保存参数配置文件
        char config_path_str[this->cam_param_.config_path.length()];
        strcpy(config_path_str, this->cam_param_.config_path.data());
        status = CameraSaveParameterToFile(hCamera, config_path_str);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Save Cam Param failed! Error code: [%d]", status);
        }

        status = CameraStop(hCamera);
        status = CameraUnInit(hCamera);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Camera Close failed! Error code: [%d]", status);
        }
        CameraAlignFree(g_pRgbBuffer);
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Free Align image buffer failed! Error code: [%d]", status);
        }

        return true;
    }

    void MvsCamera::deviceReset()
    {
        
    }

    bool MvsCamera::isOpen()
    {
        return is_open_;
    }

    void MvsCamera::setResolution(int width, int height)
    {
        // TODO: Set Resolution
        status = CameraSetImageResolution(hCamera, &tCapability.pImageSizeDesc[0]); // 设置图像分辨率
        if (status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Set Resolution failed! Error code: [%d]", status);
        }
    }
    void MvsCamera::setExposureTime(float ExposureTime)
    {   // 手动曝光
        bool usingAe = false;
        status = CameraSetAeState(hCamera, usingAe);
        
        // 设置曝光时间
        status = CameraSetExposureTime(hCamera, ExposureTime);
        if(status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Set exposure time failed! Error code: [%d]", status);
        }
    }

    void MvsCamera::setBalance(int value, unsigned int value_num)
    {
        status = CameraSetOnceWB(hCamera);
        if(status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Set OnceWB failed! Error code: [%d]", status);
        }
    }

    void MvsCamera::setGain(int value, int exp_gain)
    {
        // CameraSetGain(hCamera, gainRValue, gainGValue, gainBValue);
        status = CameraSetAnalogGain(hCamera, exp_gain);
        if(status != CAMERA_STATUS_SUCCESS)
        {
            RCLCPP_WARN(logger_, "Set AnalogGain failed! Error code: [%d]", status);
        }
    }

    bool MvsCamera::getImage(::cv::Mat &Src, sensor_msgs::msg::Image& image_msg)
    {
        // RCLCPP_INFO(logger_, "Camera getImage called...");
        if (is_camera_initialized_) {
            status = CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000);
            if (status != CAMERA_STATUS_SUCCESS) {
                RCLCPP_WARN(logger_, "Get image buffer failed! Error code: %d", status);
                return false;
            }
            if (CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo) == CAMERA_STATUS_SUCCESS) {
                Src = cv::Mat(tCapability.pImageSizeDesc->iHeight, tCapability.pImageSizeDesc->iWidth, CV_8UC3);
                memcpy(Src.data, g_pRgbBuffer, tCapability.pImageSizeDesc->iWidth * tCapability.pImageSizeDesc->iHeight * 3 * sizeof(unsigned char));
                cvtColor(Src, Src, COLOR_RGB2BGR);
                
                // for (int ii = 0; ii < 10; ii++)
                // {
                //     cout << pbyBuffer[ii] << " ";
                // }
                image_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(Src.step);
                image_msg.is_bigendian = false;
                image_msg.data.assign(Src.datastart, Src.dataend);

                status = CameraReleaseImageBuffer(hCamera, pbyBuffer);
                if (status != CAMERA_STATUS_SUCCESS) {
                    RCLCPP_WARN(logger_, "Release image buffer failed! Error code: %d", status);
                    // return false;
                }
                return true;
            } else {
                return false;
            }
        }
    }
}