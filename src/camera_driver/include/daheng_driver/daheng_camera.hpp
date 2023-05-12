/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-09 12:12:19
 * @LastEditTime: 2023-04-14 02:47:12
 * @FilePath: /TUP-Vision-2023-Based/src/camera_driver/include/daheng_driver/daheng_camera.hpp
 */
//daheng
#include "../../dependencies/daheng_sdk/include/DxImageProc.h"
#include "../../dependencies/daheng_sdk/include/GxIAPI.h"

//c++
#include <iostream>

//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "../../global_user/include/global_user/global_user.hpp"

using namespace std;
using namespace cv;
using namespace global_user;
namespace camera_driver
{
    //本类现只应对使用单个相机的情况
    class DaHengCam
    {
    private:
        //枚举操作成功与失败信息
        GX_STATUS status = GX_STATUS_SUCCESS;
        //相机设备
        GX_DEV_HANDLE hDevice = NULL;
        //定 义 GXDQBuf 的 传 入 参 数,包含图片内存，大小等信息
        PGX_FRAME_BUFFER pFrameBuffer;

        bool                set_contrast;               ///< Contrast Set
        bool                set_color;
        bool                set_saturation;

        int                 timestamp_offset = 0;
        int64_t             m_i64ColorCorrection;       ///< Color correction param
        int64_t             lastImgTimestamp;           ///< timestamp of last img
        void*               pGammaLut;                  ///< Gamma look up table
        int                 nLutLength;                 ///< Gamma look up table length
        void*               pContrastLut;             ///< Contrast look up table
        int                 m_nContrastLutLength;       ///< Contrast look up table length
        double              dGammaParam;
        VxInt32             contrast_factor;                    ///< Contrast Set factor
        VxInt32             saturation_factor;
        int64_t             nContrastParam;
        int64_t             nColorCorrectionParam;
        VxInt16             nSaturation;

        // char *pRGB24Buf;

    public:
        //构造函数，初始化库
        DaHengCam(CameraParam daheng_param);
        DaHengCam();
        //析构函数释放资源
        ~DaHengCam();
        
    public:
        bool init();
        bool open();
        bool close();
    
    private:
        //打开设备
        int startDevice(int serial_number);
        //使设备开始采集
        bool setStreamOn();
        //设置分辨率，支持1:1(最大1280*1024),1:2,2:1,2:2(最小640*512),默认1:1
        bool setResolution(int width_scale = 1, int height_scale = 1);
        //设置自动白平衡,0表示关闭，1表示开启
        bool setBalanceAuto(int value);
        //Color_Correct
        bool colorCorrect(bool value);
        // Set Gamma
        bool setGamma(bool set_status,double dGammaParam);
        // Set Contrast
        bool setContrast(bool set_status,int dContrastParam);
        // Set_Saturation
        bool setSaturation(bool set_status,int dSaturationParam);
    
    public:
        //手动设置曝光值,单位us,正常大小应在2000至8000
        bool setExposureTime(int ExposureTime);
        
        //设置曝光增益
        bool setGain(int value, int ExpGain);

        //手动设置白平衡,value表示平衡通道，value_number表示具体值,0、1、2对应B、G、R，value_number范围为10到80,10表示正常
        bool setBalance(int value, float value_number);
        
        //采集一次图像,更新时间戳
        bool updateTimestampOffset(std::chrono::_V2::steady_clock::time_point time_start);
        
        //读取相机时间戳
        int getTimestamp();

        //采集图像
        bool getImage(cv::Mat &Src, sensor_msgs::msg::Image& image_msg);
        
        //设备复位
        bool deviceReset();

    public:
        CameraParam cam_param_;
        rclcpp::Logger logger_;
        bool is_initialized_;
    };

}