/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 03:24:50
 * @LastEditTime: 2023-06-02 21:56:20
 * @FilePath: /TUP-Vision-2023-Based/src/global_user/include/global_user/global_user.hpp
 */
#ifndef GLOBAL_USER_HPP_
#define GLOBAL_USER_HPP_

#include <string>
#include <vector>
#include <thread>
#include <memory>
#include <iterator>
#include <unistd.h>
#include <future>
#include <fstream>
#include <yaml-cpp/yaml.h>

//opencv
#include <opencv2/opencv.hpp>

//eigen
#include <Eigen/Dense>
#include <Eigen/Core>

//linux
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

// daheng
#define DAHENG_IMAGE_WIDTH 1280 
#define DAHENG_IMAGE_HEIGHT 1024
// hik
#define HIK_IMAGE_WIDTH 1440     
#define HIK_IMAGE_HEIGHT 1080
// usb
#define USB_IMAGE_WIDTH 640     
#define USB_IMAGE_HEIGHT 480
// mvs
#define MVS_IMAGE_WIDTH 1280     
#define MVS_IMAGE_HEIGHT 1024

using namespace std;
using namespace Eigen;
namespace global_user
{   
    /**
     * @brief Global variables and funcs.
     * 
     */

    struct CameraParam
    {
        int fps;
        int cam_id;
        int image_width;
        int image_height;
        int width_scale;
        int height_scale;
        int exposure_time;
        double exposure_gain;
        bool auto_balance;
        double exposure_gain_b;
        double exposure_gain_g;
        double exposure_gain_r;
        double balance_b;
        double balance_g;
        double balance_r;
        bool using_video;
        string video_path;
        string config_path = "src/camera_driver/config/daheng_cam_param.ini";

        CameraParam()
        {
            cam_id = 1;
            image_width = 1280;
            image_height = 1024;
            width_scale = 1;
            height_scale = 1;
            exposure_time = 3000;
            exposure_gain = 14;
            auto_balance = false;
            exposure_gain_b = 0;
            exposure_gain_g = 0;
            exposure_gain_r = 0;
            balance_b = 1.56;
            balance_g = 1.0;
            balance_r = 1.548;
            fps = 30;
            using_video = false;
            video_path = "\0";
        }
    };

    struct ImageSize
    {
        int width;
        int height;

        ImageSize()
        {
            this->width = DAHENG_IMAGE_WIDTH;
            this->height = DAHENG_IMAGE_HEIGHT;
        }
    };

    class ImageInfo
    {
    public:
        std::map<int, std::string> camera_topic_map;
        std::map<int, ImageSize> image_size_map;

        ImageInfo()
        {
            camera_topic_map = 
            {
                {0, "/daheng_img"},
                {1, "/hik_img"},
                {2, "/mvs_img"},
                {3, "/usb_img"}
            };

            image_size_map[0].width = DAHENG_IMAGE_WIDTH;
            image_size_map[0].height = DAHENG_IMAGE_HEIGHT;
            image_size_map[1].width = HIK_IMAGE_WIDTH;
            image_size_map[1].height = HIK_IMAGE_HEIGHT;
            image_size_map[2].width = MVS_IMAGE_WIDTH;
            image_size_map[2].height = MVS_IMAGE_HEIGHT;
            image_size_map[3].width = USB_IMAGE_WIDTH;
            image_size_map[3].height = USB_IMAGE_HEIGHT;
        }
    };

    enum CameraType
    {
        DaHeng,
        HikRobot,
        MVSCam,
        USBCam,
    };

    enum EnemyColor 
    {
        BLUE,
        RED,
        GRAY,
        PURPLE
    };

    enum TargetType 
    {  
        SMALL, 
        BIG, 
        BUFF
    };

    enum SpinHeading
    {
        UNKNOWN,
        CLOCKWISE, 
        COUNTER_CLOCKWISE
    };

    /**
     * @brief 模式选择（自瞄跟随，自瞄预测，自瞄吊射，小符，大符，前哨站旋转装甲吊射，哨兵自瞄）
     * 
     */
    enum MODE
    {
        AUTOAIM_TRACKING,
        AUTOAIM_NORMAL,
        AUTOAIM_SLING,
        SMALL_BUFF,
        BIG_BUFF,
        OUTPOST_ROTATION_MODE,
        SENTRY_NORMAL
    };

    struct TaskData
    {
        int mode;
        cv::Mat img;
        Eigen::Quaterniond quat;
        int64_t timestamp; 
        
        TaskData()
        {
            mode = 1;
            timestamp = 0;
        }
    };

    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    struct ObjectBase
    {
        int id;
        int color;
        double conf;
        std::string key;
        Eigen::Vector3d armor3d_cam;
        Eigen::Vector3d armor3d_world;
        Eigen::Vector3d euler;
        Eigen::Matrix3d rmat;
    };
    
    struct Object
    {
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    struct VideoRecordParam
    {
        std::future<void> writer;
        cv::VideoWriter video_recorder;
        std::string save_path;
        bool is_initialized;
        bool is_first_loop;
        int frame_cnt;
        int image_width;
        int image_height;

        VideoRecordParam()
        {
            save_path = "src/camera_driver/video/";
            is_initialized = false;
            is_first_loop = true;
            frame_cnt = 0;
            image_width = 1280; //FIXME:根据相机类型设置
            image_height = 1024;
        }
    };

    struct SharedMemoryParam
    {
        key_t key;               //生成一个key
        int shared_memory_id;    //共享内存的id
        void* shared_memory_ptr; //映射共享内存，得到虚拟地址
        SharedMemoryParam()
        {
            shared_memory_ptr = nullptr;
        }
    };

    template<typename T>
    bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
    {
        int cnt = 0;
        for(int row = 0;row < matrix.rows();row++)
        {
            for(int col = 0;col < matrix.cols();col++)
            {
                matrix(row,col) = vector[cnt];
                cnt++;
            }
        }
        return true;
    }

    float calcTriangleArea(cv::Point2d pts[3]);
    float calcTetragonArea(cv::Point2d pts[4]);
    double rangedAngleRad(double &angle);

    std::string symbolicToReal(std::string path);
    std::string relativeToFull(std::string relative, std::string src);
    std::string treeToPath(std::vector<std::string> &tree);
    std::string getParent(std::string path);

    std::vector<std::string> readLines(std::string file_path);
    std::vector<std::string> generatePathTree(std::string path);

    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
    Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2);
    Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);
    float calcDistance(cv::Point2d& p1, cv::Point2d& p2);

    bool setSharedMemory(SharedMemoryParam& shared_memory_param, int id, int image_width = 1280, int image_height = 1024);
    bool getSharedMemory(SharedMemoryParam& shared_memory_param, int id);
    bool destorySharedMemory(SharedMemoryParam& shared_memory_param);
    bool autoLabel(bool& is_init, cv::Mat &img, ofstream &file, string &path_name, int64_t &timestamp, int &id, int &color, vector<cv::Point2d> &apex2d, cv::Point2i &roi_offset, cv::Size2i &input_size);

    bool isPnpSolverValidation(Eigen::Vector3d& point3d);
    bool isAngleSolverValidataion(Eigen::Vector2d& angle2d);
    void drawAimCrossCurve(cv::Mat& src);

    // bool checkDivergence(const MatrixXd& residual, const MatrixXd& S, double threshold);
    // bool checkDivergence(double residual, double threshold, vector<double>& variances, int window_size);
    // bool checkDivergence(const MatrixXd& F, const MatrixXd& P, const MatrixXd& H, const MatrixXd& R);
    bool checkDivergence(const MatrixXd& statePre, const MatrixXd& stateCovPre, const MatrixXd& H, const MatrixXd& R, const VectorXd& measurement);
} // namespace global_user

#endif
