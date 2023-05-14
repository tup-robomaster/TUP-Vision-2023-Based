#ifndef __INFERENCE_H
#define __INFERENCE_H

#include "public.h"

namespace TRTInferV1
{
    struct GridAndStride
    {
        int grid0;
        int grid1;
        int stride;
    };

    struct Object
    {
        cv::Rect_<float> rect;
        int cls;
        int color;
        float prob;
        std::vector<cv::Point2f> pts;
    };

    struct DetectObject : Object
    {
        int area;
        cv::Point2f apex[32];
    };

    /**
     * @brief TRT推理
     * 高性能TRT YOLOX推理模块
     */
    class TRTInfer
    {
    private:
        const char *INPUT_BLOB_NAME = "images";
        const char *OUTPUT_BLOB_NAME = "output";
        int num_apex = -1;
        int num_classes = -1;
        int num_colors = -1;
        int topK = -1;

    private:
        Logger gLogger;
        IRuntime *runtime;
        ICudaEngine *engine;
        IExecutionContext *context;
        void *buffers[2];
        int output_size = 1;
        int inputIndex = -1;
        int outputIndex = -1;
        Dims input_dims;
        Dims output_dims;
        uint8_t *img_host = nullptr;
        uint8_t *img_device = nullptr;
        float *output;

    private:
        int inter_frame_compensation = 0;

    private:
        inline int argmax(const float *ptr, int len);
        void qsort_descent_inplace(std::vector<DetectObject> &objects, int left, int right);
        void qsort_descent_inplace(std::vector<DetectObject> &objects);
        inline float intersection_area(const DetectObject &a, const DetectObject &b);
        void nms_sorted_bboxes(std::vector<DetectObject> &objects, std::vector<int> &picked, float nms_threshold);
        float calcTriangleArea(cv::Point2f pts[3]);
        float calcPolygonArea(cv::Point2f pts[32]);
        void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int> &strides, std::vector<GridAndStride> &grid_strides);
        void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float *feat_ptr,
                                    Eigen::Matrix<float, 3, 3> &transform_matrix, float prob_threshold,
                                    std::vector<DetectObject> &objects);
        void decodeOutputs(const float *prob, std::vector<DetectObject> &objects, Eigen::Matrix<float, 3, 3> &transform_matrix, float confidence_threshold, float nms_threshold);
        void postprocess(std::vector<std::vector<DetectObject>> &batch_res, std::vector<cv::Mat> &frames, float &confidence_threshold, float &nms_threshold);

    public:
        /**
         * @brief 构造函数
         * @param device
         * 使用的GPU索引
         */
        TRTInfer(const int device);
        ~TRTInfer();

        /**
         * @brief 初始化TRT模型
         * @param engine_file_path
         * engine路径
         * @param batch_size
         * 推理时使用的batch_size,输入图片数量不可大于此设置值，此设定值不可大于构建引擎时应用的maxBatchSize，最佳设定值为maxBatchSize/2
         * @param num_apex
         * num_apex设定值，角点数量，最大值32
         * @param num_classes
         * num_classes设定值，类别数量
         * @param num_colors
         * num_colors设定值，颜色数量
         * @param topK
         * topK设定值
         */
        bool initMoudle(const std::string engine_file_path, const int batch_size, const int num_apex, const int num_classes, const int num_colors, const int topK);
        /**
         * @brief 反初始化TRT模型，释放显存
         */
        void unInitMoudle();
        /**
         * @brief 保存engine文件至指定路径
         * @param data
         * 由 createEngine() 构建的序列化模型
         * @param engine_file_path
         * engine文件保存路径
         */
        void saveEngineFile(IHostMemory *data, const std::string engine_file_path);
        /**
         * @brief 执行推理
         * @param frames
         * 需要推理的图像序列，图像数量决定推理时batch_size，不可大于初始化模型时指定的batch_size
         * @param confidence_threshold
         * 置信度阈值
         * @param nms_threshold
         * 非极大值抑制阈值
         */
        std::vector<std::vector<DetectObject>> doInference(std::vector<cv::Mat> &frames, float confidence_threshold, float nms_threshold);
        /**
         * @brief 计算帧内时间补偿
         * @param limited_fps
         * 目标FPS设定值，将根据此设定值计算时间补偿，配合doInferenceLimitFPS使用
         */
        void calculate_inter_frame_compensation(const int limited_fps);
        /**
         * @brief 执行推理(帧限制)
         * @param frames
         * 需要推理的图像序列，图像数量决定推理时batch_size，不可大于初始化模型时指定的batch_size
         * @param confidence_threshold
         * 置信度阈值
         * @param nms_threshold
         * 非极大值抑制阈值
         * @param limited_fps
         * 目标FPS设定值，推理过程的帧数将尝试限定在目标值附近，若运行帧率大于设定值，实际帧数将会接近并稳定下来，指定帧数越高，实际帧数偏差越大，帧数稳定性为+1~-2FPS
         */
        std::vector<std::vector<DetectObject>> doInferenceLimitFPS(std::vector<cv::Mat> &frames, float confidence_threshold, float nms_threshold, const int limited_fps);
        /**
         * @brief 构建engine
         * @param onnx_path
         * 用于构建engine的onnx文件路径
         * @param maxBatchSize
         * 最大batch_size设定值
         * @param input_h
         * Tensor输入图像尺寸 h
         * @param input_w
         * Tensor输入图像尺寸 w
         */
        IHostMemory *createEngine(const std::string onnx_path, unsigned int maxBatchSize, int input_h, int input_w);
        /**
         * @brief 获取输入尺寸
         */
        int getInputSizeH();
        /**
         * @brief 获取输入尺寸
         */
        int getInputSizeW();
    };
}

#endif // __INFERENCE_H