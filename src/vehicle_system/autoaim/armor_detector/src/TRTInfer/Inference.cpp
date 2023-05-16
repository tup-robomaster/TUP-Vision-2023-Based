#include "../../include/TRTInfer/Inference.h"
namespace TRTInferV1
{
    static constexpr float MERGE_CONF_ERROR = 0.15;
    static constexpr float MERGE_MIN_IOU = 0.9;

    inline int TRTInfer::argmax(const float *ptr, int len)
    {
        int max_arg = 0;
        for (int i = 1; i < len; ++i)
        {
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        return max_arg;
    }

    void TRTInfer::qsort_descent_inplace(std::vector<DetectObject> &objects, int left, int right)
    {
        int i = left;
        int j = right;
        float p = objects[(left + right) / 2].prob;

        while (i <= j)
        {
            while (objects[i].prob > p)
                i++;

            while (objects[j].prob < p)
                j--;

            if (i <= j)
            {
                // swap
                std::swap(objects[i], objects[j]);
                i++;
                j--;
            }
        }
        if (left < j)
            this->qsort_descent_inplace(objects, left, j);
        if (i < right)
            this->qsort_descent_inplace(objects, i, right);
    }

    void TRTInfer::qsort_descent_inplace(std::vector<DetectObject> &objects)
    {
        if (objects.empty())
            return;
        this->qsort_descent_inplace(objects, 0, objects.size() - 1);
    }

    inline float TRTInfer::intersection_area(const DetectObject &a, const DetectObject &b)
    {
        return (a.rect & b.rect).area();
    }

    void TRTInfer::nms_sorted_bboxes(std::vector<DetectObject> &objects, std::vector<int> &picked, float nms_threshold)
    {
        picked.clear();
        const int n = objects.size();

        std::vector<float> areas(n);
        for (int i = 0; i < n; i++)
        {
            areas[i] = objects[i].rect.area();
        }

        for (int i = 0; i < n; i++)
        {
            DetectObject &a = objects[i];
            int keep = 1;
            for (int j = 0; j < (int)picked.size(); j++)
            {
                DetectObject &b = objects[picked[j]];
                // intersection over union
                float inter_area = intersection_area(a, b);
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                float iou = inter_area / union_area;
                if (iou > nms_threshold || isnan(iou))
                {
                    keep = 0;
                    // Stored for Merge
                    if (iou > MERGE_MIN_IOU && abs(a.prob - b.prob) < MERGE_CONF_ERROR && a.cls == b.cls && a.color == b.color)
                    {
                        for (int i = 0; i < this->num_apex; i++)
                        {
                            b.pts.emplace_back(a.apex[i]);
                        }
                    }
                }
            }
            if (keep)
                picked.emplace_back(i);
        }
    }

    float TRTInfer::calcTriangleArea(cv::Point2f pts[3])
    {
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));
        auto p = (a + b + c) / 2.f;
        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    float TRTInfer::calcPolygonArea(cv::Point2f pts[32])
    {
        int area = 0;
        if (this->num_apex > 2)
        {
            for (int i = 0; i < this->num_apex - 3; ++i)
            {
                area += calcTriangleArea(&pts[i]);
            }
        }
        else
        {
            area = abs(pts[1].x - pts[0].x) * abs(pts[1].y - pts[0].y);
        }
        return area;
    }

    void TRTInfer::generate_grids_and_stride(const int target_w, const int target_h, std::vector<int> &strides, std::vector<GridAndStride> &grid_strides)
    {
        for (auto stride : strides)
        {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;

            for (int g1 = 0; g1 < num_grid_h; g1++)
            {
                for (int g0 = 0; g0 < num_grid_w; g0++)
                {
                    GridAndStride grid_stride = {g0, g1, stride};
                    grid_strides.emplace_back(grid_stride);
                }
            }
        }
    }

    void TRTInfer::generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float *feat_ptr,
                                          Eigen::Matrix<float, 3, 3> &transform_matrix, float prob_threshold,
                                          std::vector<DetectObject> &objects)
    {

        const int num_anchors = grid_strides.size();
        // Travel all the anchors
        for (int anchor_idx = 0; anchor_idx < num_anchors; ++anchor_idx)
        {
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            const int stride = grid_strides[anchor_idx].stride;
            const int num_apex_expend = 2 * this->num_apex;
            const int basic_pos = anchor_idx * (num_apex_expend + 1 + this->num_colors + this->num_classes);

            float x_box[32];
            float y_box[32];

            for (int apex_id = 0; apex_id < num_apex_expend; apex_id = apex_id + 2)
            {
                x_box[apex_id / 2] = (feat_ptr[basic_pos + apex_id] + grid0) * stride;
                y_box[apex_id / 2] = (feat_ptr[basic_pos + apex_id + 1] + grid1) * stride;
            }

            int box_color = argmax(feat_ptr + basic_pos + (num_apex_expend + 1), this->num_colors);
            int box_class = argmax(feat_ptr + basic_pos + (num_apex_expend + 1) + this->num_colors, this->num_classes);
            float box_objectness = (feat_ptr[basic_pos + num_apex_expend]);
            float box_prob = box_objectness;

            if (box_prob >= prob_threshold)
            {
                DetectObject obj;

                Eigen::MatrixXf apex_norm(3, this->num_apex);
                Eigen::MatrixXf apex_dst(3, this->num_apex);

                for (int apex_id = 0; apex_id < this->num_apex; ++apex_id)
                {
                    apex_norm(0, apex_id) = x_box[apex_id];
                    apex_norm(1, apex_id) = y_box[apex_id];
                    apex_norm(2, apex_id) = 1;
                }

                apex_dst = transform_matrix * apex_norm;

                for (int i = 0; i < 4; i++)
                {
                    obj.apex[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
                    obj.pts.emplace_back(obj.apex[i]);
                }

                std::vector<cv::Point2f> tmp(obj.apex, obj.apex + 4);
                obj.rect = cv::boundingRect(tmp);
                obj.cls = box_class;
                obj.color = box_color;
                obj.prob = box_prob;

                objects.emplace_back(obj);
            }
        } // point anchor loop
    }

    void TRTInfer::decodeOutputs(const float *prob, std::vector<DetectObject> &objects, Eigen::Matrix<float, 3, 3> &transform_matrix, float confidence_threshold, float nms_threshold)
    {
        std::vector<DetectObject> proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        this->generate_grids_and_stride(this->input_dims.d[3], this->input_dims.d[2], strides, grid_strides);
        this->generateYoloxProposals(grid_strides, prob, transform_matrix, confidence_threshold, proposals);
        this->qsort_descent_inplace(proposals);
        if (int(proposals.size()) >= this->topK)
            proposals.resize(this->topK);
        std::vector<int> picked;
        this->nms_sorted_bboxes(proposals, picked, nms_threshold);
        int count = picked.size();
        objects.resize(count);
        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];
        }
    }

    void TRTInfer::postprocess(std::vector<std::vector<DetectObject>> &batch_res, std::vector<cv::Mat> &frames, float &confidence_threshold, float &nms_threshold)
    {
        for (int b = 0; b < int(frames.size()); ++b)
        {
            auto &res = batch_res[b];
            float r = std::min(this->input_dims.d[3] / (frames[b].cols * 1.0), this->input_dims.d[2] / (frames[b].rows * 1.0));
            int unpad_w = r * frames[b].cols;
            int unpad_h = r * frames[b].rows;

            int dw = this->input_dims.d[3] - unpad_w;
            int dh = this->input_dims.d[2] - unpad_h;

            dw /= 2;
            dh /= 2;

            Eigen::Matrix3f transform_matrix;
            transform_matrix << 1.0 / r, 0, -dw / r,
                0, 1.0 / r, -dh / r,
                0, 0, 1;
            this->decodeOutputs(&this->output[b * this->output_size], res, transform_matrix, confidence_threshold, nms_threshold);
            for (auto object = res.begin(); object != res.end(); ++object)
            {
                // 对候选框预测角点进行平均,降低误差
                if ((*object).pts.size() >= 8)
                {
                    auto N = (*object).pts.size();
                    cv::Point2f pts_final[this->num_apex];
                    for (int i = 0; i < (int)N; ++i)
                    {
                        pts_final[i % this->num_apex] += (*object).pts[i];
                    }

                    for (int i = 0; i < this->num_apex; ++i)
                    {
                        pts_final[i].x = pts_final[i].x / (N / this->num_apex);
                        pts_final[i].y = pts_final[i].y / (N / this->num_apex);
                        (*object).apex[i] = pts_final[i];
                    }
                }

                (*object).area = (int)(this->calcPolygonArea((*object).apex));
            }
        }
    }

    TRTInfer::TRTInfer(const int device)
    {
        cudaSetDevice(device);
    }

    TRTInfer::~TRTInfer()
    {
    }

    bool TRTInfer::initMoudle(const std::string engine_file_path, const int batch_size, const int num_apex, const int num_classes, const int num_colors, const int topK)
    {
        assert(num_apex <= 32 && num_apex >= 2);
        assert(batch_size > 0 && num_classes > 0 && num_colors > 0 && topK > 0);
        this->num_apex = num_apex;
        this->num_classes = num_classes;
        this->num_colors = num_colors;
        this->topK = topK;
        char *trtModelStream{nullptr};
        size_t size{0};
        std::ifstream file(engine_file_path, std::ios::binary);
        if (file.good())
        {
            file.seekg(0, file.end);
            size = file.tellg();
            file.seekg(0, file.beg);
            trtModelStream = new char[size];
            assert(trtModelStream);
            file.read(trtModelStream, size);
            file.close();
        }
        else
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Engine bad file");
            return false;
        }
        this->runtime = createInferRuntime(this->gLogger);
        assert(runtime != nullptr);
        this->engine = this->runtime->deserializeCudaEngine(trtModelStream, size);
        assert(this->engine != nullptr);
        this->context = this->engine->createExecutionContext();
        assert(context != nullptr);
        delete trtModelStream;
        this->input_dims = this->engine->getTensorShape(INPUT_BLOB_NAME);
        this->input_dims.d[0] = batch_size;
        this->output_dims = this->engine->getTensorShape(OUTPUT_BLOB_NAME);
        this->context->setInputShape(INPUT_BLOB_NAME, input_dims);
        this->output_size = output_dims.d[1] * output_dims.d[2];
        int IOtensorsNum = engine->getNbIOTensors();
        assert(IOtensorsNum == 2);
        for (int i = 0; i < IOtensorsNum; ++i)
        {
            if (strcmp(this->engine->getIOTensorName(i), INPUT_BLOB_NAME))
            {
                this->inputIndex = i;
                assert(this->engine->getTensorDataType(INPUT_BLOB_NAME) == nvinfer1::DataType::kFLOAT);
            }
            else if (strcmp(this->engine->getIOTensorName(i), OUTPUT_BLOB_NAME))
            {
                this->outputIndex = i;
                assert(this->engine->getTensorDataType(OUTPUT_BLOB_NAME) == nvinfer1::DataType::kFLOAT);
            }
        }
        if (this->inputIndex == -1 || this->outputIndex == -1)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "Uncorrect Input/Output tensor name");
            delete context;
            delete engine;
            return false;
        }
        CHECK(cudaMalloc(&buffers[inputIndex], batch_size * this->input_dims.d[1] * this->input_dims.d[2] * this->input_dims.d[3] * sizeof(float)));
        CHECK(cudaMalloc(&buffers[outputIndex], batch_size * this->output_size * sizeof(float)));
        CHECK(cudaMallocHost((void **)&this->img_host, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        CHECK(cudaMalloc((void **)&this->img_device, MAX_IMAGE_INPUT_SIZE_THRESH * 3 * sizeof(float)));
        this->output = (float *)malloc(batch_size * this->output_size * sizeof(float));
        return true;
    }

    void TRTInfer::unInitMoudle()
    {
        delete this->engine;
        delete this->context;
        delete this->runtime;
        CHECK(cudaFree(img_device));
        CHECK(cudaFreeHost(img_host));
        CHECK(cudaFree(this->buffers[this->inputIndex]));
        CHECK(cudaFree(this->buffers[this->outputIndex]));
    }

    void TRTInfer::saveEngineFile(IHostMemory *data, const std::string engine_file_path)
    {
        std::string serialize_str;
        std::ofstream serialize_output_stream;
        serialize_str.resize(data->size());
        memcpy((void *)serialize_str.data(), data->data(), data->size());
        serialize_output_stream.open(engine_file_path);
        serialize_output_stream << serialize_str;
        serialize_output_stream.close();
    }

    std::vector<std::vector<DetectObject>> TRTInfer::doInference(std::vector<cv::Mat> &frames, float confidence_threshold, float nms_threshold)
    {
        if (frames.size() == 0 || int(frames.size()) > this->input_dims.d[0])
        {
            this->gLogger.log(ILogger::Severity::kWARNING, "Invalid frames size");
            return {};
        }
        std::vector<std::vector<DetectObject>> batch_res(frames.size());
        cudaStream_t stream = nullptr;
        CHECK(cudaStreamCreate(&stream));
        float *buffer_idx = (float *)buffers[this->inputIndex];
        for (size_t b = 0; b < frames.size(); ++b)
        {
            cv::Mat &img = frames[b];
            if (img.empty())
                continue;
            size_t size_image = img.cols * img.rows * 3;
            size_t size_image_dst = this->input_dims.d[3] * this->input_dims.d[2] * 3;
            memcpy(img_host, img.data, size_image);
            CHECK(cudaMemcpyAsync(img_device, img_host, size_image, cudaMemcpyHostToDevice, stream));
            preprocess_kernel_img(img_device, img.cols, img.rows, buffer_idx, this->input_dims.d[3], this->input_dims.d[2], stream);
            buffer_idx += size_image_dst;
        }
        this->context->setOptimizationProfileAsync(0, stream);
        this->context->setTensorAddress(INPUT_BLOB_NAME, this->buffers[this->inputIndex]);
        this->context->setTensorAddress(OUTPUT_BLOB_NAME, this->buffers[this->outputIndex]);
        bool success = this->context->enqueueV3(stream);
        if (!success)
        {
            this->gLogger.log(ILogger::Severity::kERROR, "DoInference failed");
            CHECK(cudaStreamDestroy(stream));
            return {};
        }
        CHECK(cudaMemcpyAsync(this->output, buffers[this->outputIndex], frames.size() * this->output_size * sizeof(float), cudaMemcpyDeviceToHost, stream));
        CHECK(cudaStreamSynchronize(stream));
        CHECK(cudaStreamDestroy(stream));

        this->postprocess(batch_res, frames, confidence_threshold, nms_threshold);

        return batch_res;
    }

    void TRTInfer::calculate_inter_frame_compensation(const int limited_fps)
    {
        std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
        double limit_work_time = 1000000L / limited_fps;
        std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(limit_work_time));
        std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();
        this->inter_frame_compensation = std::chrono::duration<double, std::micro>(end_t - start_t).count() - limit_work_time;
    }

    std::vector<std::vector<DetectObject>> TRTInfer::doInferenceLimitFPS(std::vector<cv::Mat> &frames, float confidence_threshold, float nms_threshold, const int limited_fps)
    {
        double limit_work_time = 1000000L / limited_fps;
        std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
        std::vector<std::vector<DetectObject>> result = this->doInference(frames, confidence_threshold, nms_threshold);
        std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();
        std::chrono::duration<double, std::micro> work_time = end_t - start_t;
        if (work_time.count() < limit_work_time)
        {
            std::this_thread::sleep_for(std::chrono::duration<double, std::micro>(limit_work_time - work_time.count() - this->inter_frame_compensation));
        }
        return result;
    }

    IHostMemory *TRTInfer::createEngine(const std::string onnx_path, unsigned int maxBatchSize, int input_h, int input_w)
    {
        IBuilder *builder = createInferBuilder(this->gLogger);
        uint32_t flag = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
        INetworkDefinition *network = builder->createNetworkV2(flag);

        IParser *parser = createParser(*network, gLogger);
        if (!parser->parseFromFile(onnx_path.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING)))
        {
            this->gLogger.log(ILogger::Severity::kINTERNAL_ERROR, "failed parse the onnx mode");
        }
        // 解析有错误将返回
        for (int32_t i = 0; i < parser->getNbErrors(); ++i)
        {
            std::cout << parser->getError(i)->desc() << std::endl;
        }
        this->gLogger.log(ILogger::Severity::kINFO, "successfully parse the onnx mode");
        IBuilderConfig *config = builder->createBuilderConfig();
        IOptimizationProfile *profile = builder->createOptimizationProfile();

        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kMIN, Dims4(1, 3, input_h, input_w));
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kOPT, Dims4(int(ceil(maxBatchSize / 2.)), 3, input_h, input_w));
        profile->setDimensions(INPUT_BLOB_NAME, OptProfileSelector::kMAX, Dims4(maxBatchSize, 3, input_h, input_w));

        // Build engine
        config->addOptimizationProfile(profile);
        config->setMemoryPoolLimit(MemoryPoolType::kWORKSPACE, 10 << 20);
        config->setFlag(nvinfer1::BuilderFlag::kFP16); // 设置精度计算
        // config->setFlag(nvinfer1::BuilderFlag::kINT8);
        IHostMemory *serializedModel = builder->buildSerializedNetwork(*network, *config);
        this->gLogger.log(ILogger::Severity::kINFO, "successfully convert onnx to engine");

        // 销毁
        delete network;
        delete parser;
        delete config;
        delete builder;

        return serializedModel;
    }

    int TRTInfer::getInputSizeH()
    {
        return this->input_dims.d[2];
    }
    int TRTInfer::getInputSizeW()
    {
        return this->input_dims.d[3];
    }
}