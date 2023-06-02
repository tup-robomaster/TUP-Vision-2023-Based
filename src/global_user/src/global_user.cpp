/*
 * @Description: This is a ros_control learning project!
 * @Author: Liu Biao
 * @Date: 2022-09-05 14:01:05
 * @LastEditTime: 2023-04-14 03:18:54
 * @FilePath: /TUP-Vision-2023-Based/src/global_user/src/global_user.cpp
 */
#include "../include/global_user/global_user.hpp"

namespace global_user
{
    float calcTriangleArea(cv::Point2d pts[3])
    {
        /**
         * @brief caculate the areas of triangle
         * @param apexes
         * @return area
         */
        auto a = sqrt(pow((pts[0] - pts[1]).x, 2) + pow((pts[0] - pts[1]).y, 2));
        auto b = sqrt(pow((pts[1] - pts[2]).x, 2) + pow((pts[1] - pts[2]).y, 2));
        auto c = sqrt(pow((pts[2] - pts[0]).x, 2) + pow((pts[2] - pts[0]).y, 2));

        auto p = (a + b + c) / 2.f;
        return sqrt(p * (p - a) * (p - b) * (p - c));
    }

    float calcTetragonArea(cv::Point2d pts[4])
    {
        return calcTriangleArea(&pts[0]) + calcTriangleArea(&pts[1]);
    }

    Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
    {
        /**
         * @brief transform rotatedMatrix to euler angle
         * 
         */
        double sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        double x, y, z;
        
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }

        return {z, y, x};
    }

    Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2)
    {
        Eigen::Vector3d delta_euler;
        //将Roll表示范围由[-PI,PI]转换至[0，2PI]
        for (int i = 0; i < 3;i++)
        {
            if (euler2[i] <= 0)
                euler2[i] += CV_2PI;
            if (euler1[i] <= 0)
                euler1[i] += CV_2PI;
        }

        for (int i = 0; i < 3;i++)
        {
            if (euler2[i] > 0 && euler2[i] < (CV_PI / 2) && euler1[i] > (3 * CV_PI / 2))
                delta_euler[i] = CV_2PI + euler2[i] - euler1[i];
            else if (euler2[i] > (3 * CV_PI / 2) && euler1[i] > 0 && euler1[i] < (CV_PI / 2))
                delta_euler[i] = -CV_2PI + euler2[i] - euler1[i];
            else
                delta_euler[i] = euler2[i] - euler1[i];
        }

        return delta_euler;
    }
    
    Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta)
    {
        Eigen::Matrix3d R_x;
        Eigen::Matrix3d R_y;
        Eigen::Matrix3d R_z;
        // Calculate rotation about x axis
        R_x <<
            1,       0,              0,
            0,       cos(theta[2]),   -sin(theta[2]),
            0,       sin(theta[2]),   cos(theta[2]);
        // Calculate rotation about y axis
        R_y <<
            cos(theta[1]),    0,      sin(theta[1]),
            0,               1,      0,
            -sin(theta[1]),   0,      cos(theta[1]);
        // Calculate rotation about z axis
        R_z <<
            cos(theta[0]),    -sin(theta[0]),      0,
            sin(theta[0]),    cos(theta[0]),       0,
            0,               0,                  1;
        // Combined rotation matrix
        return R_z * R_y * R_x;
    }

    Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler)
    {
        Eigen::AngleAxisd rotVec;
        Eigen::AngleAxisd roll_angle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
        Eigen::AngleAxisd yaw_angle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd pitch_angle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));

        rotVec = roll_angle * yaw_angle * pitch_angle;
        return rotVec;
    }

    double rangedAngleRad(double &angle)
    {
        if (fabs(angle) >= CV_PI)
        {
            angle -= (angle / fabs(angle)) * CV_2PI;
            angle = rangedAngleRad(angle);
        }
        return angle;
    }
    
    std::vector<std::string> readLines(std::string file_path)
    {
        std::vector<std::string> lines;
        std::string line;
        std::ifstream file;
        file.open(file_path, std::ios::in);

        while(getline(file,line))
            lines.push_back(line);

        return lines;
    }
    
    std::string symbolicToReal(std::string path)
    {
        char path_tmp[1024];
        auto len = readlink(path.c_str(), path_tmp, 1024);
        //若路径错误返回空字符串
        if (len == -1)
            return NULL;
        std::string path_relative(path_tmp,0,len);

        return relativeToFull(path_relative,path);
    }

    std::string relativeToFull(std::string relative, std::string origin)
    {
        auto tree_relative = generatePathTree(relative);
        auto tree_origin = generatePathTree(origin);

        std::string result;

        //记数
        int relative_cnt = 0;
        for(auto dir : tree_relative)
        {
            if (dir == "..")
                relative_cnt++;
        }

        std::vector<std::string> tree_parent (tree_origin.begin(),tree_origin.end() - relative_cnt - 1);
        std::vector<std::string> tree_no_relative (tree_relative.begin() + relative_cnt,tree_relative.end());

        result = treeToPath(tree_parent) + treeToPath(tree_no_relative);

        return result;
    }

    std::string treeToPath(std::vector<std::string> &tree)
    {
        std::string result;
        for(auto node : tree)
        {
            result += "/" + node;
        }
        return result;
    }

    std::string getParent(std::string path)
    {
        int last_slash_idx = 0;
        std::string path_tmp = path;

        for(auto i = 0; i = path_tmp.find("/"), i != (int)(path_tmp.npos); path_tmp = path_tmp.substr(i + 1))
        {   
            last_slash_idx += i + 1;
        }
        
        if (last_slash_idx == 0)
            return path;
        else
            return path.substr(0,last_slash_idx - 1);
    }

    std::vector<std::string> generatePathTree(std::string path)
    {
        std::vector<std::string> tree;

        //若为相对路径，在头部插入“/”便于处理
        if (path[0] == '.')
            path.insert(0,"/");

        //使用逗号表达式控制循环
        for(auto i = 0; i = path.find("/"), i != (int)(path.npos); path = path.substr(i + 1))
        {   
            if(i == 0)
                continue;
            auto sub = path.substr(0, i);
            tree.push_back(sub);
        }

        tree.push_back(path);

        return tree;
    }

    float calcDistance(cv::Point2d& p1, cv::Point2d& p2)
    {
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }

    /**
     * @brief 创建图像数据共享内存空间
     * 
    */
    bool setSharedMemory(SharedMemoryParam& shared_memory_param, int id, int image_width, int image_height)
    {
        // 生成key
        shared_memory_param.key = ftok("./", id);
        
        // 返回内存id
        shared_memory_param.shared_memory_id = shmget(shared_memory_param.key, image_width * image_height * 3, IPC_CREAT | 0666 | IPC_EXCL);
        if(shared_memory_param.shared_memory_id == -1)
            return false;

        // 映射到内存地址
        shared_memory_param.shared_memory_ptr = shmat(shared_memory_param.shared_memory_id, 0, 0);
        if(shared_memory_param.shared_memory_ptr == (void*)-1)
            return false;
        
        return true;
    }

    bool destorySharedMemory(SharedMemoryParam& shared_memory_param)
    {
        //解除共享内存映射
        if(shared_memory_param.shared_memory_ptr)
        {
            if(shmdt(shared_memory_param.shared_memory_ptr) == -1)
            {
                // printf("Dissolution remapping failed...");
                return false;
            }
        }
        //销毁共享内存
        if(shmctl(shared_memory_param.shared_memory_id, IPC_RMID, NULL) == -1)
        {
            // printf("Destroy shared memory failed...");     
            return false;   
        }

        return true;
    }

    bool getSharedMemory(SharedMemoryParam& shared_memory_param, int id)
    {
        shared_memory_param.key = ftok("./", id);
        // 获取共享内存id
        shared_memory_param.shared_memory_id = shmget(shared_memory_param.key, 0, 0);
        if(shared_memory_param.shared_memory_id == -1)
        {
            // RCLCPP_ERROR(this->get_logger(), "Get shared memory id failed...");
            return false;
        }

        // 映射共享内存，得到虚拟地址
        shared_memory_param.shared_memory_ptr = shmat(shared_memory_param.shared_memory_id, 0, 0);
        if(shared_memory_param.shared_memory_ptr == (void*)-1)
        {
            // RCLCPP_ERROR(this->get_logger(), "Remapping shared memory failed...");
            return false;
        }
        return true;
    }

    bool autoLabel(bool& is_init, cv::Mat &img, ofstream &file, string &path_prefix, int64_t &timestamp, int &id, int &color, vector<cv::Point2d> &apex2d, cv::Point2i &roi_offset, cv::Size2i &input_size)
    {
        if(!is_init)
        {
            std::string img_name = path_prefix + to_string(timestamp) + ".jpg";
            cv::imwrite(img_name, img);
            is_init = true;
        }
        std::string label_name = path_prefix + to_string(timestamp) + ".txt";
        std::string content;

        int cls = 0;
        if(id == 7)
            cls = 9 * color - 1;
        if(id != 7)
            cls = id + color * 9;
        
        content.append(to_string(cls) + " ");
        for(auto apex : apex2d)
        {
            content.append(to_string((apex.x - roi_offset.x) / input_size.width));
            content.append(" ");
            content.append(to_string((apex.y - roi_offset.y) / input_size.height));
            content.append(" ");
        }
        content.pop_back();
        content.append("\n");
        file.open(label_name, std::ofstream::app);
        file << content;
        file.close();
        usleep(5000);
        return true;
    }

    bool isPnpSolverValidation(Eigen::Vector3d& point3d)
    {
        if (isinf(point3d[0] || isinf(point3d[1]) || isinf(point3d[2])))
        {
            return false;
        }
        else if (isnan(point3d[0]) || isnan(point3d[1] || isnan(point3d[2])))
        {
            return false;
        }
        else if (point3d.norm() >= 10.0)
        {
            return false;
        }
        return true;
    }

    bool isAngleSolverValidataion(Eigen::Vector2d& angle2d)
    {
        if (isinf(angle2d[0] || isinf(angle2d[1])))
        {
            return false;
        }
        else if (isnan(angle2d[0] || isnan(angle2d[1])))
        {
            return false;
        }
        else if (abs(angle2d[0]) >= 90.0 || abs(angle2d[1]) >= 90.0)
        {
            return false;
        }
        return true;
    }

    void drawAimCrossCurve(cv::Mat& src)
    {
        line(src, cv::Point2d(src.size().width / 2, 0), cv::Point2d(src.size().width / 2, src.size().height), {0,255,0}, 1);
        line(src, cv::Point2d(0, src.size().height / 2), cv::Point2d(src.size().width, src.size().height / 2), {0,255,0}, 1);
    }

    //新息序列不等式
    bool checkDivergence(const MatrixXd& statePre, const MatrixXd& stateCovPre, const MatrixXd& H, const MatrixXd& R, const VectorXd& measurement)
    {
        // 滤波发散判据（传统基于单步量测的新息序列不等式）
        VectorXd innovationCovPre(1, 1);
        innovationCovPre << H * stateCovPre * H.transpose() + R; 
        MatrixXd innovation(1, 1);
        innovation << measurement - H * statePre;
        MatrixXd innovationSquare = innovation * innovation.transpose();
        double traceInnovationCovPre = innovationCovPre.trace();

        return (innovationSquare(0, 0) > 100 * traceInnovationCovPre);
    }

    // bool checkDivergence(const MatrixXd& F, const MatrixXd& P, const MatrixXd& H, const MatrixXd& R)
    // {
    //     int n = F.rows(); // 状态向量维度
    //     int m = H.rows(); // 观测向量维度

    //     // 计算新息序列不等式右侧值
    //     MatrixXd Q = F * P * F.transpose();
    //     MatrixXd S = H * Q * H.transpose() + R;
    //     MatrixXd K = Q * H.transpose() * S.inverse();
    //     MatrixXd I = MatrixXd::Identity(n, n);
    //     MatrixXd P_new = (I - K * H) * Q;

    //     double rhs = 0;
    //     for (int i = 0; i < m; i++) 
    //     {
    //         rhs += log(S(i, i));
    //     }
    //     rhs += n * log(2 * M_PI) + log(P_new.determinant());

    //     // 计算新息序列不等式左侧值
    //     MatrixXd x = VectorXd::Zero(n);
    //     MatrixXd z = VectorXd::Zero(m);
    //     MatrixXd v = z - H * x;
    //     MatrixXd w = MatrixXd::Zero(n, m);
    //     MatrixXd S_sqrt = S.llt().matrixL();
    //     double lhs = 0;
    //     for (int i = 0; i < m; i++) 
    //     {
    //         double d = v(i, 0) / S_sqrt(i, i);
    //         lhs += d * d;
    //         for (int j = 0; j < n; j++) 
    //         {
    //             w(j, i) = K(j, i) / S_sqrt(i, i);
    //         }
    //     }
    //     lhs += (x.transpose() * P_new.inverse() * x)(0, 0);
    //     lhs += (w.transpose() * w).sum();

    //     return lhs > rhs;
    // }

    // //残差方差检测法
    // bool checkDivergence(double residual, double threshold, vector<double>& variances, int window_size)
    // {
    //     variances.push_back(residual * residual);
    //     if (variances.size() > window_size) 
    //     {
    //         variances.erase(variances.begin());
    //     }
    //     double var_sum = 0;
    //     for (double v : variances) 
    //     {
    //         var_sum += v;
    //     }
    //     double mean_var = var_sum / variances.size();
    //     double var_diff = 0;
    //     for (double v : variances) 
    //     {
    //         var_diff += (v - mean_var) * (v - mean_var);
    //     }
    //     double std_var = sqrt(var_diff / (variances.size() - 1));
    //     return std_var > threshold;
    // }

    // //一致性检测法
    // bool checkDivergence(const MatrixXd& residual, const MatrixXd& S, double threshold)
    // {
    //     int n = residual.rows();
    //     MatrixXd R = residual * residual.transpose() / (n - 1);
    //     MatrixXd diff = R - S;
    //     double norm_diff = diff.norm();
    //     return norm_diff > threshold;
    // }
} //global_user
