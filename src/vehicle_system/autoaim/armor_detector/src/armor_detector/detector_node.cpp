/*
 * @Description: This is a ros-based project!
 * @Author: Liu Biao
 * @Date: 2022-10-14 17:11:03
 * @LastEditTime: 2022-10-23 19:23:23
 * @FilePath: /tup_2023-10-16/src/vehicle_system/autoaim/armor_detector/src/detector_node.cpp
 */
#include "../../include/armor_detector/detector_node.hpp"

using std::placeholders::_1;

namespace armor_detector
{
    detector_node::detector_node(const rclcpp::NodeOptions& options)
    : Node("armor_detector", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting detector node...");

        try
        {
            //detector类初始化
            this->detector_ = init_detector();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        
        //armors pub
        armors_pub = this->create_publisher<global_interface::msg::Target>("/armor_info", rclcpp::SensorDataQoS());

        time_start = std::chrono::steady_clock::now();

        // Subscriptions transport type
        transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

        //image sub
        img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/hik_img",
        std::bind(&detector_node::image_callback, this, _1), transport_));
    }

    detector_node::~detector_node()
    {

    }

    std::unique_ptr<detector> detector_node::init_detector()
    {
        this->declare_parameter("armor_type_wh_thres", 3);
        this->declare_parameter("max_lost_cnt", 5);
        this->declare_parameter("max_armors_cnt", 8);
        this->declare_parameter("max_v", 8);
        this->declare_parameter("max_delta_t", 50);
        this->declare_parameter("no_crop_thres", 1e-2);
        this->declare_parameter("hero_danger_zone", 4);
        this->declare_parameter("color", true);
        
        //TODO:set by own path
        this->declare_parameter("camera_name", "00J90630561"); //相机型号
        this->declare_parameter("camera_param_path", "/home/tup/Desktop/tup_2023/src/global_user/config/camera.yaml");
        this->declare_parameter("network_path", "/home/tup/Desktop/tup_2023/src/vehicle_system/autoaim/armor_detector/model/opt-0527-002.xml");
        
        this->declare_parameter("debug_without_com", true);
        this->declare_parameter("using_imu", false);
        this->declare_parameter("using_roi", true);
        this->declare_parameter("show_aim_cross", false);
        this->declare_parameter("show_img", true);
        this->declare_parameter("detect_red", true);
        this->declare_parameter("show_fps", true);
        this->declare_parameter("print_letency", true);
        this->declare_parameter("print_target_info", true);
        
        this->declare_parameter("anti_spin_judge_high_thres", 2e4);
        this->declare_parameter("anti_spin_judge_low_thres", 2e3);
        this->declare_parameter("anti_spin_max_r_multiple", 4.5);
        // this->declare_parameter("hero_danger_zone", 99);
        this->declare_parameter("max_dead_buffer", 2) ;
        this->declare_parameter("max_delta_dist", 0.3);
        // this->declare_parameter("max_delta_t", 50);

        detector_params detector_params_;
        detector_params_.armor_type_wh_thres = this->get_parameter("armor_type_wh_thres").as_int();
        detector_params_.max_lost_cnt = this->get_parameter("max_lost_cnt").as_int();
        detector_params_.max_armors_cnt = this->get_parameter("max_armors_cnt").as_int();
        detector_params_.max_v = this->get_parameter("max_v").as_int();
        detector_params_.max_delta_t = this->get_parameter("max_delta_t").as_int();
        detector_params_.no_crop_thres = this->get_parameter("no_crop_thres").as_double();
        detector_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        bool det_red = this->get_parameter("color").as_bool();
        if(det_red)
            detector_params_.color = RED;
        else
            detector_params_.color = BLUE;

        std::string camera_name = this->get_parameter("camera_name").as_string();
        std::string camera_param_path = this->get_parameter("camera_param_path").as_string();
        std::string network_path = this->get_parameter("network_path").as_string();

        debug_params debug_;
        debug_.detect_red = this->get_parameter("detect_red").as_bool();
        debug_.debug_without_com  = this->get_parameter("debug_without_com").as_bool();
        debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
        debug_.show_img = this->get_parameter("show_img").as_bool();
        debug_.using_imu = this->get_parameter("using_imu").as_bool();
        debug_.using_roi = this->get_parameter("using_roi").as_bool();
        debug_.show_fps = this->get_parameter("show_fps").as_bool();
        debug_.print_letency = this->get_parameter("print_letency").as_bool();
        debug_.print_target_info = this->get_parameter("print_target_info").as_bool();


        gyro_params gyro_params_;
        gyro_params_.anti_spin_judge_high_thres = this->get_parameter("anti_spin_judge_high_thres").as_double();
        gyro_params_.anti_spin_judge_low_thres = this->get_parameter("anti_spin_judge_low_thres").as_double();
        gyro_params_.anti_spin_max_r_multiple = this->get_parameter("anti_spin_max_r_multiple").as_double();
        gyro_params_.hero_danger_zone = this->get_parameter("hero_danger_zone").as_int();
        gyro_params_.max_dead_buffer = this->get_parameter("max_dead_buffer").as_int() ;
        gyro_params_.max_delta_dist = this->get_parameter("max_delta_dist").as_double();
        gyro_params_.max_delta_t = this->get_parameter("max_delta_t").as_int();

        return std::make_unique<detector>(camera_name, camera_param_path, network_path, detector_params_, debug_, gyro_params_);
    }

    void detector_node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
    {
        // RCLCPP_INFO(this->get_logger(), "image callback...");
        global_user::TaskData src;
        std::vector<Armor> armors;

        if(!img_info)
        {
            return;
        }

        auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
        img.copyTo(src.img);
        // cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
        // cv::imshow("image", src.img);
        // cv::waitKey(0);

        auto time_img_sub = std::chrono::steady_clock::now();
        src.timestamp = (int)(std::chrono::duration<double, std::milli>(time_img_sub - time_start).count());
        
        if(detector_->armor_detect(src))
        {   //find armors
            // RCLCPP_INFO(this->get_logger(), "armors detector...");
            Eigen::Vector3d aiming_point_cam;
            
            //target's spinning status detect 
            if(detector_->gyro_detector(src, aiming_point_cam))
            {
                global_interface::msg::Target target_info;
                target_info.aiming_point.x = aiming_point_cam[0];
                target_info.aiming_point.y = aiming_point_cam[1];
                target_info.aiming_point.z = aiming_point_cam[2];
                target_info.timestamp = src.timestamp;

                //publish target's information containing 3d point and timestamp.
                armors_pub->publish(target_info);
            }
            // global_interface::msg::Armors armors_info;
            // for(int ii = 0; ii < detector_->armors.size(); ii++)
            // {
            //     armors_info.armors[ii].id = detector_->armors[ii].id;
            //     armors_info.armors[ii].color = detector_->armors[ii].color;
            //     armors_info.armors[ii].area = detector_->armors[ii].area;
            //     armors_info.armors[ii].key = detector_->armors[ii].key;
            //     armors_info.armors[ii].center3d_cam.x = detector_->armors[ii].center3d_cam[0];
            //     armors_info.armors[ii].center3d_cam.y = detector_->armors[ii].center3d_cam[1];
            //     armors_info.armors[ii].center3d_cam.z = detector_->armors[ii].center3d_cam[2];
            //     armors_info.armors[ii].center3d_world.x = detector_->armors[ii].center3d_world[0];
            //     armors_info.armors[ii].center3d_world.x = detector_->armors[ii].center3d_world[1];
            //     armors_info.armors[ii].center3d_world.x = detector_->armors[ii].center3d_world[2];
            //     armors_info.armors[ii].type = detector_->armors[ii].type;
            // }
            // armors_pub->publish(armors_info);
        }
    }
} //namespace detector

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<armor_detector::detector_node>());
    rclcpp::shutdown();

    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(armor_detector::detector_node)