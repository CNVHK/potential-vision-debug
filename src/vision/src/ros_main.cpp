#include <iostream>
#include <string>
#include <vector>
#include <fmt/chrono.h>
#include <sstream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/cboard.hpp"
#include "rm_interfaces/msg/referee.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include "detector.hpp"
#include "armor.hpp"
#include "tracker.hpp"
#include "solver.hpp"
#include "aimer.hpp"
#include "shooter.hpp"
#include "math_tools.hpp"
class VisionNode : public rclcpp::Node{
public:
    explicit VisionNode() : Node("vision_node"), running_(true), has_new_image_(false){
        target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("/vision/target", 10);
        gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("/vision/gimbal_cmd", rclcpp::SensorDataQoS());
        cboard_pub_ = this->create_publisher<rm_interfaces::msg::Cboard>("/vision/auto_aim", rclcpp::SensorDataQoS());
        debug_armor_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/vision/debug/armor_world_xyz", 10);
        // debug_target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/vision/debug/traget", 10);
        debug_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/vision/debug/image/compressed", rclcpp::SensorDataQoS());
        armor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/vision/debug/armor_markers", 10);
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/hik_camera/image_raw",rclcpp::SensorDataQoS(), std::bind(&VisionNode::image_callback, 
            this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/mavlink/serial_driver/imu_raw", rclcpp::SensorDataQoS(), std::bind(&VisionNode::imu_callback,
            this, std::placeholders::_1));
        referee_sub_ = this->create_subscription<rm_interfaces::msg::Referee>( "/mavlink/referee", 10, std::bind(&VisionNode::referee_callback, 
            this, std::placeholders::_1));
        // timer_ = this->create_wall_timer(
        // std::chrono::microseconds(10), std::bind(&VisionNode::timer_callback, this));
        last_fps_time_ = this->now();
        this->declare_parameter("config_path", ""); 
        config_path_ = this->get_parameter("config_path").as_string();
        processing_thread_ = std::thread(&VisionNode::processingLoop, this);
    
    }
    
    ~VisionNode() override {
        running_ = false;
        cv_.notify_all();
        if (processing_thread_.joinable())
            processing_thread_.join();
    }

    
private:
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Cboard>::SharedPtr cboard_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_armor_pub_;
    // rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr armor_marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Referee>::SharedPtr referee_sub_;
    rclcpp::Time last_image_stamp_;
    rm_interfaces::msg::Target target_;
    
    rm_interfaces::msg::GimbalCmd gimbal_cmd_;
    rm_interfaces::msg::Cboard cboard_;
    rm_interfaces::msg::Referee referee_;
    sensor_msgs::msg::Image::SharedPtr img_latest_;
    std::unique_ptr<auto_aim::Detector> detector_;
    std::unique_ptr<auto_aim::Solver> solver_;
    std::unique_ptr<auto_aim::Tracker> tracker_;
    std::unique_ptr<auto_aim::Aimer> aimer_;
    std::unique_ptr<auto_aim::Shooter> shooter_;
    std::string config_path_;
    std::thread processing_thread_;
    std::atomic<bool> running_;
    std::mutex mutex_;
    std::condition_variable cv_; 
    Eigen::Quaterniond q_;
    bool has_new_image_;
    bool debug_ = false;
    rclcpp::TimerBase::SharedPtr timer_{nullptr};
    int frame_count_ = 0;                    
    rclcpp::Time last_fps_time_;            
    double current_fps_ = 0.0;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            img_latest_ = msg;
            has_new_image_ = true;
        }
        cv_.notify_one();
    }
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)  {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        q_ = Eigen::Quaterniond(qw, qx, qy, qz); 
        q_.normalize();
        RCLCPP_INFO(this->get_logger(), "Received IMU: w=%.2f, x=%.2f, y=%.2f, z=%.2f", qw, qx, qy, qz);
    }
    void referee_callback(const rm_interfaces::msg::Referee::SharedPtr msg) {
        referee_ = *msg;
    }

    // void timer_callback() {
    // }
                                                           
    void draw_points(cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness) {
        std::vector<std::vector<cv::Point>> contours = {points};
        cv::drawContours(img, contours, -1, color, thickness);
    }

    void draw_points(cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness) {
        std::vector<cv::Point> int_points(points.begin(), points.end());
        draw_points(img, int_points, color, thickness);
    }
    void processingLoop(){
        detector_ = std::make_unique<auto_aim::Detector>(config_path_, false);        
        solver_ = std::make_unique<auto_aim::Solver>(config_path_);
        tracker_ = std::make_unique<auto_aim::Tracker>(config_path_, *solver_);
        aimer_ = std::make_unique<auto_aim::Aimer>(config_path_);
        shooter_ = std::make_unique<auto_aim::Shooter>(config_path_);

        if (debug_){
            cv::namedWindow("HikCamera", cv::WINDOW_NORMAL);
            cv::resizeWindow("HikCamera", 640, 480);
        }
        while(running_){
            sensor_msgs::msg::Image::SharedPtr msg;
            {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return has_new_image_ || !running_; });
            if (!running_) break;

            msg = img_latest_;
            has_new_image_ = false;
            }
            if (msg){
                vision_auto_aim(msg);
            }

        }
        
    }

    // void gimbal_load(const auto_aim::Command &command){
    //     gimbal_cmd_.control = command.control;
    //     gimbal_cmd_.fire_advice = command.shoot;
    //     gimbal_cmd_.pitch = command.pitch;
    //     gimbal_cmd_.yaw = command.yaw;
    // }

    void target_load(const auto_aim::Target &target) {
        auto ekf_x = target.ekf_x();
        target_.x = ekf_x[0];
        target_.vx = ekf_x[1];
        target_.y = ekf_x[2];
        target_.vy = ekf_x[3];
        target_.z = ekf_x[4];
        target_.vz = ekf_x[5];
        target_.yaw = ekf_x[6];
        target_.vyaw = ekf_x[7];
        target_.r = ekf_x[8];
        target_.dr = ekf_x[9];
        target_.dz = ekf_x[10];
        target_.dr = 0.0;
        target_.dz = 0.0;
    }




    void vision_auto_aim(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            rclcpp::Time current_ros_time = this->now();
            auto current_steady_time = std::chrono::steady_clock::now();
            rclcpp::Time img_ros_time = msg->header.stamp;
            auto delay = current_ros_time - img_ros_time;
            auto t = current_steady_time - std::chrono::nanoseconds(delay.nanoseconds());
            // auto t = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>(
            //     std::chrono::nanoseconds(img_time.nanoseconds())
            // );
            // auto t = std::chrono::steady_clock::now();
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            solver_->set_R_gimbal2world(q_);
            Eigen::Vector3d ypr = tool::eulers(solver_->R_gimbal2world(), 2, 1, 0);
            auto target_color = referee_.is_red ? auto_aim::Color::blue : auto_aim::Color::red; 
            auto armors = detector_->detect_armors(img, target_color);
            // RCLCPP_INFO(this->get_logger(), "is_red %d", referee_.is_red);
            auto targets = tracker_->track(armors, t, target_color, false);
            auto command = aimer_->aim(targets, t, 25, solver_->R_gimbal2world());
            command.shoot = shooter_->shoot(command, *aimer_, targets, ypr);
            cboard_.is_detected = command.control;
            cboard_.yaw = command.yaw;
            cboard_.pitch = command.pitch;
            cboard_.robot_id = 0;
            cboard_.is_fire = command.shoot;
            // gimbal_load(command);
            // gimbal_pub_->publish(gimbal_cmd_);
            RCLCPP_INFO(this->get_logger(), "Control: %d, Shoot: %d, Pitch: %.3f; Yaw: %.3f, Imu_y: %.3f", command.control, command.shoot, command.pitch, command.yaw, ypr[0] * (180.0 / CV_PI));
            if (!targets.empty()) {
                auto target = targets.front();
                auto ekf_x = target.ekf_x();
                cboard_.wr = - ekf_x[7] * (60 / (2*CV_PI));
                if (!armors.empty()) {
                    auto armor = armors.front();
                    // auto xyz = armors.front().xyz_in_world;
                    // RCLCPP_INFO(this->get_logger(), "World XYZ: [%.2f, %.2f, %.2f]", xyz.x(), xyz.y(), xyz.z());
                    cboard_.robot_id = armor.name + 1;
                    cboard_.distance = armor.ypd_in_world[2];
                    geometry_msgs::msg::PointStamped pt_msg;
                    pt_msg.header.frame_id = "odom"; 
                    pt_msg.point.x = armor.xyz_in_world[0];
                    pt_msg.point.y = armor.xyz_in_world[1];
                    pt_msg.point.z = armor.xyz_in_world[2];
                    debug_armor_pub_->publish(pt_msg);
                    target_load(target);
                    target_pub_->publish(target_);
                    // if (debug_){
                    draw_points(img, armor.points, {0, 0, 255}, 3);
                    std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
                    visualization_msgs::msg::MarkerArray marker_array;
                    int marker_id = 0;
                    
                    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
                        visualization_msgs::msg::Marker marker;
                        marker.header.frame_id = "odom"; 
                        marker.header.stamp = current_ros_time;
                        marker.ns = "ekf_armors";
                        marker.id = marker_id++;
                        marker.type = visualization_msgs::msg::Marker::CUBE; 
                        marker.action = visualization_msgs::msg::Marker::ADD;
                        marker.pose.position.x = xyza[0];
                        marker.pose.position.y = xyza[1];
                        marker.pose.position.z = xyza[2];

                        Eigen::Quaterniond q(Eigen::AngleAxisd(xyza[3], Eigen::Vector3d::UnitZ()));
                        marker.pose.orientation.x = q.x();
                        marker.pose.orientation.y = q.y();
                        marker.pose.orientation.z = q.z();
                        marker.pose.orientation.w = q.w();

                        if (armor.type == auto_aim::ArmorType::small)
                        {
                            //设置尺寸 (模拟装甲板大小，假设厚度0.02m，宽0.135m，高0.125m)
                            marker.scale.x = 0.02;  
                            marker.scale.y = 0.135; 
                            marker.scale.z = 0.125; 
                        } 
                        else{
                            marker.scale.x = 0.02;  
                            marker.scale.y = 0.235; 
                            marker.scale.z = 0.125; 
                        }

                        marker.color.r = 0.0f;
                        marker.color.g = 1.0f;
                        marker.color.b = 0.0f;
                        marker.color.a = 0.8f; 

                        marker.lifetime = rclcpp::Duration::from_seconds(0.1); 

                        marker_array.markers.push_back(marker);
                    }
                    
                    armor_marker_pub_->publish(marker_array);
                    auto min_error = 1e10;
                    Eigen::Vector4d detector_xyza;
                    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
                        auto yaw_error = std::abs(armor.ypr_in_world[0] - xyza[3]);
                        if (yaw_error < min_error) {
                            min_error = yaw_error;
                            detector_xyza = xyza;
                        }
                    }
                    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
                        if (xyza == detector_xyza) continue;
                        auto image_points =
                            solver_->reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
                        draw_points(img, image_points, {255, 255, 0}, 3);
                    }
                    // }
                }
            }
            else{
                cboard_.wr = 0;
                cboard_.distance = 0;
            }
            // double delay_ms = delay.seconds() * 1000.0;
            // RCLCPP_INFO(this->get_logger(), "Image Delay: %f ms", delay_ms);
            frame_count_++;
            rclcpp::Time current_time = this->now();
            double elapsed_time = (current_time - last_fps_time_).seconds();
            cboard_pub_->publish(cboard_);
            if (elapsed_time >= 1.0) {
                current_fps_ = frame_count_ / elapsed_time;
                RCLCPP_INFO(this->get_logger(), "Vision Node FPS: %.2f", current_fps_);
                
                frame_count_ = 0;
                last_fps_time_ = current_time;
            }
            if (debug_) {
                auto img_gimbal_info = fmt::format("gimbal_yaw: {:.2f}, gimbal_pitch: {:.2f}, imu_yaw: {:.2f}", command.yaw, command.pitch, ypr[0] * (180.0 / CV_PI));
                cv::putText(img, img_gimbal_info, cv::Point(20, 60),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0,cv::Scalar(0, 255, 255), 3);
                cv::imshow("HikCamera", img);
                cv::waitKey(1);
            }
            sensor_msgs::msg::CompressedImage compressed_msg;
            compressed_msg.header = msg->header;
            compressed_msg.format = "jpeg";
            std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 50}; 
            cv::imencode(".jpg", img, compressed_msg.data, compression_params);
            debug_img_pub_->publish(compressed_msg);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}