#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <fmt/chrono.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "auto_aim_pipeline.hpp"
#include "math_tools.hpp"
#include "rm_interfaces/msg/cboard.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/referee.hpp"
#include "rm_interfaces/msg/target.hpp"

class VisionNode : public rclcpp::Node {
public:
    explicit VisionNode() : Node("vision_node"), running_(true), has_new_image_(false)
    {
        target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("/vision/target", 10);
        gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
            "/vision/gimbal_cmd", rclcpp::SensorDataQoS());
        cboard_pub_ = this->create_publisher<rm_interfaces::msg::Cboard>(
            "/vision/auto_aim", rclcpp::SensorDataQoS());
        debug_armor_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/vision/debug/armor_world_xyz", 10);
        debug_img_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/vision/debug/image/compressed", rclcpp::SensorDataQoS());
        armor_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/vision/debug/armor_markers", 10);

        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hik_camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::image_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavlink/serial_driver/imu_raw", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::imu_callback, this, std::placeholders::_1));
        referee_sub_ = this->create_subscription<rm_interfaces::msg::Referee>(
            "/mavlink/referee", 10,
            std::bind(&VisionNode::referee_callback, this, std::placeholders::_1));

        last_fps_time_ = this->now();
        this->declare_parameter("config_path", "");
        config_path_ = this->get_parameter("config_path").as_string();
        processing_thread_ = std::thread(&VisionNode::processingLoop, this);
    }

    ~VisionNode() override
    {
        running_ = false;
        cv_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Cboard>::SharedPtr cboard_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_armor_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr armor_marker_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Referee>::SharedPtr referee_sub_;

    rm_interfaces::msg::Referee referee_;
    sensor_msgs::msg::Image::SharedPtr img_latest_;
    std::unique_ptr<auto_aim::AutoAimPipeline> pipeline_;
    std::string config_path_;
    std::thread processing_thread_;
    std::atomic<bool> running_;
    std::mutex mutex_;
    std::condition_variable cv_;
    Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
    bool has_imu_ = false;
    bool has_referee_ = false;
    bool has_new_image_;
    bool debug_ = false;
    int frame_count_ = 0;
    rclcpp::Time last_fps_time_;
    double current_fps_ = 0.0;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            img_latest_ = msg;
            has_new_image_ = true;
        }
        cv_.notify_one();
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;
        q_ = Eigen::Quaterniond(qw, qx, qy, qz);
        q_.normalize();
        has_imu_ = true;
        RCLCPP_INFO(
            this->get_logger(), "Received IMU: w=%.2f, x=%.2f, y=%.2f, z=%.2f", qw, qx, qy, qz);
    }

    void referee_callback(const rm_interfaces::msg::Referee::SharedPtr msg)
    {
        referee_ = *msg;
        has_referee_ = true;
    }

    void processingLoop()
    {
        pipeline_ = std::make_unique<auto_aim::AutoAimPipeline>(config_path_);

        if (debug_) {
            cv::namedWindow("HikCamera", cv::WINDOW_NORMAL);
            cv::resizeWindow("HikCamera", 640, 480);
        }

        while (running_) {
            sensor_msgs::msg::Image::SharedPtr msg;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this]() { return has_new_image_ || !running_; });
                if (!running_) {
                    break;
                }

                msg = img_latest_;
                has_new_image_ = false;
            }
            if (msg) {
                process_image(msg);
            }
        }
    }

    auto_aim::VisionFrameInput make_frame_input(
        const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & img)
    {
        rclcpp::Time current_ros_time = this->now();
        rclcpp::Time img_ros_time = msg->header.stamp;
        auto delay = current_ros_time - img_ros_time;
        auto current_steady_time = std::chrono::steady_clock::now();

        auto_aim::VisionFrameInput input;
        input.image = img;
        input.image_stamp = img_ros_time;
        input.current_ros_time = current_ros_time;
        input.steady_stamp = current_steady_time - std::chrono::nanoseconds(delay.nanoseconds());
        input.gimbal_orientation = q_;
        input.has_imu = has_imu_;
        input.enemy_color = referee_.is_red ? auto_aim::Color::blue : auto_aim::Color::red;
        input.has_referee = has_referee_;
        input.bullet_speed = 25.0;
        return input;
    }

    void process_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            auto input = make_frame_input(msg, img);
            auto output = pipeline_->process(input);

            RCLCPP_INFO(
                this->get_logger(), "Control: %d, Shoot: %d, Pitch: %.3f; Yaw: %.3f, Imu_y: %.3f",
                output.command.control, output.command.shoot, output.command.pitch, output.command.yaw,
                output.gimbal_ypr[0] * (180.0 / CV_PI));

            publish_result(output, input.current_ros_time);
            draw_debug(img, output, input.current_ros_time);
            publish_fps();
            publish_debug_image(msg, img);
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    void publish_result(const auto_aim::VisionFrameOutput & output, const rclcpp::Time & stamp)
    {
        cboard_pub_->publish(output.cboard);

        if (output.target_msg.has_value()) {
            target_pub_->publish(output.target_msg.value());
        }

        if (!output.armors.empty()) {
            const auto & armor = output.armors.front();
            geometry_msgs::msg::PointStamped pt_msg;
            pt_msg.header.frame_id = "odom";
            pt_msg.header.stamp = stamp;
            pt_msg.point.x = armor.xyz_in_world[0];
            pt_msg.point.y = armor.xyz_in_world[1];
            pt_msg.point.z = armor.xyz_in_world[2];
            debug_armor_pub_->publish(pt_msg);
        }
    }

    void draw_points(
        cv::Mat & img, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
    {
        std::vector<std::vector<cv::Point>> contours = {points};
        cv::drawContours(img, contours, -1, color, thickness);
    }

    void draw_points(
        cv::Mat & img, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness)
    {
        std::vector<cv::Point> int_points(points.begin(), points.end());
        draw_points(img, int_points, color, thickness);
    }

    void draw_debug(
        cv::Mat & img, auto_aim::VisionFrameOutput & output, const rclcpp::Time & current_ros_time)
    {
        if (!output.targets.empty() && !output.armors.empty()) {
            auto & target = output.targets.front();
            const auto & armor = output.armors.front();
            draw_points(img, armor.points, {0, 0, 255}, 3);
            publish_armor_markers(target, armor, current_ros_time);

            for (const auto & reprojected_armor : output.reprojected_armors) {
                draw_points(img, reprojected_armor.points, {255, 255, 0}, 3);
            }
        }

        if (debug_) {
            auto img_gimbal_info = fmt::format(
                "gimbal_yaw: {:.2f}, gimbal_pitch: {:.2f}, imu_yaw: {:.2f}",
                output.command.yaw, output.command.pitch, output.gimbal_ypr[0] * (180.0 / CV_PI));
            cv::putText(
                img, img_gimbal_info, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1.0,
                cv::Scalar(0, 255, 255), 3);
            cv::imshow("HikCamera", img);
            cv::waitKey(1);
        }
    }

    void publish_armor_markers(
        auto_aim::Target & target, const auto_aim::Armor & armor, const rclcpp::Time & current_ros_time)
    {
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

            if (armor.type == auto_aim::ArmorType::small) {
                marker.scale.x = 0.02;
                marker.scale.y = 0.135;
                marker.scale.z = 0.125;
            } else {
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
    }

    void publish_fps()
    {
        frame_count_++;
        rclcpp::Time current_time = this->now();
        double elapsed_time = (current_time - last_fps_time_).seconds();
        if (elapsed_time >= 1.0) {
            current_fps_ = frame_count_ / elapsed_time;
            RCLCPP_INFO(this->get_logger(), "Vision Node FPS: %.2f", current_fps_);
            frame_count_ = 0;
            last_fps_time_ = current_time;
        }
    }

    void publish_debug_image(const sensor_msgs::msg::Image::SharedPtr & msg, const cv::Mat & img)
    {
        sensor_msgs::msg::CompressedImage compressed_msg;
        compressed_msg.header = msg->header;
        compressed_msg.format = "jpeg";
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 50};
        cv::imencode(".jpg", img, compressed_msg.data, compression_params);
        debug_img_pub_->publish(compressed_msg);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
