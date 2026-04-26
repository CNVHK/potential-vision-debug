#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "auto_aim_pipeline.hpp"
#include "rm_interfaces/msg/referee.hpp"
#include "vision_input_buffer.hpp"
#include "vision_ros_publisher.hpp"

class VisionNode : public rclcpp::Node {
public:
    explicit VisionNode() : Node("vision_node"), running_(true)
    {
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/hik_camera/image_raw", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::image_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavlink/serial_driver/imu_raw", rclcpp::SensorDataQoS(),
            std::bind(&VisionNode::imu_callback, this, std::placeholders::_1));
        referee_sub_ = this->create_subscription<rm_interfaces::msg::Referee>(
            "/mavlink/referee", 10,
            std::bind(&VisionNode::referee_callback, this, std::placeholders::_1));

        this->declare_parameter("config_path", "");
        config_path_ = this->get_parameter("config_path").as_string();
        publisher_ = std::make_unique<VisionRosPublisher>(*this, debug_);
        processing_thread_ = std::thread(&VisionNode::processing_loop, this);
    }

    ~VisionNode() override
    {
        running_ = false;
        input_buffer_.notify_shutdown();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<rm_interfaces::msg::Referee>::SharedPtr referee_sub_;

    VisionInputBuffer input_buffer_;
    std::unique_ptr<auto_aim::AutoAimPipeline> pipeline_;
    std::unique_ptr<VisionRosPublisher> publisher_;
    std::string config_path_;
    std::thread processing_thread_;
    std::atomic<bool> running_;
    bool debug_ = false;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        input_buffer_.set_image(msg);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        input_buffer_.set_imu(msg);
        const auto & orientation = msg->orientation;
        RCLCPP_INFO(
            this->get_logger(), "Received IMU: w=%.2f, x=%.2f, y=%.2f, z=%.2f",
            orientation.w, orientation.x, orientation.y, orientation.z);
    }

    void referee_callback(const rm_interfaces::msg::Referee::SharedPtr msg)
    {
        input_buffer_.set_referee(msg);
    }

    void processing_loop()
    {
        pipeline_ = std::make_unique<auto_aim::AutoAimPipeline>(config_path_);

        if (debug_) {
            cv::namedWindow("HikCamera", cv::WINDOW_NORMAL);
            cv::resizeWindow("HikCamera", 640, 480);
        }

        while (running_) {
            auto msg = input_buffer_.wait_for_image(running_);
            if (!msg) {
                continue;
            }
            process_image(msg);
        }
    }

    void process_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            auto current_ros_time = this->now();
            auto current_steady_time = std::chrono::steady_clock::now();
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            auto input = input_buffer_.make_frame_input(msg, img, current_ros_time, current_steady_time);
            auto output = pipeline_->process(input);

            RCLCPP_INFO(
                this->get_logger(), "Control: %d, Shoot: %d, Pitch: %.3f; Yaw: %.3f, Imu_y: %.3f",
                output.command.control, output.command.shoot, output.command.pitch, output.command.yaw,
                output.gimbal_ypr[0] * (180.0 / CV_PI));

            publisher_->publish(output, msg, img, current_ros_time);
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
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
