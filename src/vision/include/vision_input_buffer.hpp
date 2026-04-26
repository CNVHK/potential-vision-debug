#ifndef POTENTIAL_VISION_INPUT_BUFFER_HPP
#define POTENTIAL_VISION_INPUT_BUFFER_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "rm_interfaces/msg/referee.hpp"
#include "vision_types.hpp"

class VisionInputBuffer {
public:
    void set_image(const sensor_msgs::msg::Image::SharedPtr & msg);
    void set_imu(const sensor_msgs::msg::Imu::SharedPtr & msg);
    void set_referee(const rm_interfaces::msg::Referee::SharedPtr & msg);
    void notify_shutdown();

    sensor_msgs::msg::Image::SharedPtr wait_for_image(const std::atomic<bool> & running);
    auto_aim::VisionFrameInput make_frame_input(
        const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & img,
        const rclcpp::Time & current_ros_time,
        std::chrono::steady_clock::time_point current_steady_time) const;

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    rm_interfaces::msg::Referee referee_;
    Eigen::Quaterniond q_{Eigen::Quaterniond::Identity()};
    bool has_new_image_ = false;
    bool has_imu_ = false;
    bool has_referee_ = false;
};

#endif  // POTENTIAL_VISION_INPUT_BUFFER_HPP
