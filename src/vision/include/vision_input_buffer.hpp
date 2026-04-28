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

// ROS 输入缓存层。
//
// 图像、IMU、裁判系统消息到达频率不同，这个类负责保存最新状态，并在每帧图像到来时
// 组合成 VisionFrameInput。这样主节点不需要关心多路消息同步细节，Pipeline 也不直接接触 ROS 回调。
class VisionInputBuffer {
public:
    // 图像是驱动处理线程的主时钟：每收到一帧图像就唤醒一次 Pipeline。
    void set_image(const sensor_msgs::msg::Image::SharedPtr & msg);

    // IMU 和裁判系统保存最新值，图像到来时随帧取快照。
    void set_imu(const sensor_msgs::msg::Imu::SharedPtr & msg);
    void set_referee(const rm_interfaces::msg::Referee::SharedPtr & msg);

    // 节点析构时唤醒等待线程，避免处理线程卡在条件变量上。
    void notify_shutdown();

    sensor_msgs::msg::Image::SharedPtr wait_for_image(const std::atomic<bool> & running);

    // 使用图像时间戳和当前 ROS 时间估计图像对应的 steady_clock 时间。
    // 后续预测统一使用 steady_clock，避免 ROS 时间源变化影响 dt。
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
