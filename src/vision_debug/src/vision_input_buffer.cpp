#include "vision_input_buffer.hpp"

void VisionInputBuffer::set_image(const sensor_msgs::msg::Image::SharedPtr & msg)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_image_ = msg;
        has_new_image_ = true;
    }
    cv_.notify_one();
}

void VisionInputBuffer::set_imu(const sensor_msgs::msg::Imu::SharedPtr & msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    double qx = msg->orientation.x;
    double qy = msg->orientation.y;
    double qz = msg->orientation.z;
    double qw = msg->orientation.w;
    q_ = Eigen::Quaterniond(qw, qx, qy, qz);
    q_.normalize();
    has_imu_ = true;
}

void VisionInputBuffer::set_referee(const rm_interfaces::msg::Referee::SharedPtr & msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    referee_ = *msg;
    has_referee_ = true;
}

void VisionInputBuffer::notify_shutdown()
{
    cv_.notify_all();
}

sensor_msgs::msg::Image::SharedPtr VisionInputBuffer::wait_for_image(
    const std::atomic<bool> & running)
{
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [this, &running]() { return has_new_image_ || !running.load(); });
    if (!running.load()) {
        return nullptr;
    }

    auto msg = latest_image_;
    has_new_image_ = false;
    return msg;
}

auto_aim::VisionFrameInput VisionInputBuffer::make_frame_input(
    const sensor_msgs::msg::Image::SharedPtr & msg, cv::Mat & img,
    const rclcpp::Time & current_ros_time,
    std::chrono::steady_clock::time_point current_steady_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    rclcpp::Time img_ros_time = msg->header.stamp;
    auto delay = current_ros_time - img_ros_time;

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
