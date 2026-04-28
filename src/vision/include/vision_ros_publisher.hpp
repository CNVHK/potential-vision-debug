#ifndef POTENTIAL_VISION_ROS_PUBLISHER_HPP
#define POTENTIAL_VISION_ROS_PUBLISHER_HPP

#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_interfaces/msg/cboard.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "vision_types.hpp"

// ROS 输出适配层。
//
// Pipeline 输出的是统一的 VisionFrameOutput，这个类负责把它拆成下游控制消息和调试话题。
// 这样后续更换通信协议、关闭 debug 图像、或增加 rosbag 回放输出时，不需要修改算法编排层。
class VisionRosPublisher {
public:
    VisionRosPublisher(rclcpp::Node & node, bool debug);

    // 发布顺序保持旧逻辑：先控制结果，再 debug 绘制/FPS/压缩图像。
    void publish(
        auto_aim::VisionFrameOutput & output, const sensor_msgs::msg::Image::SharedPtr & image_msg,
        cv::Mat & image, const rclcpp::Time & current_ros_time);

private:
    // 控制与目标状态输出。/vision/auto_aim 是当前下游使用的主输出。
    void publish_result(const auto_aim::VisionFrameOutput & output, const rclcpp::Time & stamp);

    // 调试绘制会直接修改传入图像，然后由 publish_debug_image 发布压缩图。
    void draw_debug(
        cv::Mat & image, auto_aim::VisionFrameOutput & output, const rclcpp::Time & current_ros_time);

    // RViz Marker：展示 EKF 推算出的各装甲板位置。
    void publish_armor_markers(
        auto_aim::Target & target, const auto_aim::Armor & armor, const rclcpp::Time & current_ros_time);
    void publish_fps();
    void publish_debug_image(
        const sensor_msgs::msg::Image::SharedPtr & image_msg, const cv::Mat & image);
    void draw_points(
        cv::Mat & image, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness);
    void draw_points(
        cv::Mat & image, const std::vector<cv::Point2f> & points, const cv::Scalar & color, int thickness);

    rclcpp::Node & node_;
    bool debug_;
    int frame_count_ = 0;
    rclcpp::Time last_fps_time_;
    double current_fps_ = 0.0;

    rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
    rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
    rclcpp::Publisher<rm_interfaces::msg::Cboard>::SharedPtr cboard_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr debug_img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_armor_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr armor_marker_pub_;
};

#endif  // POTENTIAL_VISION_ROS_PUBLISHER_HPP
