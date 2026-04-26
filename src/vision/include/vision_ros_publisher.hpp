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

class VisionRosPublisher {
public:
    VisionRosPublisher(rclcpp::Node & node, bool debug);

    void publish(
        auto_aim::VisionFrameOutput & output, const sensor_msgs::msg::Image::SharedPtr & image_msg,
        cv::Mat & image, const rclcpp::Time & current_ros_time);

private:
    void publish_result(const auto_aim::VisionFrameOutput & output, const rclcpp::Time & stamp);
    void draw_debug(
        cv::Mat & image, auto_aim::VisionFrameOutput & output, const rclcpp::Time & current_ros_time);
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
