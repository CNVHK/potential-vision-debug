#ifndef POTENTIAL_VISION_TYPES_HPP
#define POTENTIAL_VISION_TYPES_HPP

#include <chrono>
#include <list>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include "armor.hpp"
#include "target.hpp"
#include "command.hpp"
#include "rm_interfaces/msg/cboard.hpp"
#include "rm_interfaces/msg/target.hpp"

namespace auto_aim {

struct ReprojectedArmor {
    std::vector<cv::Point2f> points;
};

struct VisionFrameInput {
    cv::Mat image;
    rclcpp::Time image_stamp;
    rclcpp::Time current_ros_time;
    std::chrono::steady_clock::time_point steady_stamp;
    Eigen::Quaterniond gimbal_orientation{Eigen::Quaterniond::Identity()};
    bool has_imu = false;
    Color enemy_color = Color::red;
    bool has_referee = false;
    double bullet_speed = 25.0;
};

struct VisionFrameOutput {
    Command command{false, false, 0.0, 0.0, false};
    Eigen::Vector3d gimbal_ypr{0.0, 0.0, 0.0};
    std::vector<Armor> armors;
    std::list<Target> targets;
    std::vector<ReprojectedArmor> reprojected_armors;
    std::optional<rm_interfaces::msg::Target> target_msg;
    rm_interfaces::msg::Cboard cboard;
};

}  // namespace auto_aim

#endif  // POTENTIAL_VISION_TYPES_HPP
