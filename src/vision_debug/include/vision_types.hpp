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

// 调试绘制用的重投影装甲板点集。它不参与控制闭环，只用于检查 EKF 预测姿态是否合理。
struct ReprojectedArmor {
    std::vector<cv::Point2f> points;
};

// 单帧算法输入。ROS 层负责把多路消息缓存成这个结构，Pipeline 只依赖这个纯输入。
//
// 这里同时保存 ROS 时间和 steady_clock 时间：
// - ROS 时间用于发布消息、debug marker 和对齐图像时间戳；
// - steady_clock 用于算法预测，避免系统时间跳变影响 EKF 和弹道预测。
struct VisionFrameInput {
    // cv::Mat 只引用当前图像数据，不在结构内做深拷贝；调用方必须保证本帧处理期间图像有效。
    cv::Mat image;
    rclcpp::Time image_stamp;
    rclcpp::Time current_ros_time;
    std::chrono::steady_clock::time_point steady_stamp;

    // 云台姿态来自 IMU。无 IMU 时保持单位四元数，has_imu 用于以后增加降级策略。
    Eigen::Quaterniond gimbal_orientation{Eigen::Quaterniond::Identity()};
    bool has_imu = false;

    // enemy_color 由裁判系统队伍颜色推导；无裁判系统时沿用默认红色敌方策略。
    Color enemy_color = Color::red;
    bool has_referee = false;

    // 当前仍保持旧逻辑默认 25 m/s，后续可切到裁判系统弹速或动态参数。
    double bullet_speed = 25.0;
};

// 单帧算法输出。它是 Pipeline 和 ROS 发布层之间的唯一数据契约。
struct VisionFrameOutput {
    // 自瞄控制结果。yaw/pitch 当前沿用下游协议，单位为度。
    Command command{false, false, 0.0, 0.0, false};

    // 当前云台姿态欧拉角，主要用于日志和射击判断。
    Eigen::Vector3d gimbal_ypr{0.0, 0.0, 0.0};

    // armors 是已经完成颜色过滤、排序，并按需完成解算的候选集合。
    std::vector<Armor> armors;
    std::list<Target> targets;

    // 以下字段用于 ROS 输出和可视化，不应反向影响算法决策。
    std::vector<ReprojectedArmor> reprojected_armors;
    std::optional<rm_interfaces::msg::Target> target_msg;
    rm_interfaces::msg::Cboard cboard;
};

}  // namespace auto_aim

#endif  // POTENTIAL_VISION_TYPES_HPP
