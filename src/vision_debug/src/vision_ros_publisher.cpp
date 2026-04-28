#include "vision_ros_publisher.hpp"

#include <fmt/chrono.h>

VisionRosPublisher::VisionRosPublisher(rclcpp::Node & node, bool debug)
    : node_(node), debug_(debug), last_fps_time_(node.now())
{
    target_pub_ = node_.create_publisher<rm_interfaces::msg::Target>("/vision/target", 10);
    gimbal_pub_ = node_.create_publisher<rm_interfaces::msg::GimbalCmd>(
        "/vision/gimbal_cmd", rclcpp::SensorDataQoS());
    cboard_pub_ = node_.create_publisher<rm_interfaces::msg::Cboard>(
        "/vision/auto_aim", rclcpp::SensorDataQoS());
    debug_armor_pub_ = node_.create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/debug/armor_world_xyz", 10);
    debug_img_pub_ = node_.create_publisher<sensor_msgs::msg::CompressedImage>(
        "/vision/debug/image/compressed", rclcpp::SensorDataQoS());
    armor_marker_pub_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
        "/vision/debug/armor_markers", 10);
}

void VisionRosPublisher::publish(
    auto_aim::VisionFrameOutput & output, const sensor_msgs::msg::Image::SharedPtr & image_msg,
    cv::Mat & image, const rclcpp::Time & current_ros_time)
{
    publish_result(output, current_ros_time);
    draw_debug(image, output, current_ros_time);
    publish_fps();
    publish_debug_image(image_msg, image);
}

void VisionRosPublisher::publish_result(
    const auto_aim::VisionFrameOutput & output, const rclcpp::Time & stamp)
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

void VisionRosPublisher::draw_debug(
    cv::Mat & image, auto_aim::VisionFrameOutput & output, const rclcpp::Time & current_ros_time)
{
    if (!output.targets.empty() && !output.armors.empty()) {
        auto & target = output.targets.front();
        const auto & armor = output.armors.front();
        draw_points(image, armor.points, {0, 0, 255}, 3);
        publish_armor_markers(target, armor, current_ros_time);

        for (const auto & reprojected_armor : output.reprojected_armors) {
            draw_points(image, reprojected_armor.points, {255, 255, 0}, 3);
        }
    }

    if (debug_) {
        auto img_gimbal_info = fmt::format(
            "gimbal_yaw: {:.2f}, gimbal_pitch: {:.2f}, imu_yaw: {:.2f}",
            output.command.yaw, output.command.pitch, output.gimbal_ypr[0] * (180.0 / CV_PI));
        cv::putText(
            image, img_gimbal_info, cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 1.0,
            cv::Scalar(0, 255, 255), 3);
        cv::imshow("HikCamera", image);
        cv::waitKey(1);
    }
}

void VisionRosPublisher::publish_armor_markers(
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

void VisionRosPublisher::publish_fps()
{
    frame_count_++;
    rclcpp::Time current_time = node_.now();
    double elapsed_time = (current_time - last_fps_time_).seconds();
    if (elapsed_time >= 1.0) {
        current_fps_ = frame_count_ / elapsed_time;
        RCLCPP_INFO(node_.get_logger(), "Vision Node FPS: %.2f", current_fps_);
        frame_count_ = 0;
        last_fps_time_ = current_time;
    }
}

void VisionRosPublisher::publish_debug_image(
    const sensor_msgs::msg::Image::SharedPtr & image_msg, const cv::Mat & image)
{
    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.header = image_msg->header;
    compressed_msg.format = "jpeg";
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 50};
    cv::imencode(".jpg", image, compressed_msg.data, compression_params);
    debug_img_pub_->publish(compressed_msg);
}

void VisionRosPublisher::draw_points(
    cv::Mat & image, const std::vector<cv::Point> & points, const cv::Scalar & color, int thickness)
{
    std::vector<std::vector<cv::Point>> contours = {points};
    cv::drawContours(image, contours, -1, color, thickness);
}

void VisionRosPublisher::draw_points(
    cv::Mat & image, const std::vector<cv::Point2f> & points, const cv::Scalar & color,
    int thickness)
{
    std::vector<cv::Point> int_points(points.begin(), points.end());
    draw_points(image, int_points, color, thickness);
}
