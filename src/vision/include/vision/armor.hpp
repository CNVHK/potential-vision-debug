#ifndef POTENTIAL_VISION_ARMOR_HPP
#define POTENTIAL_VISION_ARMOR_HPP
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <Eigen/Core>
namespace auto_aim {
    enum Color {
        red = 0,
        blue = 1,
        extinguish = 2,
    };
    const std::vector<std::string> color_names = {"red", "blue", "extinguish"};

    enum ArmorType {
        small,
        big
    };
    const std::vector<std::string> armor_types = {"small", "big"};

    enum ArmorName {
        one,
        two,
        three,
        four,
        five,
        sentry,
        outpost,
        base,
        not_armor

    };
    const std::vector<std::string> armor_name = {"one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};

    enum ArmorPriority {
        first = 1,
        second,
        third,
        forth,
        fifth
    };


    struct Lightbar {
        cv::RotatedRect rotated_rect;
        cv::Point2f center, top_left, top_right, bottom_left, bottom_right, top_middle, bottom_middle, top2bottom, axis;
        Color color;
        double angle, angle_error, width, length, ratio;
        Lightbar(const cv::RotatedRect & rotated_rect);
        // Lightbar() {};
    };

    struct Armor {
        Color color;
        Lightbar right, left;
        std::vector<cv::Point2f> points;
        cv::Point2f center_norm;
        cv::Point2f center, top_left, top_right, bottom_left, bottom_right, top_middle, bottom_middle, top2bottom;
        cv::Mat number_img;
        float confidence;
        double ratio;
        double yaw_raw;
        double side_ratio;
        double yaw_predict;
        double rectangular_error;
        double rectangular_error_diff;
        double top_angle_error;
        double bottom_angle_error;
        double top_bottom_angle_error;
        double light_angle_error;
        bool is_detected;
        Eigen::Vector3d xyz_in_gimbal;
        Eigen::Vector3d xyz_in_world;
        Eigen::Vector3d ypr_in_gimbal;
        Eigen::Vector3d ypr_in_world;
        Eigen::Vector3d ypd_in_world;
        ArmorName name;
        ArmorType type;
        ArmorPriority priority;
        Armor(const Lightbar &left, const Lightbar &right);
        // Armor() {};
    };
}
#endif //POTENTIAL_VISION_ARMOR_HPP