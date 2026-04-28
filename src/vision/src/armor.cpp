
#include "armor.hpp"
#include <cmath>
namespace auto_aim {
    Lightbar::Lightbar(const cv::RotatedRect &rotated_rect)
        : rotated_rect(rotated_rect){
        std::vector<cv::Point2f> corners(4);
        rotated_rect.points(&corners[0]);
        std::sort(corners.begin(), corners.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
            return a.y < b.y;
        });

        center = rotated_rect.center;
        top_middle = (corners[0] + corners[1]) / 2;
        bottom_middle = (corners[2] + corners[3]) / 2;
        top2bottom = bottom_middle - top_middle;
        axis = top2bottom / cv::norm(top2bottom);
        width = (cv::norm(corners[0] - corners[1]) + cv::norm(corners[2] - corners[3])) / 2;
        length = cv::norm(top_middle - bottom_middle);
        angle = std::atan2(top2bottom.y, top2bottom.x);
        angle_error = std::abs(angle - CV_PI / 2);
        ratio = length / width;
    }
    Armor::Armor(const Lightbar & left, const Lightbar & right) : left(left), right(right) {
        color = left.color;
        center = (left.center + right.center) / 2;
        points.emplace_back(left.top_middle);
        points.emplace_back(right.top_middle);
        points.emplace_back(right.bottom_middle);
        points.emplace_back(left.bottom_middle);

        auto top_left2right = right.top_middle - left.top_middle;
        auto bottom_left2right = right.bottom_middle - left.bottom_middle;
        top_angle_error = std::abs(std::atan2(top_left2right.y, top_left2right.x));
        light_angle_error = std::abs(left.angle / right.angle);
        bottom_angle_error = std::abs(std::atan2(bottom_left2right.y, bottom_left2right.x));
        top_bottom_angle_error = std::abs(std::abs(top_angle_error) - std::abs(bottom_angle_error));
        auto left2right = left.center - right.center;
        auto width = cv::norm(left2right);
        auto max_lightbar_length = std::max(left.length, right.length);
        auto min_lightbar_length = std::min(left.length, right.length);
        ratio = width / max_lightbar_length;
        side_ratio = max_lightbar_length / min_lightbar_length;

        auto roll = std::atan2(left2right.y, left2right.x);
        auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
        auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);
        rectangular_error_diff =  std::abs(left_rectangular_error - right_rectangular_error);
        rectangular_error = std::max(left_rectangular_error, right_rectangular_error);


    }

}
