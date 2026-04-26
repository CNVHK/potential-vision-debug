#include "auto_aim_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include "math_tools.hpp"

namespace auto_aim {

AutoAimPipeline::AutoAimPipeline(std::string config_path)
{
    detector_ = std::make_unique<Detector>(config_path, false);
    solver_ = std::make_unique<Solver>(config_path);
    tracker_ = std::make_unique<Tracker>(config_path);
    aimer_ = std::make_unique<Aimer>(config_path);
    shooter_ = std::make_unique<Shooter>(config_path);
}

VisionFrameOutput AutoAimPipeline::process(const VisionFrameInput & input)
{
    VisionFrameOutput output;

    solver_->set_R_gimbal2world(input.gimbal_orientation);
    output.gimbal_ypr = tool::eulers(solver_->R_gimbal2world(), 2, 1, 0);

    output.armors = detector_->detect_armors(input.image, input.enemy_color);
    prepare_armors(output.armors, input.enemy_color);
    solve_required_armors(output.armors, input.steady_stamp);
    output.targets = tracker_->track(output.armors, input.steady_stamp, input.enemy_color, false);
    output.command =
        aimer_->aim(output.targets, input.steady_stamp, input.bullet_speed, solver_->R_gimbal2world());
    output.command.shoot = shooter_->shoot(output.command, output.targets, output.gimbal_ypr);

    output.cboard.is_detected = output.command.control;
    output.cboard.yaw = output.command.yaw;
    output.cboard.pitch = output.command.pitch;
    output.cboard.robot_id = 0;
    output.cboard.is_fire = output.command.shoot;
    output.cboard.wr = 0.0;
    output.cboard.distance = 0.0;

    if (output.targets.empty()) {
        return output;
    }

    auto target = output.targets.front();
    auto ekf_x = target.ekf_x();
    output.cboard.wr = -ekf_x[7] * (60.0 / (2.0 * CV_PI));

    if (!output.armors.empty()) {
        auto & armor = output.armors.front();
        output.cboard.robot_id = armor.name + 1;
        output.cboard.distance = armor.ypd_in_world[2];
        output.target_msg = make_target_msg(target);
        output.reprojected_armors = make_reprojected_armors(armor, target);
    }

    return output;
}

void AutoAimPipeline::prepare_armors(std::vector<Armor> & armors, Color enemy_color) const
{
    armors.erase(
        std::remove_if(
            armors.begin(), armors.end(),
            [&](const Armor & armor) { return armor.color != enemy_color; }),
        armors.end());

    std::sort(armors.begin(), armors.end(), [](const Armor & a, const Armor & b) {
        cv::Point2f img_center(1440 / 2, 1080 / 2);
        auto distance_1 = cv::norm(a.center - img_center);
        auto distance_2 = cv::norm(b.center - img_center);
        return distance_1 < distance_2;
    });

    std::stable_sort(armors.begin(), armors.end(), [](const Armor & a, const Armor & b) {
        return a.priority < b.priority;
    });
}

void AutoAimPipeline::solve_required_armors(
    std::vector<Armor> & armors, std::chrono::steady_clock::time_point timestamp)
{
    if (armors.empty()) {
        return;
    }

    if (tracker_->needs_target_initialization(timestamp)) {
        solver_->solve(armors.front());
        return;
    }

    auto identity = tracker_->tracked_armor_identity();
    if (!identity.has_value()) {
        return;
    }

    for (auto & armor : armors) {
        if (armor.name == identity->name && armor.type == identity->type) {
            solver_->solve(armor);
        }
    }
}

rm_interfaces::msg::Target AutoAimPipeline::make_target_msg(const Target & target) const
{
    rm_interfaces::msg::Target msg;
    auto ekf_x = target.ekf_x();
    msg.x = ekf_x[0];
    msg.vx = ekf_x[1];
    msg.y = ekf_x[2];
    msg.vy = ekf_x[3];
    msg.z = ekf_x[4];
    msg.vz = ekf_x[5];
    msg.yaw = ekf_x[6];
    msg.vyaw = ekf_x[7];
    msg.r = ekf_x[8];
    msg.dr = 0.0;
    msg.dz = 0.0;
    return msg;
}

std::vector<ReprojectedArmor> AutoAimPipeline::make_reprojected_armors(
    const Armor & detected_armor, Target & target)
{
    std::vector<ReprojectedArmor> reprojected_armors;
    auto armor_xyza_list = target.armor_xyza_list();
    auto min_error = 1e10;
    Eigen::Vector4d detector_xyza;

    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        auto yaw_error = std::abs(detected_armor.ypr_in_world[0] - xyza[3]);
        if (yaw_error < min_error) {
            min_error = yaw_error;
            detector_xyza = xyza;
        }
    }

    for (const Eigen::Vector4d & xyza : armor_xyza_list) {
        if (xyza == detector_xyza) {
            continue;
        }
        reprojected_armors.push_back(
            {solver_->reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name)});
    }

    return reprojected_armors;
}

}  // namespace auto_aim
