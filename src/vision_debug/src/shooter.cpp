#include "shooter.hpp"
#include <yaml-cpp/yaml.h>
#include "math_tools.hpp"

namespace auto_aim
{
    Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0, false}
    {
        YAML::Node config = YAML::LoadFile(config_path);
        first_tolerance_ = config["first_tolerance"].as<double>();   
        second_tolerance_ = config["second_tolerance"].as<double>(); 
        judge_distance_ = config["judge_distance"].as<double>();
        auto_fire_ = config["auto_fire"].as<bool>();
    }

    bool Shooter::shoot(
      const Command & command, const std::list<auto_aim::Target> & targets,
      const Eigen::Vector3d & gimbal_pos) {
        if (!command.control || targets.empty() || !auto_fire_) return false;
        auto target_x = targets.front().ekf_x()[0];
        auto target_y = targets.front().ekf_x()[2];
        // 远距离允许更大的角度容差，近距离收紧容差减少误击。
        auto tolerance = std::sqrt(tool::square(target_x) + tool::square(target_y)) > judge_distance_
                           ? second_tolerance_
                           : first_tolerance_;

        auto get_deg_diff = [](double a, double b) {
          double diff = std::fmod(a - b, 360.0);
          if (diff > 180.0) diff -= 360.0;
          if (diff < -180.0) diff += 360.0;
          return std::abs(diff);
        };
        if (
          // 当前云台接近上一帧稳定命令，且新命令没有突变时，才建议开火。
          get_deg_diff(last_command_.yaw , command.yaw) < tolerance * 2 &&  // 命令突变时不射击
          get_deg_diff(gimbal_pos[0] * (180.0 / CV_PI) , last_command_.yaw) < tolerance &&
          command.aim_point_valid) {
            last_command_ = command;
            return true;
        }

        last_command_ = command;
        return false;
    }

}  // namespace auto_aim
