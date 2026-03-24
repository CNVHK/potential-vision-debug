#include "shooter.hpp"
#include <yaml-cpp/yaml.h>
#include "math_tools.hpp"

namespace auto_aim
{
    Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
    {
        YAML::Node config = YAML::LoadFile(config_path);
        first_tolerance_ = config["first_tolerance"].as<double>();   
        second_tolerance_ = config["second_tolerance"].as<double>(); 
        judge_distance_ = config["judge_distance"].as<double>();
        auto_fire_ = config["auto_fire"].as<bool>();
    }

    bool Shooter::shoot(
      const Command & command, const auto_aim::Aimer & aimer,
      const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos) {
        if (!command.control || targets.empty() || !auto_fire_) return false;
        auto target_x = targets.front().ekf_x()[0];
        auto target_y = targets.front().ekf_x()[2];
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
          get_deg_diff(last_command_.yaw , command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
          get_deg_diff(gimbal_pos[0] * (180.0 / CV_PI) , last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
          aimer.debug_aim_point.valid) {
            last_command_ = command;
            return true;
        }
        // if (
        //   std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
        //   std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
        //   aimer.debug_aim_point.valid) {
        //     last_command_ = command;
        //     return true;
        //   }

        last_command_ = command;
        return false;
    }

}  // namespace auto_aim