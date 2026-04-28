#ifndef POTENTIAL_VISION_AIMER_HPP
#define POTENTIAL_VISION_AIMER_HPP
#include <Eigen/Dense>
#include <chrono>
#include <list>
#include "command.hpp"
#include "target.hpp"

namespace auto_aim {

    struct AimPoint {
        bool valid;
        Eigen::Vector4d xyza;
    };

    // 瞄准模块：根据追踪目标、弹速和云台姿态预测击打点，并输出云台 yaw/pitch。
    //
    // Aimer 不判断是否真正开火，只负责给出可打的角度和 aim_point_valid。
    class Aimer {
    public:
        AimPoint debug_aim_point;
        explicit Aimer(const std::string & config_path);
        Command aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
            Eigen::Matrix3d R, bool to_now = true);
    private:
        double yaw_offset_;
        std::optional<double> left_yaw_offset_, right_yaw_offset_;
        double pitch_offset_;
        double comming_angle_;
        double leaving_angle_;
        double lock_id_ = -1;
        double high_speed_delay_time_;
        double low_speed_delay_time_;
        double decision_speed_;
        Eigen::Vector3d t_gimbal2gun_;
        AimPoint choose_aim_point(Target & target);
    };

}
#endif //POTENTIAL_VISION_AIMER_HPP
