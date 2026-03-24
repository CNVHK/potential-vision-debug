#ifndef POTENTIAL_VISION_AIMER_HPP
#define POTENTIAL_VISION_AIMER_HPP
#include <Eigen/Dense>
#include <chrono>
#include <list>
#include "target.hpp"

namespace auto_aim {

    struct AimPoint {
        bool valid;
        Eigen::Vector4d xyza;
    };

    struct Command
    {
        bool control;
        bool shoot;
        double yaw;
        double pitch;
    };


    class Aimer {
    public:
        AimPoint debug_aim_point;
        explicit Aimer(const std::string & config_path);
        Command aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
            bool to_now = true);

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
        AimPoint choose_aim_point(Target & target);
    };

}
#endif //POTENTIAL_VISION_AIMER_HPP