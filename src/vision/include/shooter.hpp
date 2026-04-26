#ifndef POTENTIAL_VISION_SHOOTER_HPP
#define POTENTIAL_VISION_SHOOTER_HPP
#include <string>

#include "command.hpp"
#include "target.hpp"

namespace auto_aim
{
    class Shooter
    {
    public:
        Shooter(const std::string & config_path);

        bool shoot(
          const Command & command,
          const std::list<Target> & targets, const Eigen::Vector3d & gimbal_pos);

    private:
        Command last_command_;
        double judge_distance_;
        double first_tolerance_;
        double second_tolerance_;
        bool auto_fire_;
    };
}  // namespace auto_aim
#endif //POTENTIAL_VISION_SHOOTER_HPP
