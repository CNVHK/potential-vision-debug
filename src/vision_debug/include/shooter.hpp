#ifndef POTENTIAL_VISION_SHOOTER_HPP
#define POTENTIAL_VISION_SHOOTER_HPP
#include <string>

#include "command.hpp"
#include "target.hpp"

namespace auto_aim
{
    // 开火判定模块：在 Aimer 给出角度后，根据角度突变、当前云台角和目标距离决定是否开火。
    //
    // Shooter 只消费 Command，不读取 Aimer 内部状态，便于之后替换瞄准策略或单独测试开火逻辑。
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
