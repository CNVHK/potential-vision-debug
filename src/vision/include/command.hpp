#ifndef POTENTIAL_VISION_COMMAND_HPP
#define POTENTIAL_VISION_COMMAND_HPP

namespace auto_aim {

struct Command
{
    bool control;
    bool shoot;
    double yaw;
    double pitch;
    bool aim_point_valid;
};

}  // namespace auto_aim

#endif  // POTENTIAL_VISION_COMMAND_HPP
