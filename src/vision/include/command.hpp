#ifndef POTENTIAL_VISION_COMMAND_HPP
#define POTENTIAL_VISION_COMMAND_HPP

namespace auto_aim {

// Aimer 输出、Shooter 消费的控制命令。
//
// 当前 yaw/pitch 单位沿用下游协议：角度制。内部很多算法仍使用弧度，后续若统一单位，
// 这个结构是最需要优先明确和保护的接口。
struct Command
{
    // 是否接管云台控制。
    bool control;
    // Shooter 最终给出的开火建议。
    bool shoot;
    double yaw;
    double pitch;
    // Aimer 是否找到了有效击打点；Shooter 用它替代读取 Aimer 内部 debug 状态。
    bool aim_point_valid;
};

}  // namespace auto_aim

#endif  // POTENTIAL_VISION_COMMAND_HPP
