#ifndef POTENTIAL_AUTO_AIM_PIPELINE_HPP
#define POTENTIAL_AUTO_AIM_PIPELINE_HPP

#include <memory>
#include <string>
#include <chrono>

#include "aimer.hpp"
#include "detector.hpp"
#include "shooter.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "vision_types.hpp"

namespace auto_aim {

// 自瞄算法编排层。
//
// 这个类是 ROS 层和算法模块之间的分界线：
// - ROS 层只负责把消息整理成 VisionFrameInput；
// - Pipeline 负责按固定顺序调用检测、解算、追踪、瞄准、射击；
// - 各算法模块不直接订阅/发布 ROS 话题。
class AutoAimPipeline {
public:
    explicit AutoAimPipeline(std::string config_path);

    VisionFrameOutput process(const VisionFrameInput & input);

private:
    // 统一候选准备逻辑：颜色过滤、中心距离排序、目标优先级排序。
    // Tracker 默认接收已经准备好的候选，避免 Tracker 再耦合颜色和选择策略。
    void prepare_armors(std::vector<Armor> & armors, Color enemy_color) const;

    // 按 Tracker 状态选择需要 PnP 的装甲板，避免每帧全量 solve 带来额外开销。
    void solve_required_armors(
        std::vector<Armor> & armors, std::chrono::steady_clock::time_point timestamp);

    // 将内部 Target 状态转换为调试/外部使用的 ROS 消息。
    rm_interfaces::msg::Target make_target_msg(const Target & target) const;

    // 根据 EKF 目标状态生成重投影点，用于调试预测装甲板位置。
    std::vector<ReprojectedArmor> make_reprojected_armors(
        const Armor & detected_armor, Target & target);

    std::unique_ptr<Detector> detector_;
    std::unique_ptr<Solver> solver_;
    std::unique_ptr<Tracker> tracker_;
    std::unique_ptr<Aimer> aimer_;
    std::unique_ptr<Shooter> shooter_;
};

}  // namespace auto_aim

#endif  // POTENTIAL_AUTO_AIM_PIPELINE_HPP
