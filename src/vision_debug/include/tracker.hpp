#ifndef POTENTIAL_VISION_TRACKER_HPP
#define POTENTIAL_VISION_TRACKER_HPP
#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "target.hpp"

namespace auto_aim {
    struct TrackedArmorIdentity {
        ArmorName name;
        ArmorType type;
    };

    // 追踪模块：维护目标状态机和整车 EKF，不负责颜色过滤、候选排序或 PnP 解算。
    //
    // Pipeline 会根据 Tracker 暴露的状态决定本帧需要解算哪些装甲板，从而避免全量 PnP。
    class Tracker {
    public:
        explicit Tracker(const std::string & config_path);
        // 输入装甲板必须已在 Pipeline 中完成颜色过滤、优先级排序和必要解算。
        std::list<Target> track(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t, const Color &enemy_color, bool use_enemy_color);
        bool needs_target_initialization(std::chrono::steady_clock::time_point t) const;
        std::optional<TrackedArmorIdentity> tracked_armor_identity() const;
    private:
        Color enemy_color_;
        int min_detect_count_;
        int max_temp_lost_count_;
        int detect_count_;
        int temp_lost_count_;
        int outpost_max_temp_lost_count_;
        int normal_temp_lost_count_;
        std::string state_, pre_state_;
        Target target_;
        std::chrono::steady_clock::time_point last_timestamp_;
        ArmorPriority omni_target_priority_;
        void state_machine(bool found);
        bool update_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t);
        bool set_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t);
    };
}

#endif //POTENTIAL_VISION_TRACKER_HPP
