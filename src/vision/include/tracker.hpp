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

    class Tracker {
    public:
        explicit Tracker(const std::string & config_path);
        // armors must be color-filtered, priority-sorted, and solved by the pipeline before tracking.
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
