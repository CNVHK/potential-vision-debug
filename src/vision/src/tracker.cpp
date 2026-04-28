#include <yaml-cpp/yaml.h>
#include <numeric>
#include "tracker.hpp"
#include "math_tools.hpp"
namespace auto_aim {
    Tracker::Tracker(const std::string & config_path) :
      detect_count_(0),
      temp_lost_count_(0),
      state_{"lost"},
      pre_state_{"lost"},
      last_timestamp_(std::chrono::steady_clock::now()),
      omni_target_priority_{ArmorPriority::fifth} {
        YAML::Node config = YAML::LoadFile(config_path);
        enemy_color_ = (config["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
        min_detect_count_ = config["min_detect_count"].as<int>();
        max_temp_lost_count_ = config["max_temp_lost_count"].as<int>();
        outpost_max_temp_lost_count_ = config["outpost_max_temp_lost_count"].as<int>();
        normal_temp_lost_count_ = max_temp_lost_count_;
    }

    bool Tracker::needs_target_initialization(std::chrono::steady_clock::time_point t) const {
        auto dt = tool::delta_time(t, last_timestamp_);
        // 与 track() 内部超时规则保持一致；Pipeline 用它决定本帧是否只解算首个候选。
        return state_ == "lost" || (state_ != "lost" && dt > 0.1);
    }

    std::optional<TrackedArmorIdentity> Tracker::tracked_armor_identity() const {
        // 只暴露目标身份，不暴露完整 Target，避免 Pipeline 依赖 EKF 内部状态。
        if (state_ == "lost") return std::nullopt;
        return TrackedArmorIdentity{target_.name, target_.armor_type};
    }

    std::list<Target> Tracker::track(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t, const Color &, bool) {
        auto dt = tool::delta_time(t, last_timestamp_);
        last_timestamp_ = t;
        if (state_ != "lost" && dt > 0.1) {
            // 图像时间间隔过长时，旧预测不再可信，直接回到 lost 重新初始化。
            state_ = "lost";
        }
        bool found;
        if (state_ == "lost") {
            found = set_target(armors, t);
        }
        else {
            target_.predict(t);
            found = update_target(armors, t);
        }
        state_machine(found);
        if (state_ != "lost" && target_.diverged()) {
            state_ = "lost";
            return {};
        }

        if (std::accumulate(
            target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
            (0.4 * target_.ekf().window_size)) {
                // 连续观测异常说明匹配或模型已经不可靠，主动丢失目标。
                state_ = "lost";
                return {};
        }

        if (state_ == "lost") return {};
        std::list<Target> targets = {target_};
        return targets;

    }


    void Tracker::state_machine(bool found) {
        if (state_ == "lost") {
            if (!found) return;
            state_ = "detecting";
            detect_count_ = 1;
        }

        else if (state_ == "detecting") {
            if (found) {
                detect_count_++;
                if (detect_count_ >= min_detect_count_) state_ = "tracking";
            } else {
                detect_count_ = 0;
                state_ = "lost";
            }
        }

        else if (state_ == "tracking") {
            if (found) return;

            temp_lost_count_ = 1;
            state_ = "temp_lost";
        }

        else if (state_ == "temp_lost") {
            if (found) {
                state_ = "tracking";
            } else {
                temp_lost_count_++;
                if (target_.name == ArmorName::outpost)
                    // 前哨站短暂丢失更常见，允许更长的临时丢失窗口。
                        max_temp_lost_count_ = outpost_max_temp_lost_count_;
                else
                    max_temp_lost_count_ = normal_temp_lost_count_;

                if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
            }
        }

    }
    bool Tracker::set_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t) {
        if (armors.empty()) return false;
        // Pipeline 已经完成排序，这里直接取最高优先级候选初始化目标。
        auto & armor = armors.front();
        auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
        if (is_balance) {
            Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
            target_ = Target(armor, t, 0.2, 4, P0_dig);
        }
        else if (armor.name == ArmorName::outpost) {
            Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 0.01, 0, 0}};
        }

        else if (armor.name == ArmorName::base) {
            Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
        }

        else {
            Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 1, 100, 1, 1, 1}};
            target_ = Target(armor, t, 0.2, 4, P0_dig);
        }

        return true;
    }

    bool Tracker::update_target(std::vector<Armor> & armors, std::chrono::steady_clock::time_point t) {
        int found_count = 0;
        for (auto & armor : armors) {
            // 追踪阶段只接受同 ID、同大小类型的装甲板更新当前目标。
            if (armor.name != target_.name or armor.type != target_.armor_type) continue;
            found_count++;
            target_.update(armor);
        }
        if (found_count == 0) return false;
        return true;
    }

}
