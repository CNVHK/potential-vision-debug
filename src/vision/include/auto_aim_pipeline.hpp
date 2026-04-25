#ifndef POTENTIAL_AUTO_AIM_PIPELINE_HPP
#define POTENTIAL_AUTO_AIM_PIPELINE_HPP

#include <memory>
#include <string>

#include "aimer.hpp"
#include "detector.hpp"
#include "shooter.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "vision_types.hpp"

namespace auto_aim {

class AutoAimPipeline {
public:
    explicit AutoAimPipeline(std::string config_path);

    VisionFrameOutput process(const VisionFrameInput & input);

private:
    rm_interfaces::msg::Target make_target_msg(const Target & target) const;
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
