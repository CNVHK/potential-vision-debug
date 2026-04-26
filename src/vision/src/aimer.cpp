#include <yaml-cpp/yaml.h>
#include <cmath>
#include <vector>
#include "aimer.hpp"
#include "math_tools.hpp"
#include "trajectory.hpp"

namespace auto_aim {
    Aimer::Aimer(const std::string & config_path) :left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt) {
        YAML::Node config = YAML::LoadFile(config_path);
        yaw_offset_ = config["yaw_offset"].as<double>() / 57.3;        // degree to rad
        pitch_offset_ = config["pitch_offset"].as<double>() / 57.3;    // degree to rad
        comming_angle_ = config["comming_angle"].as<double>() / 57.3;  // degree to rad
        leaving_angle_ = config["leaving_angle"].as<double>() / 57.3;  // degree to rad
        high_speed_delay_time_ = config["high_speed_delay_time"].as<double>();
        low_speed_delay_time_ = config["low_speed_delay_time"].as<double>();
        decision_speed_ = config["decision_speed"].as<double>();
        auto t_gimbal2gun_data = config["t_gimbal2gun"].as<std::vector<double>>();
        t_gimbal2gun_ = Eigen::Matrix<double, 3, 1>(t_gimbal2gun_data.data());
    }

    Command Aimer::aim(std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
            Eigen::Matrix3d R_gimbal2world, bool to_now) {
        if (targets.empty()) return {false, false, 0, 0, false};
        auto target = targets.front();
        auto ekf = target.ekf();
        double delay_time = target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
        auto future = timestamp;
        if (to_now) {
            double dt;
            dt = tool::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
            future += std::chrono::microseconds(static_cast<int64_t>(dt * 1e6));
            target.predict(future);
        }
        else {
            auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
            future += std::chrono::microseconds(static_cast<int64_t>(dt * 1e6));
            target.predict(future);
        }
        auto aim_point0 = choose_aim_point(target);
        debug_aim_point = aim_point0;
        if (!aim_point0.valid) {
            return {false, false, 0, 0, false};
        }
        Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
        auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
        tool::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
        if (trajectory0.unsolvable) {
            debug_aim_point.valid = false;
            return {false, false, 0, 0, false};
        }
        bool converged = false;
        double prev_fly_time = trajectory0.fly_time;
        tool::Trajectory current_traj = trajectory0;
        std::vector<Target> iteration_target(10, target);
        Eigen::Vector3d gun_offset_in_world = R_gimbal2world * t_gimbal2gun_;
        for (int iter = 0; iter < 10; ++iter) {
            auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
            iteration_target[iter].predict(predict_time);
            auto aim_point = choose_aim_point(iteration_target[iter]);
            debug_aim_point = aim_point;
            if (!aim_point.valid) {
                return {false, false, 0, 0, false};
            }

            // 计算新弹道
            Eigen::Vector3d xyz_gimbal = aim_point.xyza.head(3);
            Eigen::Vector3d xyz_gun = xyz_gimbal - gun_offset_in_world;
            double d = std::sqrt(xyz_gun.x() * xyz_gun.x() + xyz_gun.y() * xyz_gun.y());
            current_traj = tool::Trajectory(bullet_speed, d, xyz_gun.z());

            // 检查弹道是否可解
            if (current_traj.unsolvable) {
                debug_aim_point.valid = false;
                return {false, false, 0, 0, false};
            }

            // 检查收敛条件
            if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
                converged = true;
                break;
            }
            prev_fly_time = current_traj.fly_time;
        }
        Eigen::Vector3d final_xyz_gimbal = debug_aim_point.xyza.head(3);
        Eigen::Vector3d final_xyz_gun = final_xyz_gimbal - gun_offset_in_world;
        //gimbal_to_gun
        // double yaw = -std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
        // double pitch = (current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
        double yaw = std::atan2(final_xyz_gun.y(), final_xyz_gun.x()) * (180.0 / CV_PI) + yaw_offset_ ;
        double pitch = current_traj.pitch * (180.0 / CV_PI) + pitch_offset_;
        return {true, false, yaw, pitch, debug_aim_point.valid};
    }
    AimPoint Aimer::choose_aim_point(Target & target) {
        Eigen::VectorXd ekf_x = target.ekf_x();
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        auto armor_num = armor_xyza_list.size();
        if (!target.jumped) return {true, armor_xyza_list[0]};
        auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);


        // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
        std::vector<double> delta_angle_list;
        for (int i = 0; i < armor_num; i++) {
            auto delta_angle = tool::limit_rad(armor_xyza_list[i][3] - center_yaw);
            delta_angle_list.emplace_back(delta_angle);
        }

        if (std::abs(target.ekf_x()[7]) <= 2 && target.name != ArmorName::outpost) {
            std::vector<int> id_list;
            for (int i = 0; i < armor_num; i++) {
                if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
                id_list.push_back(i);
            }
            if (id_list.empty()) {
                return {false, armor_xyza_list[0]};
            }
            if (id_list.size() > 1) {
                int id0 = id_list[0], id1 = id_list[1];
                if (lock_id_ != id0 && lock_id_ != id1)
                    lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

                return {true, armor_xyza_list[lock_id_]};
            }
            lock_id_ = -1;
            return {true, armor_xyza_list[id_list[0]]};
        }

        double coming_angle, leaving_angle;
        if (target.name == ArmorName::outpost) {
            coming_angle = 70 / 57.3;
            leaving_angle = 30 / 57.3;
        } else {
            coming_angle = comming_angle_;
            leaving_angle = leaving_angle_;
        }
        for (int i = 0; i < armor_num; i++) {
            if (std::abs(delta_angle_list[i]) > coming_angle) continue;
            if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
            if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
        }

        return {false, armor_xyza_list[0]};

    }

}
