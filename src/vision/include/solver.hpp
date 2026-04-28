#ifndef POTENTIAL_VISION_SOLVER_HPP
#define POTENTIAL_VISION_SOLVER_HPP
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include  <vector>
#include  <cmath>
#include "armor.hpp"


namespace auto_aim {
    // 3D 解算模块：把 Detector 输出的 2D 装甲板角点解算到云台系/世界系。
    //
    // solve() 会原地写入 Armor 的 xyz/ypr/ypd 字段。当前仍复用 Armor 作为 2D+3D 载体，
    // 后续如果拆 DetectedArmor/SolvedArmor，需要优先从这里切开。
    class Solver {
    public:
        explicit Solver(std::string &config_path);
        Eigen::Matrix3d R_gimbal2world() const;
        void set_R_gimbal2world(const Eigen::Quaterniond & q);
        void solve(Armor &armor);
        std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name);
    private:
        cv::Mat camera_matrix_;
        cv::Mat distort_coeffs_;
        Eigen::Matrix3d R_gimbal2imubody_;
        Eigen::Matrix3d R_camera2gimbal_;
        Eigen::Vector3d t_camera2gimbal_;
        Eigen::Matrix3d R_gimbal2world_;
        void optimize_yaw(Armor & armor);
        double armor_reprojection_error(Armor & armor, double yaw, const double & inclined);
    };
}
#endif //POTENTIAL_VISION_SOLVER_HPP
