#include <yaml-cpp/yaml.h>
#include "solver.hpp"
#include "math_tools.hpp"
namespace auto_aim {
    constexpr double LIGHTBAR_LENGTH = 47e-3;     // 单位：m
    constexpr double BIG_ARMOR_WIDTH = 230e-3;    // 单位：m
    constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 单位：m

    const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
      {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
      {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
    const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
      {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
      {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
      {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};

    Solver::Solver(std::string &config_path) :R_gimbal2world_(Eigen::Matrix3d::Identity()){
        YAML::Node config = YAML::LoadFile(config_path);
        auto R_gimbal2imubody_data = config["R_gimbal2imubody_data"].as<std::vector<double>>();
        auto R_camera2gimbal_data = config["R_camera2gimbal"].as<std::vector<double>>();
        auto t_camera2gimbal_data = config["t_camera2gimbal"].as<std::vector<double>>();
        auto camera_matrix_data = config["camera_matrix_data"].as<std::vector<double>>();
        auto distort_coeffs_data = config["distortion_coeffs_data"].as<std::vector<double>>();

        R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
        R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
        t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());
        Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
        Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
        cv::eigen2cv(camera_matrix, camera_matrix_);
        cv::eigen2cv(distort_coeffs, distort_coeffs_);
    }


    Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }

    void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q) {
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
    }

    void Solver::solve(Armor & armor) {
        const auto &object_point = (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_point,
            armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,cv::SOLVEPNP_IPPE);

        Eigen::Vector3d xyz_in_camera;
        cv::cv2eigen(tvec, xyz_in_camera);
        armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
        armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

        cv::Mat rmat;
        cv::Rodrigues(rvec, rmat);
        Eigen::Matrix3d R_armor2camera;
        cv::cv2eigen(rmat, R_armor2camera);

        Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
        Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
        armor.ypr_in_gimbal = tool::eulers(R_armor2gimbal, 2, 1, 0);
        armor.ypr_in_world = tool::eulers(R_armor2world, 2, 1, 0);
        armor.ypd_in_world = tool::xyz2ypd(armor.xyz_in_world);

        auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
        if (is_balance) return;

        optimize_yaw(armor);
    }

    void Solver::optimize_yaw(Armor & armor) {
        Eigen::Vector3d gimbal_ypr = tool::eulers(R_gimbal2world_, 2, 1, 0);
        constexpr double SEARCH_RANGE = 140;
        auto yaw0 = tool::limit_rad(gimbal_ypr[0] - SEARCH_RANGE / 2 * CV_PI / 180.0);
        auto min_error = 1e10;
        auto best_yaw = armor.ypr_in_world[0];

        for (int i=0; i < SEARCH_RANGE; i++) {
            double yaw = tool::limit_rad(yaw0 + i * CV_PI / 180.0);
            auto error = armor_reprojection_error(armor, yaw, (i - SEARCH_RANGE / 2) * CV_PI / 180.0);

            if (error < min_error) {
                min_error = error;
                best_yaw = yaw;
            }
        }
        constexpr double FIN_SEARCH_RANGE = 100;
        auto fin_yaw0 = tool::limit_rad(best_yaw - 0.03 * FIN_SEARCH_RANGE / 2 * CV_PI / 180.0);
        for (int i=0; i <FIN_SEARCH_RANGE; i++) {
            double yaw = tool::limit_rad(fin_yaw0 + 0.03 * i * CV_PI / 180.0);
            auto error = armor_reprojection_error(armor, yaw, 0.03 * (i - FIN_SEARCH_RANGE / 2) * CV_PI / 180.0);
            if (error < min_error) {
                min_error = error;
                best_yaw = yaw;
            }
        }
        armor.yaw_raw = armor.ypr_in_world[0];
        armor.ypr_in_world[0] = best_yaw + 0.105; // 约 6 度经验补偿
    }



    std::vector<cv::Point2f> Solver::reproject_armor(const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) {
        auto sin_yaw = std::sin(yaw);
        auto cos_yaw = std::cos(yaw);

        auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
        auto sin_pitch = std::sin(pitch);
        auto cos_pitch = std::cos(pitch);

        // clang-format off
        const Eigen::Matrix3d R_armor2world {
        {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
        {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
        {         -sin_pitch,        0,           cos_pitch}
        };
        // clang-format on

        // 从世界系反推装甲板到相机系的位姿。
        const Eigen::Vector3d & t_armor2world = xyz_in_world;
        Eigen::Matrix3d R_armor2camera =
          R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
        Eigen::Vector3d t_armor2camera =
          R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

        // 转成 OpenCV 重投影所需的 rvec/tvec。
        cv::Vec3d rvec;
        cv::Mat R_armor2camera_cv;
        cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
        cv::Rodrigues(R_armor2camera_cv, rvec);
        cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

        std::vector<cv::Point2f> image_points;
        const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
        cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
        return image_points;
    }

    double Solver::armor_reprojection_error(Armor & armor, double yaw, const double & inclined) {
        auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
        auto error = 0.0;
        for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
        return error;
    }



}
